#include "config.h"
#include "drive.h"
#include "motors.h"
#include "mpu6050.h"
#include "sonar.h"
#include "heading.h"
#include "timer.h"
#include "debug.h"
#include "power.h"

static center_mode_t s_mode = CENTER_GYRO_ONLY;
static int16_t       s_last_corr = 0;
static int32_t       s_hold_heading = 0;
static drive_debug_t s_dbg;

// Wall-stuck recovery state. See WALL_STUCK_MS in config.h.
static uint8_t  s_emerg_side    = 0; // 0 none, 1 left-near, 2 right-near
static uint32_t s_emerg_since   = 0; // millis() the current side went near
static uint16_t s_emerg_best_cm = 0; // furthest the near wall has got since
                                     // then -- progress resets the stuck timer
static uint8_t  s_recover_phase = 0; // 0 none, 1 reversing, 2 pivoting away
static uint32_t s_recover_until = 0; // millis() deadline for current phase

const drive_debug_t *Drive_Debug(void) { return &s_dbg; }

void Drive_Begin(void) {
    s_last_corr = 0;
    s_mode = CENTER_GYRO_ONLY;
    s_hold_heading = 0;
    s_emerg_side    = 0;
    s_emerg_since   = 0;
    s_emerg_best_cm = 0;
    s_recover_phase = 0;
    s_recover_until = 0;
    Heading_Reset();

    // Breakaway kick: the motors will not start from rest at cruise PWM.
    // RAMPED rather than stepped -- a 0 -> KICK_PWM step draws locked-rotor
    // current with no back-EMF opposing it, and every failed run in the Mode 10
    // logs browned out at exactly such a kick. Same impulse, spread over
    // KICK_MS, roughly half the peak.
    Power_SetActivity(ACT_DRIVE_KICK);
#if KICK_RAMP
    {
        uint32_t t0 = millis();
        uint32_t el;
        while ((el = millis() - t0) < KICK_MS) {
            uint8_t p = (uint8_t)(MOTOR_MIN_PWM +
                (((uint32_t)(KICK_PWM - MOTOR_MIN_PWM) * el) / KICK_MS));
            Motors_Forward(p, p);
            // Sample the rail HERE, densely. The brown-out dip lasts a couple
            // of milliseconds, so the once-per-20ms tick sampler would usually
            // miss it entirely -- and this loop is exactly where the current
            // spike that causes it happens. One conversion is ~0.1 ms.
            Power_Task();
        }
    }
#else
    Motors_Forward(KICK_PWM, KICK_PWM);
    Timer_WaitMs(KICK_MS);
#endif
    Power_SetActivity(ACT_DRIVING);
}

void Drive_Stop(void) {
    Power_SetActivity(ACT_DRIVE_BRAKE);
#if DRIVE_BRAKE_MS > 0
    // ACTIVE BRAKE. Cutting the motors alone leaves the chassis coasting --
    // around 17 cm was observed, enough to put the nose into a wall the
    // controller had correctly decided to stop short of. A brief reverse pulse
    // dumps that momentum, the same way TURN_BRAKE_* already does for pivots.
    //
    // Blocking for DRIVE_BRAKE_MS will overrun one control tick. That is
    // deliberate and harmless HERE specifically: the robot is stopping, so
    // there is no control decision left for this tick to make. Expect one
    // TICK_OVERRUN_WARN_MS count per stop.
    Motors_SetLeft(DIR_REV,  DRIVE_BRAKE_PWM);
    Motors_SetRight(DIR_REV, DRIVE_BRAKE_PWM);
    Timer_WaitMs(DRIVE_BRAKE_MS);
#endif
    Motors_Stop();
    Power_SetActivity(ACT_IDLE);
    s_last_corr = 0;
    // Clear the diagnostic snapshot too. Drive_Tick() stops being called once
    // the caller halts, so without this the telemetry keeps reporting the last
    // live PWM/correction values indefinitely -- making a stopped robot look
    // like it is still driving.
    s_dbg.error_cm  = 0;
    s_dbg.wall_term = 0;
    s_dbg.gyro_term = 0;
    s_dbg.corr      = 0;
    s_dbg.pwm_l     = 0;
    s_dbg.pwm_r     = 0;
    s_dbg.branch    = BRANCH_NORMAL;
}

center_mode_t Drive_Mode(void)        { return s_mode; }
int16_t       Drive_LastCorrection(void) { return s_last_corr; }

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void tick_at(uint8_t base, int16_t gyro_rate) {
    uint8_t l_ok = Sonar_IsValid(SONAR_LEFT);
    uint8_t r_ok = Sonar_IsValid(SONAR_RIGHT);
    uint16_t l_cm = Sonar_Median(SONAR_LEFT);
    uint16_t r_cm = Sonar_Median(SONAR_RIGHT);

    // A side OPENING is not a far wall -- and the difference is the whole
    // reason a junction does not throw the robot into it.
    //
    // Validity alone is not enough here. Sonar_IsValid() only rejects a reading
    // past SONAR_MAX_RANGE_CM, but an opening in a 40 cm maze ranges across the
    // gap to the far wall of the next cell: about 45-55 cm, comfortably in
    // range and therefore "valid". The differential controller then saw
    // error = R - L = 14 - 50 = -36, clamped to full-scale, and steered HARD
    // into the opening -- at every junction, which is exactly where the robot
    // can least afford it.
    //
    // Sonar_IsOpen() is the test that already knows the difference (it is what
    // maze.c classifies junctions with); the centring loop simply never asked.
    // An open side is treated as ABSENT, so the controller falls back to
    // holding station off the one real wall instead of chasing a phantom.
    uint8_t l_wall = l_ok && !Sonar_IsOpen(SONAR_LEFT);
    uint8_t r_wall = r_ok && !Sonar_IsOpen(SONAR_RIGHT);
    // Emergency fires either on the sub-minimum too-close flag OR on a valid
    // reading inside WALL_EMERGENCY_CM. Previously only the too-close flag
    // was checked, so WALL_EMERGENCY_CM was dead code and the robot got no
    // emergency response until a wall was under 3 cm -- by which point it is
    // already touching.
    uint8_t l_near = Sonar_IsTooClose(SONAR_LEFT)  ||
                     (l_ok && l_cm <= WALL_EMERGENCY_CM);
    uint8_t r_near = Sonar_IsTooClose(SONAR_RIGHT) ||
                     (r_ok && r_cm <= WALL_EMERGENCY_CM);
    int16_t error_cm = 0;
    int16_t corr;
    // Steering authority scales with the base speed it is a differential
    // about, so a creeping approach steers as gently as it drives instead of
    // spinning on the spot. At base == DRIVE_BASE_PWM this is exactly the old
    // WALL_MAX_CORRECTION.
    int16_t max_corr = (int16_t)(((int32_t)base * WALL_MAX_CORRECTION_RATIO_PCT) / 100);

    // At a low base speed the floor-preservation below (lift BOTH wheels rather
    // than clip one) quietly cancels the speed reduction: with the floor at
    // MOTOR_MIN_PWM there is very little room under the base, so almost every
    // correction triggers a lift and the mean climbs back to cruise. Measured
    // in three Mode 10 logs: nominal creep 48, actual mean 50.6 / 52.8 / 56.2,
    // peaking at 62. Clamping the correction to the available headroom instead
    // means no lift is ever needed and the mean is honest.
    if (base < DRIVE_MEAN_PRESERVE_BELOW) {
        int16_t room = (int16_t)base - (int16_t)MOTOR_MIN_PWM;
        if (room < 0) room = 0;
        if (max_corr > room) max_corr = room;
    }

    // A one-sided emergency is only meaningful when the other side actually
    // has room to steer into. In a corridor barely wider than the chassis both
    // walls can sit inside the band at once, and hard-steering away from one
    // then drives into the other. Demote to normal centring unless the far
    // side is meaningfully clearer -- centring splits the difference, which is
    // the correct response to being squeezed.
    if (l_near && r_near) {
        /* both near: neither branch below fires anyway */
    } else if (l_near && r_ok && (int16_t)r_cm - (int16_t)l_cm < WALL_EMERG_ASYMMETRY_CM) {
        l_near = 0;
    } else if (r_near && l_ok && (int16_t)l_cm - (int16_t)r_cm < WALL_EMERG_ASYMMETRY_CM) {
        r_near = 0;
    }

    // --- ACTIVE RECOVERY --------------------------------------------------
    // Mid-escape from a wall the ramped steer-away could not break contact
    // with (see below). Keep re-issuing the current phase's motor command
    // until its deadline, then advance. Non-blocking, same tick cadence as
    // everything else -- sonar/gyro/telemetry keep running underneath it,
    // unlike a blocking maneuver which would stall the control loop.
    if (s_recover_phase != 0) {
        s_dbg.branch = BRANCH_EMERG_RECOVER;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0; s_dbg.corr = 0;
        s_dbg.pwm_l = 0; s_dbg.pwm_r = 0;
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;

        if ((int32_t)(millis() - s_recover_until) < 0) return; // motors already set, wait

        if (s_recover_phase == 1) {
            // Reverse pulse done -- pivot away from whichever wall trapped us.
            // Away from LEFT = turn right (Motors_Pivot clockwise), away from
            // RIGHT = turn left, matching the same sign convention the normal
            // one-sided emergency below already uses.
            uint8_t cw = (s_emerg_side == 1) ? 1 : 0;
            Motors_Pivot(cw, WALL_RECOVERY_PIVOT_PWM);
            s_recover_phase = 2;
            s_recover_until = millis() + WALL_RECOVERY_PIVOT_MS;
            Debug_P("  -> clear of wall, pivoting away\r\n");
            return;
        }

        // Recovery complete. Sonar history was taken while pointing somewhere
        // else entirely (backing up, then pivoting) -- throw it away, same as
        // after a turn, and let normal centring re-evaluate fresh next tick.
        Motors_Stop();
        Sonar_Flush();
        s_recover_phase = 0;
        s_recover_until = 0;
        s_emerg_side    = 0;
        s_emerg_since   = 0;
        Debug_P("  -> recovered, resuming normal driving\r\n");
        return;
    }

    // --- EMERGENCY: wall closer than the controller can gracefully handle ---
    // Checked first and unconditionally. At this range the proportional term
    // is too slow, and this must work even while the chassis is rocking --
    // a collision is exactly the moment sonar gets noisiest.
    if (l_near && !r_near) {
        if (s_emerg_side != 1) {
            s_emerg_side = 1;
            s_emerg_since = millis();
            s_emerg_best_cm = l_cm;
            Debug_P("COLLISION COURSE: left wall too close, steering right\r\n");
        }
        // Steering is WORKING if the wall is receding. Restart the clock on
        // real progress so recovery only fires when the distance refuses to
        // improve -- the bare timer fired on slow-but-successful escapes too,
        // and recovery destroys the robot's position in the corridor.
        if ((int16_t)l_cm - (int16_t)s_emerg_best_cm >= WALL_STUCK_IMPROVE_CM) {
            s_emerg_best_cm = l_cm;
            s_emerg_since   = millis();
        }

        // The ramped steer-away below still drives BOTH wheels forward -- it
        // only varies the split. If a chassis corner is physically caught on
        // the wall, that forward-biased differential cannot rotate the robot
        // away: it grinds along the wall at an angle instead of turning off
        // it (observed on hardware). Give it WALL_STUCK_MS to work; past that,
        // stop pushing forward and back off instead.
        if ((millis() - s_emerg_since) > WALL_STUCK_MS) {
            Motors_SetLeft(DIR_REV,  WALL_RECOVERY_REV_PWM);
            Motors_SetRight(DIR_REV, WALL_RECOVERY_REV_PWM);
            s_recover_phase = 1;
            s_recover_until = millis() + WALL_RECOVERY_REV_MS;
            s_dbg.branch = BRANCH_EMERG_RECOVER;
            s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0; s_dbg.corr = 0;
            s_dbg.pwm_l = 0; s_dbg.pwm_r = 0;
            s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
            Debug_P("STUCK on left wall, steering alone didn't clear it -- reversing off\r\n");
            return;
        }

        // Severity ramps with proximity instead of slamming to full
        // differential at the threshold. A hard step at exactly 8cm was
        // measured producing 71 deg/s of yaw, which bounced the robot off
        // one wall straight into the other.
        {
            int16_t sev = (int16_t)WALL_EMERGENCY_CM - (int16_t)l_cm + 1;
            if (sev < 1) sev = 1;
            if (sev > WALL_EMERGENCY_CM) sev = WALL_EMERGENCY_CM;
            s_last_corr = (int16_t)(((int32_t)max_corr * sev) / WALL_EMERGENCY_CM);
        }
        s_dbg.branch = BRANCH_EMERG_L;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = (uint8_t)clamp16(base + s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.pwm_r = (uint8_t)clamp16(base - s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }
    if (r_near && !l_near) {
        if (s_emerg_side != 2) {
            s_emerg_side = 2;
            s_emerg_since = millis();
            s_emerg_best_cm = r_cm;
            Debug_P("COLLISION COURSE: right wall too close, steering left\r\n");
        }
        if ((int16_t)r_cm - (int16_t)s_emerg_best_cm >= WALL_STUCK_IMPROVE_CM) {
            s_emerg_best_cm = r_cm;
            s_emerg_since   = millis();
        }

        if ((millis() - s_emerg_since) > WALL_STUCK_MS) {
            Motors_SetLeft(DIR_REV,  WALL_RECOVERY_REV_PWM);
            Motors_SetRight(DIR_REV, WALL_RECOVERY_REV_PWM);
            s_recover_phase = 1;
            s_recover_until = millis() + WALL_RECOVERY_REV_MS;
            s_dbg.branch = BRANCH_EMERG_RECOVER;
            s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0; s_dbg.corr = 0;
            s_dbg.pwm_l = 0; s_dbg.pwm_r = 0;
            s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
            Debug_P("STUCK on right wall, steering alone didn't clear it -- reversing off\r\n");
            return;
        }

        {
            int16_t sev = (int16_t)WALL_EMERGENCY_CM - (int16_t)r_cm + 1;
            if (sev < 1) sev = 1;
            if (sev > WALL_EMERGENCY_CM) sev = WALL_EMERGENCY_CM;
            s_last_corr = -(int16_t)(((int32_t)max_corr * sev) / WALL_EMERGENCY_CM);
        }
        s_dbg.branch = BRANCH_EMERG_R;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = (uint8_t)clamp16(base + s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.pwm_r = (uint8_t)clamp16(base - s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }

    // Neither side is in emergency this tick -- clear the stuck timer so a
    // fresh contact later gets its own full WALL_STUCK_MS grace period.
    if (s_emerg_side != 0) {
        Debug_P("clear of wall, resuming normal centring\r\n");
    }
    s_emerg_side    = 0;
    s_emerg_since   = 0;
    s_emerg_best_cm = 0;

    // --- Mode selection ---------------------------------------------------
    if (l_wall && r_wall) {
        s_mode = CENTER_BOTH_WALLS;
        // Differential error. Independent of corridor width, which is exactly
        // why this works before the maze dimensions are finalised.
        // Positive => further from the right wall => steer right.
        error_cm = (int16_t)Sonar_Median(SONAR_RIGHT) - (int16_t)Sonar_Median(SONAR_LEFT);
    } else if (l_wall) {
        s_mode = CENTER_LEFT_ONLY;
        // Only here does an ABSOLUTE target matter -- and it has to be what a
        // side sonar really reads when centred, not the corridor half-width.
        // Measured L+R was 28-31 cm where the geometry predicts 24, because the
        // sensor faces sit inboard of the chassis edge. Steering at
        // CORRIDOR_HALF_CM would drive the robot ~5 cm off-centre on purpose.
        error_cm = (int16_t)SIDE_CENTRED_CM - (int16_t)Sonar_Median(SONAR_LEFT);
    } else if (r_wall) {
        s_mode = CENTER_RIGHT_ONLY;
        error_cm = (int16_t)Sonar_Median(SONAR_RIGHT) - (int16_t)SIDE_CENTRED_CM;
    } else {
        s_mode = CENTER_GYRO_ONLY;
        error_cm = 0;
    }

    // --- PD controller ----------------------------------------------------
    // corr > 0 steers RIGHT.  Gyro convention: positive z = turning LEFT,
    // so a positive rate needs a positive (rightward) correction to oppose it.
    //
    // Deadband first. error_cm is a DIFFERENCE of two sonar readings, so 1 cm
    // of quantisation on either side shows up as 2 here. Steering on that
    // produces continuous micro-yaw, and every yaw walks the front beam off
    // whatever lies ahead -- the mechanism behind both the phantom obstacles
    // and the missed ones. Inside the band the wall term is dropped entirely
    // and only gyro damping remains, so the robot tracks straight rather than
    // hunting for a centre it is already at.
    if (error_cm <= WALL_DEADBAND_CM && error_cm >= -WALL_DEADBAND_CM) {
        error_cm = 0;
    }
    corr = (int16_t)(((int32_t)WALL_KP_NUM * error_cm) / WALL_KP_DEN);

    // While rocking, apply the wall term at REDUCED gain rather than dropping
    // it entirely. The original code froze it completely, which meant that
    // sustained motor vibration disabled centring for an entire run and the
    // robot drove into a wall on gyro damping alone.
    s_dbg.branch = BRANCH_NORMAL;
    if (Motion_IsSuspect()) {
        corr = (int16_t)(((int32_t)corr * WALL_ROCK_GAIN_NUM) / WALL_ROCK_GAIN_DEN);
        s_dbg.branch = BRANCH_ROCKING;
    }
    s_dbg.wall_term = corr;

    s_dbg.gyro_term = (int16_t)(((int32_t)WALL_KD_NUM * gyro_rate) / WALL_KD_DEN);
    corr += s_dbg.gyro_term;

    // --- YAW GOVERNOR -----------------------------------------------------
    // Stop ADDING rotation once the chassis is already turning fast in the
    // direction we want to steer. Beyond roughly 20 deg/s the sonars are far
    // enough off-axis that their readings no longer describe the corridor --
    // and the front beam in particular walks off whatever lies ahead, which
    // is how an obstacle at 50cm went undetected. This does not reverse the
    // correction; it just refuses to wind it up further and lets the existing
    // rotation carry the robot back toward centre.
    // Sign convention: positive gyro_rate = turning LEFT, positive corr =
    // steer RIGHT.
    if (corr > 0 && gyro_rate < -YAW_GOVERNOR_LSB) corr = 0;
    if (corr < 0 && gyro_rate >  YAW_GOVERNOR_LSB) corr = 0;

    corr  = clamp16(corr, -max_corr, max_corr);

    s_dbg.error_cm = error_cm;
    s_dbg.corr     = corr;
    s_dbg.l_ok     = l_ok;
    s_dbg.r_ok     = r_ok;

    s_last_corr = corr;
    {
        int16_t l = (int16_t)base + corr;
        int16_t r = (int16_t)base - corr;

        // Preserve the differential when a wheel would fall under the stall
        // floor: lift both rather than clipping one, so the robot keeps
        // steering instead of just slowing down.
        if (l < MOTOR_MIN_PWM) { r += (MOTOR_MIN_PWM - l); l = MOTOR_MIN_PWM; }
        if (r < MOTOR_MIN_PWM) { l += (MOTOR_MIN_PWM - r); r = MOTOR_MIN_PWM; }
        l = clamp16(l, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        r = clamp16(r, MOTOR_MIN_PWM, MOTOR_MAX_PWM);

        s_dbg.pwm_l = (uint8_t)l;
        s_dbg.pwm_r = (uint8_t)r;
        Motors_Forward((uint8_t)l, (uint8_t)r);
    }
}

void Drive_Tick(int16_t gyro_rate) {
    tick_at(DRIVE_BASE_PWM, gyro_rate);
}

