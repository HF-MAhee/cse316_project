#include "config.h"
#include "drive.h"
#include "motors.h"
#include "mpu6050.h"
#include "sonar.h"
#include "heading.h"
#include "timer.h"
#include "debug.h"

static center_mode_t s_mode = CENTER_GYRO_ONLY;
static int16_t       s_last_corr = 0;
static int32_t       s_hold_heading = 0;
static drive_debug_t s_dbg;

// Wall-stuck recovery state. See WALL_STUCK_MS in config.h.
static uint8_t  s_emerg_side    = 0; // 0 none, 1 left-near, 2 right-near
static uint32_t s_emerg_since   = 0; // millis() the current side went near
static uint8_t  s_recover_phase = 0; // 0 none, 1 reversing, 2 pivoting away
static uint32_t s_recover_until = 0; // millis() deadline for current phase

const drive_debug_t *Drive_Debug(void) { return &s_dbg; }

void Drive_Begin(void) {
    s_last_corr = 0;
    s_mode = CENTER_GYRO_ONLY;
    s_hold_heading = 0;
    s_emerg_side    = 0;
    s_emerg_since   = 0;
    s_recover_phase = 0;
    s_recover_until = 0;
    Heading_Reset();

    // Breakaway kick: the motors will not start from rest at cruise PWM.
    Motors_Forward(KICK_PWM, KICK_PWM);
    Timer_WaitMs(KICK_MS);
}

void Drive_Stop(void) {
    Motors_Stop();
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

// One gyro sample on an exact HOLD_TICK_MS schedule, integrating as it goes.
// Returns the bias-corrected rate so the D term does not need a second read.
static int16_t hold_sample(uint32_t *next_ms) {
    gyro_xyz_t g;
    while ((int32_t)(millis() - *next_ms) < 0) { /* wait for the slot */ }
    MPU6050_ReadAll(&g);
    Heading_Add(g.z, HOLD_TICK_MS);
    Motion_Update(&g);
    *next_ms += HOLD_TICK_MS;
    return Gyro_Rate(g.z);
}

int32_t Drive_StraightHold(uint8_t pwm, uint32_t ms, int32_t start_offset_raw) {
    uint32_t t0        = millis();
    uint32_t next_ms   = t0 + HOLD_TICK_MS;
    int32_t  worst     = 0;    // largest |heading error| seen, raw
    int32_t  corr_sum  = 0;    // for the mean -- a persistent non-zero mean is
    uint16_t n_samples = 0;    // the signature of a weaker motor on one side
    uint8_t  decimate  = 0;

    Heading_Reset();

    // Breakaway kick. Tracked, unlike Drive_Begin()'s -- both wheels forward
    // together barely yaws the chassis, but counting it costs nothing and
    // keeps the leg's heading frame honest from the first millisecond.
    Motors_Forward(KICK_PWM, KICK_PWM);
    while ((millis() - t0) < KICK_MS) hold_sample(&next_ms);

    while ((millis() - t0) < ms) {
        int16_t rate = hold_sample(&next_ms);
        // Positive error = rotated LEFT of the target heading, and positive
        // corr steers RIGHT, so both terms take a positive coefficient --
        // same sign convention as Drive_Tick().
        int32_t err_raw   = Heading_Raw() + start_offset_raw;
        int32_t err_deg10 = (err_raw * 10L) / GYRO_LSB_MS_PER_DEGREE;
        int16_t corr;
        int16_t l, r;

        if (err_raw > worst)  worst =  err_raw;
        if (-err_raw > worst) worst = -err_raw;

        corr = (int16_t)(((int32_t)HOLD_KP_NUM * err_deg10) / HOLD_KP_DEN);
        corr = (int16_t)(corr + (((int32_t)HOLD_KD_NUM * rate) / HOLD_KD_DEN));
        corr = clamp16(corr, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);

        l = (int16_t)pwm + corr;
        r = (int16_t)pwm - corr;
        // Same floor preservation as Drive_Tick(): lift both rather than
        // clipping one, so the differential survives.
        if (l < MOTOR_MIN_PWM) { r += (MOTOR_MIN_PWM - l); l = MOTOR_MIN_PWM; }
        if (r < MOTOR_MIN_PWM) { l += (MOTOR_MIN_PWM - r); r = MOTOR_MIN_PWM; }
        l = clamp16(l, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        r = clamp16(r, MOTOR_MIN_PWM, MOTOR_MAX_PWM);

        s_dbg.error_cm  = 0;
        s_dbg.wall_term = 0;
        s_dbg.gyro_term = corr;
        s_dbg.corr      = corr;
        s_dbg.pwm_l     = (uint8_t)l;
        s_dbg.pwm_r     = (uint8_t)r;
        s_dbg.branch    = BRANCH_NORMAL;
        Motors_Forward((uint8_t)l, (uint8_t)r);

        corr_sum += corr;
        if (n_samples < 0xFFFF) n_samples++;

#if HOLD_TRACE
        if (++decimate >= HOLD_SAMPLE_EVERY) {
            decimate = 0;
            Debug_P("L,");
            Debug_Int((int32_t)(millis() - t0));
            Debug_P(",");
            Debug_Int(err_deg10);
            Debug_P(",");
            Debug_Int(rate);
            Debug_P(",");
            Debug_Int(corr);
            Debug_P(",");
            Debug_Int(l);
            Debug_P(",");
            Debug_Int(r);
            Debug_NL();
        }
#else
        (void)decimate;
#endif
    }

    Motors_Stop();

    // How well the leg tracked.
    //   off10   where it finished relative to the heading it was holding
    //   max10   worst excursion on the way there
    //   net10   total rotation over the leg, for the whole-square accounting
    //   mcorr10 MEAN correction in TENTHS of a PWM count (tenths so a small
    //           steady bias does not round away). Near zero = the wheels are
    //           matched and the controller only fights noise. Persistently
    //           non-zero = one motor is systematically weaker and the
    //           controller is holding a constant offset to compensate, which
    //           no amount of gyro tuning will fix.
    //   drop    cumulative; compare leg to leg to confirm the L stream is intact
    Debug_P("  HOLD ");
    Debug_KVF("off10", ((Heading_Raw() + start_offset_raw) * 10L) / GYRO_LSB_MS_PER_DEGREE);
    Debug_KVF("max10", (worst * 10L) / GYRO_LSB_MS_PER_DEGREE);
    Debug_KVF("net10", (Heading_Raw() * 10L) / GYRO_LSB_MS_PER_DEGREE);
    Debug_KVF("mcorr10", n_samples ? ((corr_sum * 10L) / (int32_t)n_samples) : 0);
    Debug_KVF("n", (int32_t)n_samples);
    Debug_KVF("drop", Debug_Dropped());
    Debug_NL();

    return Heading_Raw();
}

void Drive_Tick(int16_t gyro_rate) {
    uint8_t l_ok = Sonar_IsValid(SONAR_LEFT);
    uint8_t r_ok = Sonar_IsValid(SONAR_RIGHT);
    uint16_t l_cm = Sonar_Median(SONAR_LEFT);
    uint16_t r_cm = Sonar_Median(SONAR_RIGHT);
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
            Debug_P("COLLISION COURSE: left wall too close, steering right\r\n");
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
            s_last_corr = (int16_t)(((int32_t)WALL_MAX_CORRECTION * sev) / WALL_EMERGENCY_CM);
        }
        s_dbg.branch = BRANCH_EMERG_L;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = (uint8_t)clamp16(DRIVE_BASE_PWM + s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.pwm_r = (uint8_t)clamp16(DRIVE_BASE_PWM - s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }
    if (r_near && !l_near) {
        if (s_emerg_side != 2) {
            s_emerg_side = 2;
            s_emerg_since = millis();
            Debug_P("COLLISION COURSE: right wall too close, steering left\r\n");
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
            s_last_corr = -(int16_t)(((int32_t)WALL_MAX_CORRECTION * sev) / WALL_EMERGENCY_CM);
        }
        s_dbg.branch = BRANCH_EMERG_R;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = (uint8_t)clamp16(DRIVE_BASE_PWM + s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.pwm_r = (uint8_t)clamp16(DRIVE_BASE_PWM - s_last_corr, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }

    // Neither side is in emergency this tick -- clear the stuck timer so a
    // fresh contact later gets its own full WALL_STUCK_MS grace period.
    if (s_emerg_side != 0) {
        Debug_P("clear of wall, resuming normal centring\r\n");
    }
    s_emerg_side  = 0;
    s_emerg_since = 0;

    // --- Mode selection ---------------------------------------------------
    if (l_ok && r_ok) {
        s_mode = CENTER_BOTH_WALLS;
        // Differential error. Independent of corridor width, which is exactly
        // why this works before the maze dimensions are finalised.
        // Positive => further from the right wall => steer right.
        error_cm = (int16_t)Sonar_Median(SONAR_RIGHT) - (int16_t)Sonar_Median(SONAR_LEFT);
    } else if (l_ok) {
        s_mode = CENTER_LEFT_ONLY;
        // Only here does the absolute corridor width matter.
        error_cm = (int16_t)CORRIDOR_HALF_CM - (int16_t)Sonar_Median(SONAR_LEFT);
    } else if (r_ok) {
        s_mode = CENTER_RIGHT_ONLY;
        error_cm = (int16_t)Sonar_Median(SONAR_RIGHT) - (int16_t)CORRIDOR_HALF_CM;
    } else {
        s_mode = CENTER_GYRO_ONLY;
        error_cm = 0;
    }

    // --- PD controller ----------------------------------------------------
    // corr > 0 steers RIGHT.  Gyro convention: positive z = turning LEFT,
    // so a positive rate needs a positive (rightward) correction to oppose it.
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

    corr  = clamp16(corr, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);

    s_dbg.error_cm = error_cm;
    s_dbg.corr     = corr;
    s_dbg.l_ok     = l_ok;
    s_dbg.r_ok     = r_ok;

    s_last_corr = corr;
    {
        int16_t l = (int16_t)DRIVE_BASE_PWM + corr;
        int16_t r = (int16_t)DRIVE_BASE_PWM - corr;

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
