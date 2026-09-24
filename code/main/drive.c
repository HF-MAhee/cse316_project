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
static drive_debug_t s_dbg;

// Wall-stuck recovery state. See WALL_STUCK_MS in config.h.
static uint8_t  s_emerg_side    = 0; // 0 none, 1 left-near, 2 right-near
static uint32_t s_emerg_since   = 0; // millis() the current side went near
static uint16_t s_emerg_best_cm = 0; // furthest the near wall has got since
                                     // then -- progress resets the stuck timer
static uint8_t  s_recover_phase = 0; // 0 none, 1 reversing, 2 pivoting away
static uint32_t s_recover_until = 0; // millis() deadline for current phase
static int32_t  s_recover_hdg0  = 0; // grid error when the pivot-away began

// Wall alignment (see align_to_walls): the last few fresh pings of each side
// wall, with the gyro heading at the moment each was taken.
typedef struct {
    uint16_t t[WALL_ALIGN_SAMPLES];     // ms, low 16 bits of millis()
    int16_t  y[WALL_ALIGN_SAMPLES];     // wall distance, mm, lever-arm corrected
    int16_t  h[WALL_ALIGN_SAMPLES];     // grid heading error, tenths of a degree
    uint8_t  n, i, seq;
    int16_t  pend[WALL_ALIGN_DELAY];    // estimates waiting to see the wall go on
    uint8_t  pend_n;
    int16_t  last;                      // latest estimate made (not yet applied)
    uint32_t last_ms;                   // ...and when
} wall_fit_t;
static wall_fit_t    s_al[2];                 // 0 = left, 1 = right
static uint32_t      s_begin_ms = 0;

const drive_debug_t *Drive_Debug(void) { return &s_dbg; }

// Keep the heading integrated through a blocking pulse. The brake and the
// drive-off kick are exactly where the chassis yaws hardest (15-30 deg/s in
// the real logs), and both used to be blind waits -- so that yaw never reached
// the heading, and the robot set off on each leg at an angle nobody knew about.
static void track_heading(void) {
    gyro_xyz_t g;
    MPU6050_ReadAll(&g);
    Heading_AddNow(g.z);
}

void Drive_Begin(void) {
    s_last_corr = 0;
    s_mode = CENTER_GYRO_ONLY;
    s_emerg_side    = 0;
    s_emerg_since   = 0;
    s_emerg_best_cm = 0;
    s_recover_phase = 0;
    s_recover_until = 0;
    s_al[0].n       = 0;
    s_al[1].n       = 0;
    s_al[0].pend_n  = 0;
    s_al[1].pend_n  = 0;
    s_begin_ms      = millis();
    Heading_Reset();

    // Breakaway kick: the motors will not start from rest at cruise PWM.
    // RAMPED rather than stepped -- a 0 -> KICK_PWM step draws locked-rotor
    // current with no back-EMF opposing it, and every failed run in the early
    // dead-end test logs browned out at exactly such a kick. Same impulse, spread over
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
            track_heading();
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
    {
        uint32_t t0 = millis();
        while ((millis() - t0) < DRIVE_BRAKE_MS) {
            track_heading();
            Timer_WaitMs(TURN_TICK_MS);
        }
    }
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

// ---------------------------------------------------------------------------
//  Keep the maze grid honest, using the walls.
//
//  The grid heading is integrated gyro, so any error in the gyro's scale
//  accumulates with every degree the robot turns -- and the left-hand rule
//  turns a lot: run 1 of the demo maze nets up to ~1000 degrees, where a 3%
//  sensitivity error (the MPU-6050's datasheet tolerance) would be 30.
//
//  A wall does not drift. Driving along one, the rate the side distance
//  changes IS the robot's angle to the corridor: sin(angle) = lateral speed /
//  forward speed. Over WALL_ALIGN_TICKS that gives an independent, noisy but
//  unbiased measurement of where the grid should be, and the grid is nudged a
//  little toward it each tick. Only while it is safe to believe: a continuous
//  wall (no jump in the reading), no rocking, no hard yaw, not spinning up.
// ---------------------------------------------------------------------------
// One side. Called every tick; does something only when that sonar has a
// fresh ping. Returns 1 with an estimate of how far the grid is off (tenths
// of a degree, grid minus truth) once the last WALL_ALIGN_SAMPLES pings
// describe one straight wall -- and the wall has then gone on for another
// WALL_ALIGN_DELAY pings.
//
// That delay is not caution for its own sake. The last few cm of a wall
// before an opening are its END, and the echo off a wall end grows smoothly
// as the sonar passes it: gently enough to pass the straight-line test, and
// it reads as the robot steering away. In simulation that alone turned the
// grid 4 degrees on one leg. Holding each estimate until the wall is known to
// continue throws exactly those away.
static uint8_t wall_fit(wall_fit_t *w, sonar_id_t id, uint8_t steady, int16_t *off10) {
    uint16_t mm = Sonar_LatestMm(id);
    int16_t  h10, y;
    int32_t  st = 0, sy = 0, sh = 0, stt = 0, sty = 0, res = 0;
    int32_t  tbar, ybar, slope_num, slope_den;
    uint8_t  k;

    if (Sonar_Seq(id) == w->seq) return 0;           // nothing new
    w->seq = Sonar_Seq(id);

    // A corridor wall, read cleanly: in the band a roughly centred robot
    // sees, and continuous with the previous ping. Anything else -- an edge
    // echo, an opening, a far wall -- starts the fit over.
    if (!steady || mm == 0 ||
        mm + WALL_ALIGN_BAND_CM * 10u < SIDE_CENTRED_CM * 10u ||
        mm > (SIDE_CENTRED_CM + WALL_ALIGN_BAND_CM) * 10u) { w->n = 0; w->pend_n = 0; return 0; }

    h10 = (int16_t)Heading_GridErrorTenths();
    // The sonar sits SIDE_SONAR_TO_AXLE_CM ahead of the axle, so yaw swings
    // it sideways by itself: rotating LEFT by a moves the left sonar a*r
    // closer to the left wall. Add that back, so the fit sees the axle.
    // (r mm per tenth-degree = SIDE_SONAR_TO_AXLE_CM * 10 * pi / 1800.)
    y = (int16_t)mm;
    {
        int16_t lever = (int16_t)(((int32_t)h10 * SIDE_SONAR_TO_AXLE_CM * 10L) / 573L);
        y = (id == SONAR_LEFT) ? (int16_t)(y + lever) : (int16_t)(y - lever);
    }
    if (w->n > 0) {
        int16_t prev = w->y[(uint8_t)((w->i + WALL_ALIGN_SAMPLES - 1) % WALL_ALIGN_SAMPLES)];
        if (y - prev > WALL_ALIGN_MAX_JUMP_CM * 10 || prev - y > WALL_ALIGN_MAX_JUMP_CM * 10) {
            w->n = 0;
            w->pend_n = 0;
        }
    }
    w->t[w->i] = (uint16_t)millis();
    w->y[w->i] = y;
    w->h[w->i] = h10;
    w->i = (uint8_t)((w->i + 1) % WALL_ALIGN_SAMPLES);
    if (w->n < WALL_ALIGN_SAMPLES) w->n++;
    if (w->n < WALL_ALIGN_SAMPLES) return 0;

    // Least-squares slope of distance against time, over every ping in the
    // window rather than two endpoints: sub-millimetre-per-ping drift is
    // exactly what is being measured, and each ping is noisy.
    {
        uint16_t t0 = w->t[w->i];                   // oldest (ring is full)
        for (k = 0; k < WALL_ALIGN_SAMPLES; k++) {
            int32_t tk = (int32_t)(uint16_t)(w->t[k] - t0);
            st  += tk;          sy  += w->y[k];      sh += w->h[k];
            stt += tk * tk;     sty += tk * w->y[k];
        }
        tbar = st / WALL_ALIGN_SAMPLES;
        ybar = sy / WALL_ALIGN_SAMPLES;
        slope_num = sty - (st * sy) / WALL_ALIGN_SAMPLES;       // mm*ms
        slope_den = stt - (st * st) / WALL_ALIGN_SAMPLES;       // ms^2
        if (slope_den <= 0) return 0;

        // Straight? The end of a wall bends the curve; reject it.
        for (k = 0; k < WALL_ALIGN_SAMPLES; k++) {
            int32_t tk = (int32_t)(uint16_t)(w->t[k] - t0) - tbar;
            int32_t fit = ybar + (slope_num * tk) / slope_den;
            int32_t e = (int32_t)w->y[k] - fit;
            res += e * e;
        }
        if (res > (int32_t)WALL_ALIGN_MAX_RMS_MM * WALL_ALIGN_MAX_RMS_MM * WALL_ALIGN_SAMPLES) {
            w->pend_n = 0;          // a bend: whatever was pending was the start of it
            return 0;
        }
    }
    // Angle = sideways speed / forward speed. slope is mm/ms = m/s; the
    // cruise speed in the same units is TRAVEL_SPEED_CMS / 100. Rotated
    // LEFT, the left wall closes (negative slope) and the right one opens.
    {
        // Scale the denominator down rather than the numerator up: slope_num
        // x 57300 overflows 32 bits at steep angles; slope_den / 100 costs
        // well under 1% of precision (it is ~10^5 ms^2 over the window).
        int32_t a10 = (slope_num * 573L) / ((slope_den / 100L) * TRAVEL_SPEED_CMS + 1L);
        int32_t wall10 = (id == SONAR_LEFT) ? -a10 : a10;
        int32_t off = sh / WALL_ALIGN_SAMPLES - wall10;          // grid minus truth
        uint8_t k2;
        if (off >  WALL_ALIGN_MAX_STEP_10) off =  WALL_ALIGN_MAX_STEP_10;
        if (off < -WALL_ALIGN_MAX_STEP_10) off = -WALL_ALIGN_MAX_STEP_10;
        w->last    = (int16_t)off;
        w->last_ms = millis();
        if (w->pend_n < WALL_ALIGN_DELAY) {
            w->pend[w->pend_n++] = (int16_t)off;
            return 0;
        }
        *off10 = w->pend[0];                                    // oldest: wall went on
        for (k2 = 0; k2 + 1 < WALL_ALIGN_DELAY; k2++) w->pend[k2] = w->pend[k2 + 1];
        w->pend[WALL_ALIGN_DELAY - 1] = (int16_t)off;
    }
    return 1;
}

static void align_to_walls(int16_t gyro_rate) {
    uint8_t steady = !Motion_IsSuspect() &&
                     gyro_rate < WALL_ALIGN_MAX_RATE_LSB && gyro_rate > -WALL_ALIGN_MAX_RATE_LSB &&
                     (millis() - s_begin_ms) > (KICK_MS + DRIVE_SPINUP_MS);
    uint8_t k;
    for (k = 0; k < 2; k++) {
        int16_t off10;
        const wall_fit_t *other = &s_al[k ^ 1u];
        if (!wall_fit(&s_al[k], k ? SONAR_RIGHT : SONAR_LEFT, steady, &off10)) continue;
        // Both walls in view, and they disagree about the angle: one of them
        // is not a wall (a wall END, most likely -- see wall_fit). Use neither.
        if (other->pend_n > 0 && (millis() - other->last_ms) < WALL_ALIGN_PAIR_MS) {
            int16_t d = (int16_t)(s_al[k].last - other->last);
            if (d > WALL_ALIGN_AGREE_10 || d < -WALL_ALIGN_AGREE_10) continue;
        }
        Heading_GridAdjust(((int32_t)off10 * GYRO_LSB_MS_PER_DEGREE) /
                           (10L * WALL_ALIGN_GAIN_DEN));
    }
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
    // in three dead-end test logs: nominal creep 48, actual mean 50.6 / 52.8 / 56.2,
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

        if (s_recover_phase == 2) {
            // The pivot-away ends on ANGLE, not just time: a fixed 300 ms
            // pivot is anything from 30 to well over 90 degrees depending on
            // the battery, and a big one leaves the robot facing a different
            // corridor with no idea it has done so.
            int32_t turned = Heading_GridError() - s_recover_hdg0;
            if (turned < 0) turned = -turned;
            if (turned >= (int32_t)WALL_RECOVERY_PIVOT_DEG * GYRO_LSB_MS_PER_DEGREE)
                s_recover_until = millis();
        }
        if ((int32_t)(millis() - s_recover_until) < 0) return; // motors already set, wait

        if (s_recover_phase == 1) {
            // Reverse pulse done -- pivot away from whichever wall trapped us.
            // Away from LEFT = turn right (Motors_Pivot clockwise), away from
            // RIGHT = turn left, matching the same sign convention the normal
            // one-sided emergency below already uses.
            uint8_t cw = (s_emerg_side == 1) ? 1 : 0;
            Motors_Pivot(cw, WALL_RECOVERY_PIVOT_PWM);
            s_recover_hdg0  = Heading_GridError();
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
    align_to_walls(gyro_rate);
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

    // --- HEADING HOLD -----------------------------------------------------
    // Steer back onto the maze-grid heading. Without this term the controller
    // only DAMPED yaw: a disturbance -- the drive-off kick, a wheel that bit
    // late -- was slowed down but never undone, so the robot kept the angle
    // it was knocked to and only the (deadbanded, slow) wall term eventually
    // noticed, after it had drifted a few cm toward one wall. The real logs
    // show exactly that: 25-68 deg/s of yaw at the start of legs, then L=8 cm.
    //
    // It also cooperates with centring rather than fighting it: to close a
    // lateral error the wall term has to tilt the robot, and the equilibrium
    // tilt is wall_term / HEADING_KP -- a few degrees toward the centre line,
    // shrinking to zero as the error does.
    {
        int16_t h = (int16_t)((Heading_GridErrorTenths() * HEADING_KP_NUM) /
                              (10L * HEADING_KP_DEN));
        h = clamp16(h, -HEADING_MAX_CORR, HEADING_MAX_CORR);
        s_dbg.gyro_term += h;
    }
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

