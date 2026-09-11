#include "config.h"
#include "drive.h"
#include "motors.h"
#include "sonar.h"
#include "heading.h"
#include "timer.h"

static center_mode_t s_mode = CENTER_GYRO_ONLY;
static int16_t       s_last_corr = 0;
static int32_t       s_hold_heading = 0;
static drive_debug_t s_dbg;

const drive_debug_t *Drive_Debug(void) { return &s_dbg; }

void Drive_Begin(void) {
    s_last_corr = 0;
    s_mode = CENTER_GYRO_ONLY;
    s_hold_heading = 0;
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

    // --- EMERGENCY: wall closer than the controller can gracefully handle ---
    // Checked first and unconditionally. At this range the proportional term
    // is too slow, and this must work even while the chassis is rocking --
    // a collision is exactly the moment sonar gets noisiest.
    if (l_near && !r_near) {
        s_last_corr = WALL_MAX_CORRECTION;      // hard right, away from left wall
        s_dbg.branch = BRANCH_EMERG_L;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = MOTOR_MIN_PWM + WALL_MAX_CORRECTION;
        s_dbg.pwm_r = MOTOR_MIN_PWM;
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }
    if (r_near && !l_near) {
        s_last_corr = -WALL_MAX_CORRECTION;     // hard left, away from right wall
        s_dbg.branch = BRANCH_EMERG_R;
        s_dbg.error_cm = 0; s_dbg.wall_term = 0; s_dbg.gyro_term = 0;
        s_dbg.corr = s_last_corr;
        s_dbg.pwm_l = MOTOR_MIN_PWM;
        s_dbg.pwm_r = MOTOR_MIN_PWM + WALL_MAX_CORRECTION;
        s_dbg.l_ok = l_ok; s_dbg.r_ok = r_ok;
        Motors_Forward(s_dbg.pwm_l, s_dbg.pwm_r);
        return;
    }

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
