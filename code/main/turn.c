#include "config.h"
#include <avr/wdt.h>
#include "turn.h"
#include "motors.h"
#include "mpu6050.h"
#include "heading.h"
#include "timer.h"
#include "sonar.h"
#include "debug.h"
#include "power.h"

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

// Largest |yaw rate| seen in the current turn, raw LSB. Reset per turn.
static int32_t s_peak_rate = 0;

// Signed yaw rate of the latest turn_sample(), raw LSB (positive = left).
static int16_t s_last_rate = 0;

// Per-sample streaming (turn-debug mode only) and coast measurement state.
static uint8_t  s_in_settle    = 0;   // inside settle_tracked()
static uint32_t s_settle_t0    = 0;
static int32_t  s_last_move_ms = 0;   // last time this settle saw real motion
static int32_t  s_coast_ms     = 0;   // result of the most recent settle


#if TURN_TRACE
// The tag comes from flash, like every other debug literal -- see the note in
// debug.h about SRAM.
static void turn_trace_p(const char *tag_flash) {
    Debug_P("  T ");
    Debug_StrP(tag_flash);
    Debug_KVF(" hdg10", Heading_DegreesTenths());
    Debug_NL();
}
#define turn_trace(tag) turn_trace_p(PSTR(tag))
#else
#define turn_trace(tag) ((void)0)
#endif

// One gyro sample on an exact TURN_TICK_MS schedule. Because Heading_Add()
// carries dt explicitly, this loop's rate is independent of the main control
// loop's rate -- both feed the same accumulator in the same units.
static void turn_sample(uint32_t *next_ms) {
    gyro_xyz_t g;
    int32_t r;
    while ((int32_t)(millis() - *next_ms) < 0) { wdt_reset(); }
    MPU6050_ReadAll(&g);
    Heading_Add(g.z, TURN_TICK_MS);
    Motion_Update(&g);
    // Track the peak rate so gyro clipping is visible. Done in int32 because
    // negating INT16_MIN would overflow.
    r = Gyro_Rate(g.z);
    s_last_rate = (int16_t)r;

    if (r < 0) r = -r;
    if (r > s_peak_rate) s_peak_rate = r;

    // Coast measurement: while the motors are off, remember the last moment
    // the chassis was still genuinely rotating. That is the real coast time,
    // which is what TURN_SETTLE_MS has to cover.
    if (s_in_settle && r >= TURN_STILL_LSB) {
        s_last_move_ms = (int32_t)(millis() - s_settle_t0);
    }

    *next_ms += TURN_TICK_MS;
}

// One straight-back pulse, still integrating the heading.
static void pulse_straight_back(uint32_t *next_ms) {
    uint32_t t0 = millis();
    Motors_SetLeft(DIR_REV,  BACKOFF_PULSE_PWM);
    Motors_SetRight(DIR_REV, BACKOFF_PULSE_PWM);
    while ((millis() - t0) < BACKOFF_PULSE_MS) turn_sample(next_ms);
    Motors_Stop();
}

// Motors off, but keep integrating: the chassis coasts after power is cut and
// that coast is real rotation.
static void settle_tracked(uint16_t ms, uint32_t *next_ms) {
    uint32_t t0 = millis();
    Motors_Stop();
    s_in_settle    = 1;
    s_settle_t0    = t0;
    s_last_move_ms = 0;
    while ((millis() - t0) < ms) turn_sample(next_ms);
    s_in_settle = 0;
    s_coast_ms  = s_last_move_ms;
}

// Front distance while standing still: median of three fresh pings, in cm.
// SONAR_TOO_CLOSE comes back as 0 so it compares as "nearer than anything".
static uint16_t front_still(void) {
    uint16_t v[3], t;
    uint8_t  k;
    for (k = 0; k < 3; k++) {
        uint8_t tries = 0;
        if (k) Timer_WaitMs(30);            // let the last echo die away
        // After a no-echo ping the sensor stays busy for ~38 ms; wait it out.
        while ((v[k] = Sonar_PingNow(SONAR_FRONT)) == SONAR_BUSY && ++tries < 8) {
            Timer_WaitMs(10);
        }
        if (v[k] == SONAR_BUSY)      v[k] = SONAR_NO_ECHO;
        if (v[k] == SONAR_TOO_CLOSE) v[k] = 0;
    }
    if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    if (v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
    if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    return v[1];
}

// Creep straight back in small pulses from a standstill -- NEVER a timed
// reverse at speed (see BACKOFF_PULSE_MS in config.h). Each pulse is followed
// by a full stop and settle, the heading is integrated throughout, and the
// front sonar is re-measured before every pulse.
//
// Stops when the front reads at least stop_front_cm (0 = do not look), when it
// has backed off max_cm from the first reading (0 = no limit), or after
// max_pulses. Returns the number of pulses used; *front_out gets the last
// front reading taken (SONAR_NO_ECHO if it never looked).
static uint8_t creep_back(uint8_t max_pulses, uint16_t stop_front_cm, uint16_t max_cm,
                          uint32_t *next_ms, uint16_t *front_out) {
    uint16_t f0 = SONAR_NO_ECHO, f = SONAR_NO_ECHO;
    uint8_t  k;

    for (k = 0; k < max_pulses; k++) {
        if (stop_front_cm) {
            f = front_still();
            if (k == 0) f0 = f;
            if (f != SONAR_NO_ECHO && f >= stop_front_cm) break;
            if (max_cm && f0 != SONAR_NO_ECHO && f != SONAR_NO_ECHO && f >= f0 + max_cm) break;
        }
        Power_SetActivity(ACT_REVERSING);
        pulse_straight_back(next_ms);
        settle_tracked(BACKOFF_SETTLE_MS, next_ms);     // motors off, coast counted
    }
    if (stop_front_cm && k == max_pulses) f = front_still();   // where the last pulse left it
    Power_SetActivity(ACT_IDLE);
    if (front_out) *front_out = f;
    return k;
}

static uint16_t isqrt32(uint32_t x) {
    uint32_t r = 0, bit = 1UL << 30;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else                r >>= 1;
        bit >>= 2;
    }
    return (uint16_t)r;
}

// Signed pivot command: positive turns in the requested direction, negative
// the other way (braking, or backing out of an overshoot). Anything below the
// motor floor is not a usable command -- coast instead of buzzing.
static void drive_pivot(int16_t u, uint8_t cw) {
    if (u >= (int16_t)MOTOR_MIN_PWM)       Motors_Pivot(cw,  (uint8_t)u);
    else if (u <= -(int16_t)MOTOR_MIN_PWM) Motors_Pivot(!cw, (uint8_t)(-u));
    else                                   Motors_Stop();
}

// One turn, closed loop on the gyro rate the whole way. See "HOW A TURN
// WORKS NOW" in config.h for the control law and how to tune it.
static void execute_single(int32_t target_tenths, turn_dir_t dir, turn_result_t *res) {
    const uint8_t cw     = (dir == TURN_RIGHT) ? 1 : 0;
    const int32_t target = (target_tenths * GYRO_LSB_MS_PER_DEGREE) / 10L;
    const int32_t arrive = ((int32_t)TURN_CTL_ARRIVE_TENTHS * GYRO_LSB_MS_PER_DEGREE) / 10L;
    const int32_t accept = ((int32_t)TURN_DEADBAND_TENTHS   * GYRO_LSB_MS_PER_DEGREE) / 10L;

    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint32_t t_start = millis();
    int16_t  u        = 0;      // current command, PWM, signed (see drive_pivot)
    uint8_t  boost    = 0;      // breakaway boost, PWM
    uint16_t stall_ms = 0;      // time spent driven but not moving
    uint8_t  stops    = 0;      // times it has arrived and settled
    uint8_t  backed_off = 0;
    uint8_t  tick     = 0;

    Heading_Reset();
    res->timed_out   = 0;
    res->nudges_used = 0;       // here: extra control passes after a settle
    res->initial_error_tenths = 0;
    res->coast_ms    = 0;
    s_peak_rate      = 0;
    s_last_rate      = 0;

    Power_SetActivity(ACT_TURN_SWEEP);
    for (;;) {
        // Progress and speed IN THE TURNING DIRECTION (positive = toward the
        // target), whichever way the gyro counts.
        const int32_t prog  = (dir == TURN_RIGHT) ? -Heading_Raw() : Heading_Raw();
        const int32_t err   = target - prog;
        const int32_t w10   = ((int32_t)((dir == TURN_RIGHT) ? -s_last_rate : s_last_rate)
                               * 10000L) / GYRO_LSB_MS_PER_DEGREE;       // tenths of deg/s
        int32_t wd10, want;

        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }

        // --- ARRIVED? stop, settle (still integrating), then judge --------
        if (abs32(err) <= arrive && abs32(w10) <= (int32_t)TURN_CTL_STOP_DPS * 10) {
            int32_t settled;
            Motors_Stop();
            u = 0; boost = 0; stall_ms = 0;
            settle_tracked(TURN_CTL_SETTLE_MS, &next_ms);
            stops++;
            settled = target - ((dir == TURN_RIGHT) ? -Heading_Raw() : Heading_Raw());
            if (stops == 1) {
                res->initial_error_tenths = (settled * 10L) / GYRO_LSB_MS_PER_DEGREE;
                res->coast_ms = s_coast_ms;
            }
            turn_trace("stop");
            if (abs32(settled) <= accept || stops > TURN_CTL_MAX_RESTARTS) break;
            res->nudges_used++;
            continue;
        }

        // --- allowed speed: what can still be stopped in the room left ----
        {
            uint32_t e10 = (uint32_t)((abs32(err) * 10L) / GYRO_LSB_MS_PER_DEGREE);
            int32_t  wd  = isqrt32(((uint32_t)TURN_CTL_DECEL_DPS2 * e10) / 5UL);  // deg/s
            if (wd > TURN_CTL_MAX_DPS) wd = TURN_CTL_MAX_DPS;
            if (wd < TURN_CTL_MIN_DPS) wd = TURN_CTL_MIN_DPS;
            wd10 = (err >= 0) ? wd * 10 : -wd * 10;
        }

        // --- control law, in the frame of the allowed direction -----------
        {
            const int32_t sgn = (wd10 >= 0) ? 1 : -1;
            const int32_t e   = sgn * (wd10 - w10);          // >0: too slow, <0: too fast
            const int32_t moving = abs32(w10) >= (int32_t)TURN_CTL_STALL_DPS * 10;

            if (e >= 0) {
                // Too slow: drive. Breakaway boost builds while stuck.
                if (!moving && abs32(u) >= TURN_CTL_PWM_FLOOR) {
                    if (boost + TURN_CTL_BOOST_STEP <= TURN_CTL_BOOST_MAX) boost += TURN_CTL_BOOST_STEP;
                    stall_ms += TURN_TICK_MS;
                } else {
                    if (boost) boost--;
                    stall_ms = 0;
                }
                want = TURN_CTL_PWM_FLOOR + boost +
                       (e * TURN_CTL_KP_NUM) / (TURN_CTL_KP_DEN * 10L);
            } else {
                // Too fast: reverse torque in proportion. No floor -- a small
                // excess just coasts off.
                if (boost) boost--;
                stall_ms = 0;
                want = (e * TURN_CTL_KB_NUM) / (TURN_CTL_KB_DEN * 10L);   // negative
            }
            want *= sgn;
            if (want >  TURN_CTL_PWM_MAX) want =  TURN_CTL_PWM_MAX;
            if (want < -TURN_CTL_PWM_MAX) want = -TURN_CTL_PWM_MAX;
        }

        // --- slew-limit, apply ---------------------------------------------
        if (want > u + TURN_CTL_SLEW)      u = (int16_t)(u + TURN_CTL_SLEW);
        else if (want < u - TURN_CTL_SLEW) u = (int16_t)(u - TURN_CTL_SLEW);
        else                               u = (int16_t)want;
        drive_pivot(u, cw);

        // --- pinned: driven hard, not moving -------------------------------
        if (stall_ms >= TURN_CTL_PINNED_MS && boost >= TURN_CTL_BOOST_MAX && !backed_off) {
            backed_off = 1;
            Motors_Stop();
            u = 0; boost = 0; stall_ms = 0;
            creep_back(TURN_UNSTICK_PULSES, 0, 0, &next_ms, 0);
            Power_SetActivity(ACT_TURN_SWEEP);
            turn_trace("pinned, backed off");
            continue;
        }

#if TURN_TRACE && TURN_TRACE_PROFILE
        if ((++tick & 7) == 0) {
            Debug_KVF("  T hdg10", Heading_DegreesTenths());
            Debug_KVF("wd", wd10 / 10);
            Debug_KVF("w", w10 / 10);
            Debug_KVF("u", u);
            Debug_NL();
        }
#else
        (void)tick;
#endif
        turn_sample(&next_ms);
    }

    Motors_Stop();
    res->achieved_tenths = abs32(Heading_DegreesTenths());
    res->peak_rate       = s_peak_rate;

    {
        int32_t left = target - abs32(Heading_Raw());
        res->final_error_tenths = (left * 10L) / GYRO_LSB_MS_PER_DEGREE;
        res->converged = (abs32(left) <= accept) ? 1 : 0;
        // Undershooting a RIGHT turn leaves the chassis LEFT of where it
        // should point (positive); undershooting a LEFT turn leaves it right.
        res->residual_raw = (dir == TURN_RIGHT) ? left : -left;
    }

    // Direction check. Convention: positive gyro Z = turning LEFT, so a right
    // turn must accumulate negative. The result above works on |heading|, so
    // without this a turn that went the wrong way reports a clean success.
    {
        int32_t signed_hdg = Heading_Raw();
        if (dir == TURN_RIGHT) res->wrong_way = (signed_hdg > 0) ? 1 : 0;
        else                   res->wrong_way = (signed_hdg < 0) ? 1 : 0;
    }
}

static void execute_tenths(int32_t target_tenths, turn_dir_t dir, turn_result_t *res) {
    execute_single(target_tenths, dir, res);

    // Recalibrate the gyro bias after every pivot (your requirement).
    // The chassis is stationary here, which is the only time a valid bias
    // measurement is possible. If it will not hold still the previous offset
    // is kept rather than corrupted.
    res->recal_ok = Gyro_CalibrateQuick();

    // Everything in the sonar filters was measured while pointing a different
    // direction. Throw it away.
    Sonar_Flush();
    Heading_Reset();
}

void Turn_Execute(uint16_t degrees, turn_dir_t dir, turn_result_t *res) {
    execute_tenths((int32_t)degrees * 10L, dir, res);
}

void Turn_90(turn_dir_t dir, turn_result_t *res) {
    Turn_Execute(90, dir, res);
}

// Between the two halves of a U-turn. The first 90 has left the robot facing
// what was the side wall on the turning side; if that wall is too close, the
// second 90 drives the front corner into it. Only when the axle is past the
// corridor centre line toward that wall, creep straight back in pulses to the
// centre line and no further: UTURN_MID_FRONT_CM, capped at
// UTURN_BACKOFF_MAX_PULSES pulses and UTURN_BACKOFF_MAX_CM of travel.
//
// side_cm is what that side read BEFORE the turn. It resolves one ambiguity:
// right up against a wall an HC-SR04 often returns no echo at all, which on
// its own reads as open space. If there was a wall on that side, no echo now
// means too close, not open.
//
// The heading is integrated throughout. Returns the rotation picked up while
// reversing, tenths of a degree, positive = in the turning direction, so the
// second half can take it off its own target.
static int32_t uturn_clearance(turn_dir_t dir, uint16_t side_cm) {
    uint16_t f = front_still();
    uint16_t f_end;
    uint32_t next_ms;
    uint8_t  pulses;
    int32_t  h10;

    if (f == SONAR_NO_ECHO && side_cm < OPENING_THRESHOLD_CM) f = 0;

    Debug_KVF("  mid-uturn front", (int32_t)f);
    if (f >= UTURN_MID_FRONT_CM) {
        Debug_P("ok\r\n");
        return 0;
    }

    Debug_P("past centre, creeping back\r\n");
    next_ms = millis() + TURN_TICK_MS;
    pulses = creep_back(UTURN_BACKOFF_MAX_PULSES, UTURN_MID_FRONT_CM, UTURN_BACKOFF_MAX_CM,
                        &next_ms, &f_end);
    h10 = Heading_DegreesTenths();
    Debug_KVF("  back-off pulses", (int32_t)pulses);
    Debug_KVF("front", (int32_t)f_end);
    Debug_KVF("yaw10", h10);
    Debug_NL();
    return (dir == TURN_RIGHT) ? -h10 : h10;
}

void Turn_180(turn_dir_t dir, uint16_t side_cm, turn_result_t *res) {
#if TURN_180_AS_TWO_90S
    // Two 90s with a settle between usually beats one long sweep: momentum
    // has less time to build, so there is less coast to correct for. It also
    // gives a stop halfway, facing a wall, which is the one moment the
    // clearance for the rest of the turn can be measured and fixed.
    turn_result_t a, b;
    int32_t moved10, second10;

    execute_tenths(900, dir, &a);
    Timer_WaitMs(200);
    moved10 = uturn_clearance(dir, side_cm);

    // The second half aims at 180 in TOTAL, not at another 90: whatever the
    // first half left inside its deadband (up to 2 degrees either way), and
    // whatever the reverse added, is taken off here instead of doubling up.
    second10 = 1800L - a.achieved_tenths - moved10;
    if (a.wrong_way) second10 = 900;        // first half is not trustworthy
    execute_tenths(second10, dir, &b);

    res->achieved_tenths      = a.achieved_tenths + moved10 + b.achieved_tenths;
    res->nudges_used          = (uint8_t)(a.nudges_used + b.nudges_used);
    res->timed_out            = a.timed_out | b.timed_out;
    res->recal_ok             = b.recal_ok;
    res->initial_error_tenths = a.initial_error_tenths + b.initial_error_tenths;
    res->peak_rate            = (a.peak_rate > b.peak_rate) ? a.peak_rate : b.peak_rate;
    res->wrong_way            = a.wrong_way | b.wrong_way;
    res->coast_ms             = (a.coast_ms > b.coast_ms) ? a.coast_ms : b.coast_ms;
    res->final_error_tenths   = b.final_error_tenths;
    res->converged            = b.converged;
    res->residual_raw         = b.residual_raw;   // only the last turn's frame
                                                  // is still current
#else
    (void)side_cm;
    Turn_Execute(180, dir, res);
#endif
}
