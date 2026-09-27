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

// Drive the pivot for a fixed duration, still integrating. Used for the kick,
// the brake and each correction nudge -- so no rotation happens uncounted.
static void pulse_tracked(uint8_t cw, uint8_t pwm, uint16_t ms, uint32_t *next_ms) {
    uint32_t t0 = millis();
    Motors_Pivot(cw, pwm);
    while ((millis() - t0) < ms) turn_sample(next_ms);
}

// Same, but ramps up to pwm across the pulse instead of stepping to it.
// Used ONLY for the kick. A pivot kick is the single largest current transient
// the firmware asks for -- both motors stalled, driven in opposite directions,
// no back-EMF -- and every failed run in the early dead-end test logs reset at exactly
// this point with the brown-out flag set. Spreading the same impulse over
// TURN_KICK_MS roughly halves the peak draw.
static void pulse_ramped(uint8_t cw, uint8_t pwm, uint16_t ms, uint32_t *next_ms) {
    uint32_t t0 = millis();
    uint32_t el;
    Motors_Pivot(cw, MOTOR_MIN_PWM);
    while ((el = millis() - t0) < ms) {
        Motors_Pivot(cw, (uint8_t)(MOTOR_MIN_PWM +
            (((uint32_t)(pwm - MOTOR_MIN_PWM) * el) / ms)));
        // Dense rail sampling through the pivot kick -- the single largest
        // current draw in the firmware, and where every logged brown-out hit.
        // The per-tick sampler is far too slow to catch a dip this brief.
        Power_Task();
        turn_sample(next_ms);
    }
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

// Learned nudge response: ms of nudge per degree of rotation beyond the
// breakaway time, x10. Kept across turns -- battery and floor change slowly,
// so the last turn's figure is the best first guess for this one.
static uint16_t s_nudge_k10 = TURN_NUDGE_K10_INIT;

static void execute_single(int32_t target_tenths, turn_dir_t dir, turn_result_t *res) {
    const uint8_t cw = (dir == TURN_RIGHT) ? 1 : 0;
    const int32_t target   = (target_tenths * GYRO_LSB_MS_PER_DEGREE) / 10L;
    const int32_t deadband = ((int32_t)TURN_DEADBAND_TENTHS * GYRO_LSB_MS_PER_DEGREE) / 10L;
    int32_t stop_at = target - ((int32_t)TURN_STOP_MARGIN_DEG * GYRO_LSB_MS_PER_DEGREE);

    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint32_t t_start = millis();
    uint8_t  i;
    uint8_t  no_progress = 0;   // consecutive nudges that did not close the error
    const uint16_t k_at_start = s_nudge_k10;
    uint8_t  backed_off  = 0;   // the one straight back-off has been used

    if (stop_at < 0) stop_at = 0;

    Heading_Reset();
    res->timed_out   = 0;
    res->nudges_used = 0;
    s_peak_rate      = 0;

    // --- PHASE 1: kickstart, tracked -------------------------------------
    // A pivot skids the tyres sideways, so it needs more breakaway torque
    // than rolling straight. Counted, because with the wheels turning in
    // opposite directions this kick is real rotation.
    Power_SetActivity(ACT_TURN_KICK);
#if KICK_RAMP
    pulse_ramped(cw, TURN_KICK_PWM, TURN_KICK_MS, &next_ms);
#else
    pulse_tracked(cw, TURN_KICK_PWM, TURN_KICK_MS, &next_ms);
#endif
    turn_trace("kick");

    // --- PHASE 2: slow sweep, stopping early on purpose -------------------
    Power_SetActivity(ACT_TURN_SWEEP);
    Motors_Pivot(cw, TURN_PWM);
    while (abs32(Heading_Raw()) < stop_at) {
        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }
        turn_sample(&next_ms);
    }
    turn_trace("sweep");

    // --- PHASE 3: active brake, tracked -----------------------------------
    Power_SetActivity(ACT_TURN_BRAKE);
    pulse_tracked(!cw, TURN_BRAKE_PWM, TURN_BRAKE_MS, &next_ms);
    turn_trace("brake");

    // --- PHASE 4: settle, still counting the coast -------------------------
    settle_tracked(TURN_SETTLE_MS, &next_ms);
    turn_trace("settle");
    // Coast from the main sweep only -- later settles (after each nudge) would
    // overwrite s_coast_ms, and it is this one that TURN_SETTLE_MS is sized
    // against.
    res->coast_ms = s_coast_ms;

    // Residual error from the fixed early-stop margin alone, before any
    // closed-loop nudging. Positive = undershot, negative = overshot -- see
    // turn.h. This is the number that tells you whether TURN_STOP_MARGIN_DEG
    // is guessing the coast right, not just that *some* correction happened.
    res->initial_error_tenths =
        ((target - abs32(Heading_Raw())) * 10L) / GYRO_LSB_MS_PER_DEGREE;

    // --- PHASE 5: closed-loop correction ----------------------------------
    // The chassis is stopped and the accumulator holds the true angle turned
    // so far -- including any coast and tyre unwinding, since the settle kept
    // integrating. The error is therefore KNOWN, whatever the sweep did. Nudge
    // in whichever direction shrinks it (cw if undershot, the reverse if
    // overshot), let it settle, re-measure, and repeat until the measured
    // angle is within TURN_DEADBAND_TENTHS of the target.
    for (i = 0; i < TURN_MAX_NUDGES; i++) {
        int32_t err = target - abs32(Heading_Raw());
        int32_t err10, want10, moved10, h_before;
        uint16_t nudge_ms;

        if (abs32(err) <= deadband) break;
        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }

        err10   = (abs32(err) * 10L) / GYRO_LSB_MS_PER_DEGREE;
        want10  = (err10 * TURN_NUDGE_AIM_PCT) / 100L;
        {
            int32_t ms = TURN_NUDGE_MS_MIN + (want10 * (int32_t)s_nudge_k10) / 100L;
            if (ms > TURN_NUDGE_MS_MAX) ms = TURN_NUDGE_MS_MAX;
            nudge_ms = (uint16_t)ms;
        }

        h_before = abs32(Heading_Raw());
        Power_SetActivity(ACT_TURN_NUDGE);
        pulse_tracked((err > 0) ? cw : !cw, TURN_NUDGE_PWM, nudge_ms, &next_ms);
        settle_tracked(TURN_SETTLE_MS, &next_ms);
        res->nudges_used++;

        // What the nudge really did, in its own direction, tenths of a degree
        // (negative = the chassis went the other way).
        moved10 = ((abs32(Heading_Raw()) - h_before) * 10L) / GYRO_LSB_MS_PER_DEGREE;
        if (err < 0) moved10 = -moved10;

        // Learn k from it. A nudge that moved a measurable amount gives
        // k = (ms beyond breakaway) / degrees moved; average it in. One that
        // barely moved means k is too small -- raise it by half.
        if (moved10 >= 3 && nudge_ms > TURN_NUDGE_MS_MIN) {
            int32_t k_obs = ((int32_t)(nudge_ms - TURN_NUDGE_MS_MIN) * 100L) / moved10;
            int32_t k_new = ((int32_t)s_nudge_k10 + k_obs) / 2;
            if (k_new < TURN_NUDGE_K10_MIN) k_new = TURN_NUDGE_K10_MIN;
            if (k_new > TURN_NUDGE_K10_MAX) k_new = TURN_NUDGE_K10_MAX;
            s_nudge_k10 = (uint16_t)k_new;
        } else if (moved10 < 3) {
            uint16_t k_new = (uint16_t)(s_nudge_k10 + s_nudge_k10 / 2);
            s_nudge_k10 = (k_new > TURN_NUDGE_K10_MAX) ? TURN_NUDGE_K10_MAX : k_new;
        }

#if TURN_TRACE
        // Which way this nudge pushed and how long, then where it landed.
        // Nudges alternating sign run after run means the settle is ending
        // before the chassis has actually stopped coasting.
        Debug_P("  T nudge");
        Debug_Int(res->nudges_used);
        if (err > 0) Debug_P(" fwd "); else Debug_P(" rev ");
        Debug_KVF("ms", (int32_t)nudge_ms);
        Debug_KVF("hdg10", Heading_DegreesTenths());
        Debug_KVF("k10", (int32_t)s_nudge_k10);
        Debug_NL();
#endif

        // PINNED? A nudge that does not bring the error down by at least
        // TURN_STUCK_PROGRESS_DEG10 means something is stopping the chassis
        // from rotating -- in practice a corner against a wall. In the 13:06
        // dead-end run four 40 ms nudges in a row moved it 1.4, -3.8, 1.6 and
        // 0.3 degrees and the turn gave up 21 degrees short. Two of those in a
        // row: back straight off once, which frees the corner, and carry on.
        {
            // Only a long nudge counts: a short one near the target can
            // legitimately fail to break the tyres loose.
            if (nudge_ms >= TURN_STUCK_MIN_NUDGE_MS && moved10 < TURN_STUCK_PROGRESS_DEG10) {
                no_progress++;
            } else {
                no_progress = 0;
            }
            if (no_progress >= 2 && !backed_off) {
                backed_off  = 1;
                no_progress = 0;
                // The pinned nudges taught k nothing true -- it only grew.
                s_nudge_k10 = k_at_start;
                creep_back(TURN_UNSTICK_PULSES, 0, 0, &next_ms, 0);
#if TURN_TRACE
                Debug_P("  T pinned, backed off");
                Debug_KVF(" hdg10", Heading_DegreesTenths());
                Debug_NL();
#endif
            }
        }
    }

    Motors_Stop();
    res->achieved_tenths = abs32(Heading_DegreesTenths());
    res->peak_rate       = s_peak_rate;

    {
        int32_t left = target - abs32(Heading_Raw());
        res->final_error_tenths = (left * 10L) / GYRO_LSB_MS_PER_DEGREE;
        res->converged = (abs32(left) <= deadband) ? 1 : 0;
        // Undershooting a RIGHT turn leaves the chassis LEFT of where it
        // should point (positive); undershooting a LEFT turn leaves it right.
        res->residual_raw = (dir == TURN_RIGHT) ? left : -left;
    }

    // Direction check. Convention: positive gyro Z = turning LEFT, so a right
    // turn must accumulate negative. Everything above works on |heading|, so
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
