#include "config.h"
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
static uint8_t  s_sample_trace = 0;   // Turn_SampleTrace()
static uint8_t  s_decimate     = 0;
static uint32_t s_turn_t0      = 0;   // millis() at the start of this turn
static uint8_t  s_in_settle    = 0;   // inside settle_tracked()
static uint32_t s_settle_t0    = 0;
static int32_t  s_last_move_ms = 0;   // last time this settle saw real motion
static int32_t  s_coast_ms     = 0;   // result of the most recent settle

void Turn_SampleTrace(uint8_t on) { s_sample_trace = on; }

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
    while ((int32_t)(millis() - *next_ms) < 0) { /* wait for the slot */ }
    MPU6050_ReadAll(&g);
    Heading_Add(g.z, TURN_TICK_MS);
    Motion_Update(&g);
    // Track the peak rate so gyro clipping is visible. Done in int32 because
    // negating INT16_MIN would overflow.
    r = Gyro_Rate(g.z);

    if (s_sample_trace && ++s_decimate >= TURNDBG_SAMPLE_EVERY) {
        s_decimate = 0;
        Debug_P("S,");
        Debug_Int((int32_t)(millis() - s_turn_t0));
        Debug_P(",");
        Debug_Int(r);
        Debug_P(",");
        Debug_Int(Heading_DegreesTenths());
        Debug_NL();
    }

    if (r < 0) r = -r;
    if (r > s_peak_rate) s_peak_rate = r;

    // Coast measurement: while the motors are off, remember the last moment
    // the chassis was still genuinely rotating. That is the real coast time,
    // which is what TURN_SETTLE_MS has to cover.
    if (s_in_settle && r >= TURNDBG_STILL_LSB) {
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
// no back-EMF -- and every failed run in the Mode 10 logs reset at exactly
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

static void execute_single(uint16_t degrees, turn_dir_t dir, turn_result_t *res) {
    const uint8_t cw = (dir == TURN_RIGHT) ? 1 : 0;
    const int32_t target   = (int32_t)degrees * GYRO_LSB_MS_PER_DEGREE;
    const int32_t stop_at  = target - ((int32_t)TURN_STOP_MARGIN_DEG * GYRO_LSB_MS_PER_DEGREE);
    const int32_t deadband = (int32_t)TURN_DEADBAND_DEG * GYRO_LSB_MS_PER_DEGREE;

    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint32_t t_start = millis();
    uint8_t  i;

    Heading_Reset();
    res->timed_out   = 0;
    res->nudges_used = 0;
    s_peak_rate      = 0;
    s_turn_t0        = millis();
    s_decimate       = 0;

    // --- PHASE 1: kickstart, tracked -------------------------------------
    // A pivot skids the tyres sideways, so it needs more breakaway torque
    // than rolling straight. Counted, because with the wheels turning in
    // opposite directions this kick is real rotation.
#if KICK_RAMP
    pulse_ramped(cw, TURN_KICK_PWM, TURN_KICK_MS, &next_ms);
#else
    pulse_tracked(cw, TURN_KICK_PWM, TURN_KICK_MS, &next_ms);
#endif
    turn_trace("kick");

    // --- PHASE 2: slow sweep, stopping early on purpose -------------------
    Motors_Pivot(cw, TURN_PWM);
    while (abs32(Heading_Raw()) < stop_at) {
        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }
        turn_sample(&next_ms);
    }
    turn_trace("sweep");

    // --- PHASE 3: active brake, tracked -----------------------------------
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
    // The chassis is stopped and the accumulator now holds the true angle.
    // Nudge in whichever direction shrinks the error (cw if undershot, the
    // reverse if overshot), then re-measure. The nudge LENGTH scales with
    // the remaining error instead of firing the same fixed pulse regardless
    // of size -- a fixed pulse either crawls toward a large gap or blows
    // through a tiny one by the same amount, which oscillates instead of
    // converging.
    for (i = 0; i < TURN_MAX_NUDGES; i++) {
        int32_t err = target - abs32(Heading_Raw());
        int32_t err_deg_tenths;
        uint16_t nudge_ms;

        if (abs32(err) <= deadband) break;
        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }

        err_deg_tenths = (abs32(err) * 10L) / GYRO_LSB_MS_PER_DEGREE;
        nudge_ms = (uint16_t)((err_deg_tenths * TURN_NUDGE_MS_PER_DEG) / 10L);
        if (nudge_ms < TURN_NUDGE_MS_MIN) nudge_ms = TURN_NUDGE_MS_MIN;
        if (nudge_ms > TURN_NUDGE_MS_MAX) nudge_ms = TURN_NUDGE_MS_MAX;

        pulse_tracked((err > 0) ? cw : !cw, TURN_NUDGE_PWM, nudge_ms, &next_ms);
        settle_tracked(TURN_SETTLE_MS, &next_ms);
        res->nudges_used++;

#if TURN_TRACE
        // Which way this nudge pushed and how long, then where it landed.
        // Nudges alternating sign run after run means the settle is ending
        // before the chassis has actually stopped coasting.
        Debug_P("  T nudge");
        Debug_Int(res->nudges_used);
        if (err > 0) Debug_P(" fwd "); else Debug_P(" rev ");
        Debug_KVF("ms", (int32_t)nudge_ms);
        Debug_KVF("hdg10", Heading_DegreesTenths());
        Debug_NL();
#endif
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

void Turn_Execute(uint16_t degrees, turn_dir_t dir, turn_result_t *res) {
    execute_single(degrees, dir, res);

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

void Turn_90(turn_dir_t dir, turn_result_t *res) {
    Turn_Execute(90, dir, res);
}

void Turn_180(turn_dir_t dir, turn_result_t *res) {
#if TURN_180_AS_TWO_90S
    // Two 90s with a settle between usually beats one long sweep: momentum
    // has less time to build, so there is less coast to correct for.
    turn_result_t a, b;
    Turn_Execute(90, dir, &a);
    Timer_WaitMs(200);
    Turn_Execute(90, dir, &b);
    res->achieved_tenths      = a.achieved_tenths + b.achieved_tenths;
    res->nudges_used          = (uint8_t)(a.nudges_used + b.nudges_used);
    res->timed_out            = a.timed_out | b.timed_out;
    res->recal_ok             = b.recal_ok;
    res->initial_error_tenths = a.initial_error_tenths + b.initial_error_tenths;
    res->peak_rate            = (a.peak_rate > b.peak_rate) ? a.peak_rate : b.peak_rate;
    res->wrong_way            = a.wrong_way | b.wrong_way;
    res->coast_ms             = (a.coast_ms > b.coast_ms) ? a.coast_ms : b.coast_ms;
    res->final_error_tenths   = a.final_error_tenths + b.final_error_tenths;
    res->converged            = a.converged & b.converged;
    res->residual_raw         = b.residual_raw;   // only the last turn's frame
                                                  // is still current
#else
    Turn_Execute(180, dir, res);
#endif
}
