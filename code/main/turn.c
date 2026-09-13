#include "config.h"
#include "turn.h"
#include "motors.h"
#include "mpu6050.h"
#include "heading.h"
#include "timer.h"
#include "sonar.h"

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

// One gyro sample on an exact TURN_TICK_MS schedule. Because Heading_Add()
// carries dt explicitly, this loop's rate is independent of the main control
// loop's rate -- both feed the same accumulator in the same units.
static void turn_sample(uint32_t *next_ms) {
    gyro_xyz_t g;
    while ((int32_t)(millis() - *next_ms) < 0) { /* wait for the slot */ }
    MPU6050_ReadAll(&g);
    Heading_Add(g.z, TURN_TICK_MS);
    Motion_Update(&g);
    *next_ms += TURN_TICK_MS;
}

// Drive the pivot for a fixed duration, still integrating. Used for the kick,
// the brake and each correction nudge -- so no rotation happens uncounted.
static void pulse_tracked(uint8_t cw, uint8_t pwm, uint16_t ms, uint32_t *next_ms) {
    uint32_t t0 = millis();
    Motors_Pivot(cw, pwm);
    while ((millis() - t0) < ms) turn_sample(next_ms);
}

// Motors off, but keep integrating: the chassis coasts after power is cut and
// that coast is real rotation.
static void settle_tracked(uint16_t ms, uint32_t *next_ms) {
    uint32_t t0 = millis();
    Motors_Stop();
    while ((millis() - t0) < ms) turn_sample(next_ms);
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

    // --- PHASE 1: kickstart, tracked -------------------------------------
    // A pivot skids the tyres sideways, so it needs more breakaway torque
    // than rolling straight. Counted, because with the wheels turning in
    // opposite directions this kick is real rotation.
    pulse_tracked(cw, TURN_KICK_PWM, TURN_KICK_MS, &next_ms);

    // --- PHASE 2: slow sweep, stopping early on purpose -------------------
    Motors_Pivot(cw, TURN_PWM);
    while (abs32(Heading_Raw()) < stop_at) {
        if ((millis() - t_start) > TURN_TIMEOUT_MS) { res->timed_out = 1; break; }
        turn_sample(&next_ms);
    }

    // --- PHASE 3: active brake, tracked -----------------------------------
    pulse_tracked(!cw, TURN_BRAKE_PWM, TURN_BRAKE_MS, &next_ms);

    // --- PHASE 4: settle, still counting the coast -------------------------
    settle_tracked(TURN_SETTLE_MS, &next_ms);

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
    }

    Motors_Stop();
    res->achieved_tenths = abs32(Heading_DegreesTenths());
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

void Turn_180(turn_result_t *res) {
#if TURN_180_AS_TWO_90S
    // Two 90s with a settle between usually beats one long sweep: momentum
    // has less time to build, so there is less coast to correct for.
    turn_result_t a, b;
    Turn_Execute(90, TURN_RIGHT, &a);
    Timer_WaitMs(200);
    Turn_Execute(90, TURN_RIGHT, &b);
    res->achieved_tenths      = a.achieved_tenths + b.achieved_tenths;
    res->nudges_used          = (uint8_t)(a.nudges_used + b.nudges_used);
    res->timed_out            = a.timed_out | b.timed_out;
    res->recal_ok             = b.recal_ok;
    res->initial_error_tenths = a.initial_error_tenths + b.initial_error_tenths;
#else
    Turn_Execute(180, TURN_RIGHT, res);
#endif
}
