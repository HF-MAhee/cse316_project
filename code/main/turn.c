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
static uint8_t  s_in_settle    = 0;   // inside settle_tracked()
static uint32_t s_settle_t0    = 0;
static int32_t  s_last_move_ms = 0;   // last time this settle saw real motion
static int32_t  s_coast_ms     = 0;   // result of the most recent settle

// LEARNED, per robot, per battery. Both start from the config values and are
// refined by every turn, because neither is a constant of the design: the
// coast depends on the tyres, the floor and how charged the pack is, and a
// nudge's worth depends on how much of it the gearbox slack swallows.
//
// The real logs (usart_20260924_*) are why. TURN_STOP_MARGIN_DEG was 35,
// measured on an earlier, faster build; this robot coasts ~22, so every sweep
// was cut 13 degrees early, every turn landed at 71-81 degrees, and 2-5 nudges
// ground it the rest of the way. Learning the coast turns that into ~0-1.
static int32_t  s_coast_est      = (int32_t)TURN_STOP_MARGIN_DEG * GYRO_LSB_MS_PER_DEGREE;
static uint16_t s_nudge_ms_deg10 = (uint16_t)TURN_NUDGE_MS_PER_DEG * 10u;  // ms per 10 deg


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

// `target` is the rotation still needed to land on the grid, LSB*ms -- not a
// flat 90: see turn_quarter().
static void execute_single(int32_t target, turn_dir_t dir, turn_result_t *res) {
    const uint8_t cw = (dir == TURN_RIGHT) ? 1 : 0;
    const int32_t deadband = (int32_t)TURN_DEADBAND_DEG * GYRO_LSB_MS_PER_DEGREE;
    // Cut the sweep where the learned coast will carry it to the target --
    // but never so early that the sweep is mostly skipped.
    const int32_t stop_at  = (target - s_coast_est > target / 3) ? (target - s_coast_est)
                                                                  : (target / 3);
    int32_t sweep_exit;

    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint32_t t_start = millis();
    uint8_t  i;

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
    sweep_exit = abs32(Heading_Raw());
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

    // Learn how far this robot actually carries on after the sweep is cut
    // (brake included). Half-weight: fast enough that the second turn of a
    // run already lands close, slow enough that one scuffed turn cannot drag
    // the estimate far. Implausible values -- a timed-out sweep, a wall
    // strike -- are not learned from.
    {
        int32_t coast = abs32(Heading_Raw()) - sweep_exit;
        if (!res->timed_out &&
            coast > (int32_t)TURN_COAST_MIN_DEG * GYRO_LSB_MS_PER_DEGREE &&
            coast < (int32_t)TURN_COAST_MAX_DEG * GYRO_LSB_MS_PER_DEGREE) {
            s_coast_est = (s_coast_est + coast) / 2;
        }
    }

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
        nudge_ms = (uint16_t)((err_deg_tenths * (int32_t)s_nudge_ms_deg10) / 100L);
        if (nudge_ms < TURN_NUDGE_MS_MIN) nudge_ms = TURN_NUDGE_MS_MIN;
        if (nudge_ms > TURN_NUDGE_MS_MAX) nudge_ms = TURN_NUDGE_MS_MAX;

        {
            int32_t before = abs32(Heading_Raw());
            int32_t moved;
            Power_SetActivity(ACT_TURN_NUDGE);
            pulse_tracked((err > 0) ? cw : !cw, TURN_NUDGE_PWM, nudge_ms, &next_ms);
            settle_tracked(TURN_SETTLE_MS, &next_ms);

            // Learn what a millisecond of nudge is worth on this robot. It is
            // not linear -- the first few ms go into gearbox slack -- so this
            // is an average for the size of nudge actually being used, which
            // is the only size it has to be right for.
            moved = abs32(Heading_Raw()) - before;           // + = towards target
            if (err < 0) moved = -moved;
            moved = (moved * 10L) / GYRO_LSB_MS_PER_DEGREE;  // tenths
            if (moved >= 5) {
                int32_t obs = ((int32_t)nudge_ms * 100L) / moved;   // ms per 10 deg
                int32_t est = ((int32_t)s_nudge_ms_deg10 * 3L + obs) / 4L;
                if (est < TURN_NUDGE_MS_PER_DEG_MIN * 10L) est = TURN_NUDGE_MS_PER_DEG_MIN * 10L;
                if (est > TURN_NUDGE_MS_PER_DEG_MAX * 10L) est = TURN_NUDGE_MS_PER_DEG_MAX * 10L;
                s_nudge_ms_deg10 = (uint16_t)est;
            } else if (nudge_ms < TURN_NUDGE_MS_MAX) {
                // Did not move at all: the pulse never got past the slack.
                uint16_t est = (uint16_t)(s_nudge_ms_deg10 + s_nudge_ms_deg10 / 2u);
                if (est > TURN_NUDGE_MS_PER_DEG_MAX * 10u) est = TURN_NUDGE_MS_PER_DEG_MAX * 10u;
                s_nudge_ms_deg10 = est;
            }
        }
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

// One quarter turn, onto the NEXT GRID HEADING rather than "90 degrees from
// wherever the chassis happens to point".
//
// That difference is the whole fix for a robot that drives off every turn at
// an angle. The drive-off kick, a correction still in progress when the robot
// braked, a turn that stopped 2 degrees short -- each left the chassis a few
// degrees off square, and a relative 90 faithfully carried that error into
// the next leg. Aiming at the grid heading removes it at every turn instead.
static void turn_quarter(turn_dir_t dir, turn_result_t *res) {
    const int32_t quarter = 90L * GYRO_LSB_MS_PER_DEGREE;
    const int32_t maxdev  = (int32_t)TURN_GRID_MAX_DEV_DEG * GYRO_LSB_MS_PER_DEGREE;
    int32_t dev = Heading_GridError();          // + = chassis is left of the grid

    if (dev > maxdev || dev < -maxdev) {
        // Too far off to be heading noise. The gyro saw every degree of it,
        // so the likeliest story is that the chassis really is pointing down
        // a different corridor (a collision recovery, a wall strike): move the
        // grid target by whole quarters to the nearest one. Only if it is
        // still far off -- the robot is diagonal -- give up on the grid and
        // take wherever it points as square.
        Debug_KVF("  grid off by10", Heading_GridErrorTenths());
        while (dev >  quarter / 2) { Heading_GridStep(1);  dev -= quarter; }
        while (dev < -quarter / 2) { Heading_GridStep(-1); dev += quarter; }
        if (dev > maxdev || dev < -maxdev) {
            Debug_P(" -- resync");
            Heading_GridResync();
            dev = 0;
        }
        Debug_NL();
    }
    Heading_GridStep((dir == TURN_LEFT) ? 1 : -1);
    execute_single((dir == TURN_LEFT) ? (quarter - dev) : (quarter + dev), dir, res);
    res->grid_error_tenths = Heading_GridErrorTenths();
}

static void after_turn(turn_result_t *res) {
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
    turn_quarter(dir, res);
    after_turn(res);
}

// Wait between pings, still integrating the gyro. Back-to-back pings pick up
// the previous ping's reverberation off the walls of a small cell.
static void ping_gap(uint32_t *next_ms) {
    uint8_t k;
    for (k = 0; k < UTURN_PING_GAP_MS / TURN_TICK_MS; k++) turn_sample(next_ms);
}

// Median of three immediate front pings, cm.
static uint16_t front3(uint32_t *next_ms) {
    uint16_t a, b, c, t;
    a = Sonar_PingNow(SONAR_FRONT); ping_gap(next_ms);
    b = Sonar_PingNow(SONAR_FRONT); ping_gap(next_ms);
    c = Sonar_PingNow(SONAR_FRONT);
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

// Facing a wall, the front sonar measures exactly how far the AXLE is from
// it. Creep forward or back until that is half a corridor: the pivot point is
// then the centre of the cell, whatever the robot's coasting did.
//
// Used before every pivot at a front wall (the stop distance depends on how
// far this robot rolls after the brake, which varies with battery and floor),
// and half-way through a dead-end 180, where the robot faces one side wall.
//
// This is what makes the 180 clean in a 40 cm pocket. The pivot sweeps a
// ~17 cm radius, which leaves ~3 cm each side of a perfectly centred axle --
// and the axle is rarely perfectly centred: the side sonars sit ahead of it,
// so a slightly skewed robot reads "centred" while its pivot point is not,
// and the first quarter itself skids the pivot point a few cm.
void Turn_CentreOnWall(void) {
    uint32_t next_ms = millis() + TURN_TICK_MS;
    uint8_t  i;
    for (i = 0; i < UTURN_CENTRE_TRIES; i++) {
        uint16_t f = front3(&next_ms);
        int16_t  err;
        uint16_t ms;
        if (f == SONAR_NO_ECHO || f > UTURN_CENTRE_MAX_CM) return;  // not facing a wall
        err = (int16_t)f - (int16_t)UTURN_CENTRE_FRONT_CM;          // + = too far back
        if (err >= -UTURN_CENTRE_TOL_CM && err <= UTURN_CENTRE_TOL_CM) return;
        ms = (uint16_t)(((uint32_t)(err < 0 ? -err : err) * 1000UL) / UTURN_CENTRE_SPEED_CMS);
        if (ms < KICK_MS) ms = KICK_MS;
        if (ms > UTURN_CENTRE_MAX_MS) ms = UTURN_CENTRE_MAX_MS;
        Debug_KVF("  centre front", (int32_t)f);
        Debug_KVF("ms", (int32_t)((err > 0) ? ms : -(int32_t)ms));
        Debug_NL();
        {
            uint32_t t0 = millis();
            motor_dir_t d = (err > 0) ? DIR_FWD : DIR_REV;
            // Breakaway kick first -- the chassis stalls below ~60 PWM from
            // rest -- RAMPED like every other kick: a step to KICK_PWM with
            // both wheels stalled is the largest current spike there is, and
            // the kicks are where this robot browns out.
            Power_SetActivity(ACT_DRIVE_KICK);
            while ((millis() - t0) < KICK_MS) {
                uint8_t p = (uint8_t)(MOTOR_MIN_PWM +
                    (((uint32_t)(KICK_PWM - MOTOR_MIN_PWM) * (millis() - t0)) / KICK_MS));
                Motors_SetLeft(d, p);
                Motors_SetRight(d, p);
                Power_Task();
                turn_sample(&next_ms);
            }
            Power_SetActivity(ACT_DRIVING);
            Motors_SetLeft(d, UTURN_CENTRE_PWM);
            Motors_SetRight(d, UTURN_CENTRE_PWM);
            while ((millis() - t0) < ms) turn_sample(&next_ms);
            Motors_Stop();
            Power_SetActivity(ACT_IDLE);
            settle_tracked(GYRO_SETTLE_MS, &next_ms);
        }
    }
}

void Turn_180(turn_dir_t dir, turn_result_t *res) {
    // Two quarter turns with a settle between: momentum has less time to
    // build, so there is less coast to correct for -- and the second quarter
    // aims at the grid, so it also cancels whatever the first one left over.
    turn_result_t a, b;
    turn_quarter(dir, &a);
    Turn_CentreOnWall();
    turn_quarter(dir, &b);
    res->achieved_tenths      = a.achieved_tenths + b.achieved_tenths;
    res->nudges_used          = (uint8_t)(a.nudges_used + b.nudges_used);
    res->timed_out            = a.timed_out | b.timed_out;
    res->initial_error_tenths = a.initial_error_tenths + b.initial_error_tenths;
    res->peak_rate            = (a.peak_rate > b.peak_rate) ? a.peak_rate : b.peak_rate;
    res->wrong_way            = a.wrong_way | b.wrong_way;
    res->coast_ms             = (a.coast_ms > b.coast_ms) ? a.coast_ms : b.coast_ms;
    res->final_error_tenths   = b.final_error_tenths;
    res->converged            = a.converged & b.converged;
    res->grid_error_tenths    = b.grid_error_tenths;
    after_turn(res);
}
