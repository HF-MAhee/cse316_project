#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "i2c.h"
#include "mpu6050.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "maze.h"
#include "debug.h"

// ---------------------------------------------------------------------------
//  Build mode. Work through these in order -- each proves one phase on
//  hardware before the next depends on it.
//    0 = sonar telemetry only (no motion)
//    1 = wall centring in a straight corridor
//    2 = single turn accuracy test
//    3 = full maze solver
//    4 = sonar cone characterization -- motors OFF, rotate the chassis BY
//        HAND and watch heading vs L/F/R to find the angle where a beam
//        picks up the wrong wall
//    5 = open-loop square drive test -- no sonar, no corridor: 4x (2s
//        forward + 90 degree turn) to prove raw drive/turn capability
//    6 = turn debugging -- repeats one pivot with a measuring pause after
//        each, streaming the full yaw-rate profile. Nothing but the turn.
//    7 = single obstacle-avoidance cycle -- wall-centred drive straight until
//        the front is blocked, stop, turn right 90, then drive forward
//        open-loop for one fixed leg. One cycle, then halts (not a loop).
// ---------------------------------------------------------------------------
#define BUILD_MODE 3

// Decodes MCUCSR at boot. This is the definitive answer to "did it reset, and
// why" -- guesswork from log gaps is not needed once this prints. BORF set
// means brown-out: the supply sagged below the MCU threshold, which is a
// power/wiring fault, not a firmware bug.
static void report_reset_cause(void) {
    uint8_t f = MCUCSR;
    MCUCSR = 0;                     // must clear, or flags accumulate forever
    Debug_Str("RESET:");
    if (f & (1 << PORF))  Debug_Str(" power-on");
    if (f & (1 << EXTRF)) Debug_Str(" external");
    if (f & (1 << BORF))  Debug_Str(" BROWNOUT");
    if (f & (1 << WDRF))  Debug_Str(" watchdog");
    if (f == 0)           Debug_Str(" (none/unknown)");
    Debug_KV("  raw", f);
    Debug_NL();
}

static void telemetry_header(void) {
#if BUILD_MODE == 4
    Debug_Str("# hdg10,L,F,R,Lopen,Ropen,Ltoo,Rtoo,Fblocked,Fvotes");
#elif BUILD_MODE == 2 || BUILD_MODE == 5 || BUILD_MODE == 6
    Debug_Str("# no periodic CSV in this mode -- event and summary lines only");
#else
    Debug_Str("# st,L,F,R,lok,rok,near,fv,md,br,err,wt,gt,corr,pwmL,pwmR,rock,rate,gx,gy,ovr,drop");
#endif
    Debug_NL();
}

#if BUILD_MODE == 4
// Mode 4 telemetry: heading (tenths of a degree, accumulated since boot) and
// RAW (not median-filtered) L/F/R so a single bad ping from a beam edge is
// visible rather than smoothed away -- the whole point here is finding
// exactly where the cone edges are, not steering on clean data.
static void telemetry_sonar_test(void) {
    static uint32_t last = 0;
    if ((millis() - last) < SONAR_TEST_INTERVAL_MS) return;
    last = millis();

    Debug_CSV(Heading_DegreesTenths());
    Debug_CSV(Sonar_Latest(SONAR_LEFT));
    Debug_CSV(Sonar_Latest(SONAR_FRONT));
    Debug_CSV(Sonar_Latest(SONAR_RIGHT));
    Debug_CSV(Sonar_IsOpen(SONAR_LEFT));
    Debug_CSV(Sonar_IsOpen(SONAR_RIGHT));
    Debug_CSV(Sonar_IsTooClose(SONAR_LEFT));
    Debug_CSV(Sonar_IsTooClose(SONAR_RIGHT));
    Debug_CSV(Sonar_FrontBlocked());
    Debug_Int(Sonar_FrontVotes());
    Debug_NL();
}
#endif

#if BUILD_MODE != 4
static void telemetry(int16_t gyro_rate, int16_t gx, int16_t gy, uint16_t overruns) {
    static uint32_t last = 0;
    const drive_debug_t *d;
    if ((millis() - last) < TELEMETRY_INTERVAL_MS) return;
    last = millis();

#if DEBUG_LEVEL >= 1
    d = Drive_Debug();
    Debug_CSV((int32_t)Maze_State());
    Debug_CSV(Sonar_Median(SONAR_LEFT));
    Debug_CSV(Sonar_Median(SONAR_FRONT));
    Debug_CSV(Sonar_Median(SONAR_RIGHT));
    Debug_CSV(Sonar_IsValid(SONAR_LEFT));
    Debug_CSV(Sonar_IsValid(SONAR_RIGHT));
    // one field for both too-close flags: 0 none, 1 left, 2 right, 3 both
    Debug_CSV((Sonar_IsTooClose(SONAR_LEFT) ? 1 : 0) |
              (Sonar_IsTooClose(SONAR_RIGHT) ? 2 : 0));
    Debug_CSV(Sonar_FrontVotes());
    Debug_CSV((int32_t)Drive_Mode());
    Debug_CSV(d->branch);
    Debug_CSV(d->error_cm);
    Debug_CSV(d->wall_term);
    Debug_CSV(d->gyro_term);
    Debug_CSV(d->corr);
    Debug_CSV(d->pwm_l);
    Debug_CSV(d->pwm_r);
    Debug_CSV(Motion_IsSuspect());
    Debug_CSV(gyro_rate);
    Debug_CSV(gx);
    Debug_CSV(gy);
    Debug_CSV(overruns);
    Debug_Int(Debug_Dropped());
    Debug_NL();
#else
    (void)gyro_rate; (void)gx; (void)gy; (void)overruns; (void)d;
#endif
}
#endif // BUILD_MODE != 4

int main(void) {
    uint32_t next_tick;
    uint32_t run_start;

    Debug_Init();
    Timer_Init();          // before sei() so millis() is live immediately
    I2C_Init();
    Motors_Init();
    Motors_Stop();
    Sonar_Init();
    sei();

    Debug_Str("\r\n=== AGV maze solver ===\r\n");
    report_reset_cause();

    MPU6050_Init();
    Debug_Str("calibrating gyro, hold still...\r\n");
    Gyro_CalibrateFull();
    Debug_KV("offZ", Gyro_GetOffset());
    Debug_KV("offX", Gyro_GetOffsetX());
    Debug_KV("offY", Gyro_GetOffsetY());
    Debug_NL();

    Maze_Init();
    telemetry_header();
    next_tick = millis();
    run_start = millis();

#if BUILD_MODE == 2
    {
        turn_result_t r;
        Timer_WaitMs(STARTUP_DELAY_MS);
        Turn_90(TURN_RIGHT, &r);
        Debug_Str("turn test ");
        Debug_KV("ang10", r.achieved_tenths);
        // err10 > 0: the fixed early-stop + coast undershot this run, needed
        // more rotation. err10 < 0: it overshot, needed a reverse nudge.
        // Consistently one sign across repeated runs -> retune
        // TURN_STOP_MARGIN_DEG in that direction.
        Debug_KV("err10", r.initial_error_tenths);
        Debug_KV("nudges", r.nudges_used);
        // peak near 32767 = the gyro clipped at +/-500 dps, so the heading
        // under-read and the chassis physically overshot. wrong=1 = it rotated
        // the opposite way to the one commanded.
        Debug_KV("peak", r.peak_rate);
        Debug_KV("wrong", r.wrong_way);
        Debug_NL();
        Motors_Stop();
        for (;;) { }
    }
#elif BUILD_MODE == 5
    {
        uint8_t leg;
        int32_t carry = 0;      // previous turn's leftover, in this leg's frame
        int32_t total_raw = 0;  // rotation over the WHOLE path, legs and turns
        // Per-phase record, replayed as a table at the end so the whole run is
        // readable without scrolling back through the sample stream.
        int16_t leg_off10[SQUARE_SIDES], leg_net10[SQUARE_SIDES];
        int16_t trn_ang10[SQUARE_SIDES], trn_fin10[SQUARE_SIDES];
        uint8_t trn_nudge[SQUARE_SIDES], trn_conv[SQUARE_SIDES];

        // Self-describing header: every tunable that shapes this run, so the
        // log can be analysed later without having to guess the build.
        Debug_Str("# SQUARE TEST  sides=");     Debug_Int(SQUARE_SIDES);
        Debug_Str(" pwm=");                     Debug_Int(SQUARE_TEST_PWM);
        Debug_Str(" legms=");                   Debug_Int(SQUARE_LEG_MS);
        Debug_NL();
        Debug_Str("# hold: tick=");              Debug_Int(HOLD_TICK_MS);
        Debug_Str(" kp=");                       Debug_Int(HOLD_KP_NUM);
        Debug_Str("/");                          Debug_Int(HOLD_KP_DEN);
        Debug_Str(" kd=");                       Debug_Int(WALL_KD_NUM);
        Debug_Str("/");                          Debug_Int(WALL_KD_DEN);
        Debug_Str(" maxcorr=");                  Debug_Int(WALL_MAX_CORRECTION);
        Debug_Str(" pwmfloor=");                 Debug_Int(MOTOR_MIN_PWM);
        Debug_Str(" pwmceil=");                  Debug_Int(MOTOR_MAX_PWM);
        Debug_NL();
        Debug_Str("# turn: margin=");            Debug_Int(TURN_STOP_MARGIN_DEG);
        Debug_Str(" deadband=");                 Debug_Int(TURN_DEADBAND_DEG);
        Debug_Str(" settle=");                   Debug_Int(TURN_SETTLE_MS);
        Debug_Str(" pwm=");                      Debug_Int(TURN_PWM);
        Debug_Str(" lsbms_per_deg=");             Debug_Int(GYRO_LSB_MS_PER_DEGREE);
        Debug_NL();
        Debug_Str("# L,ms,err10,rate,corr,pwmL,pwmR   <- heading-hold sample\r\n");
        Debug_Str("# S,ms,rate,hdg10                  <- turn sample\r\n");
        Debug_Str("# T <phase> / HOLD / TURN / SQUARE <- phase and summary\r\n");
        // ~600 bytes of header against a 192 byte ring buffer: flush or most
        // of it is dropped. Safe here, nothing is moving yet.
        Debug_Flush();

#if SQUARE_TRACE_TURNS
        Turn_SampleTrace(1);
#endif
        Timer_WaitMs(STARTUP_DELAY_MS);

        for (leg = 0; leg < SQUARE_SIDES; leg++) {
            turn_result_t r;
            int32_t net_raw;

            Debug_Str("=== leg "); Debug_Int(leg + 1); Debug_Str(" forward ===\r\n");
            // Gyro heading hold, NOT open-loop. Two motors at equal PWM never
            // track straight (gearbox, tyre and friction mismatch), which is
            // why the original firmware had a straight-line autocorrect at
            // all. `carry` feeds the previous turn's residual in so the leg
            // steers it out rather than baking it into the path.
            net_raw    = Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS, carry);
            total_raw += net_raw;
            Timer_WaitMs(SQUARE_TURN_SETTLE_MS);

            Debug_Str("=== leg "); Debug_Int(leg + 1); Debug_Str(" turn ===\r\n");
            Turn_90(TURN_RIGHT, &r);
            // A right turn rotates negative, so subtract it from the running
            // total: four clean corners plus four straight legs must land on
            // -3600 tenths.
            total_raw -= (int32_t)r.achieved_tenths * GYRO_LSB_MS_PER_DEGREE / 10;

            Debug_Str("  TURN ");
            Debug_KV("ang10",  r.achieved_tenths);
            Debug_KV("err10",  r.initial_error_tenths);
            Debug_KV("fin10",  r.final_error_tenths);
            Debug_KV("conv",   r.converged);
            Debug_KV("nudges", r.nudges_used);
            Debug_KV("peak",   r.peak_rate);
            Debug_KV("coastms", r.coast_ms);
            Debug_KV("wrong",  r.wrong_way);
            Debug_KV("recal",  r.recal_ok);
            Debug_KV("to",     r.timed_out);
            Debug_NL();
            Debug_Str("  CUM ");
            Debug_KV("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
            Debug_NL();

            leg_off10[leg] = (int16_t)(((net_raw + carry) * 10L) / GYRO_LSB_MS_PER_DEGREE);
            leg_net10[leg] = (int16_t)((net_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
            trn_ang10[leg] = (int16_t)r.achieved_tenths;
            trn_fin10[leg] = (int16_t)r.final_error_tenths;
            trn_nudge[leg] = r.nudges_used;
            trn_conv[leg]  = r.converged;

            // Hand this turn's leftover to the next leg instead of discarding
            // it -- Turn_Execute() zeroes the accumulator, so without this the
            // residual from every corner accumulates into the square.
            carry = r.residual_raw;
        }

        // Whole-run table. legoff = how far off its held heading each leg
        // finished; legnet = how much it actually rotated (should be ~0);
        // ang/fin/nud/cnv are that corner's turn.
        Debug_NL();
        Debug_Str("# SQUARE SUMMARY  leg,legoff10,legnet10,ang10,fin10,nudges,conv\r\n");
        for (leg = 0; leg < SQUARE_SIDES; leg++) {
            Debug_Str("SQUARE,");
            Debug_Int(leg + 1);          Debug_Str(",");
            Debug_Int(leg_off10[leg]);   Debug_Str(",");
            Debug_Int(leg_net10[leg]);   Debug_Str(",");
            Debug_Int(trn_ang10[leg]);   Debug_Str(",");
            Debug_Int(trn_fin10[leg]);   Debug_Str(",");
            Debug_Int(trn_nudge[leg]);   Debug_Str(",");
            Debug_Int(trn_conv[leg]);
            Debug_NL();
        }
        // The single number that says whether the square closed in HEADING:
        // -3600 tenths is a perfect four-corner circuit. Position can still be
        // off even at -3600, since heading hold does not correct sideways
        // displacement and battery sag makes later legs shorter.
        Debug_Str("SQUARE TOTAL ");
        Debug_KV("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
        Debug_KV("ideal10", -3600L);
        Debug_KV("drop", Debug_Dropped());
        Debug_NL();
        Debug_Str("SQUARE TEST DONE\r\n");
        Debug_Flush();
        Motors_Stop();
        for (;;) { }
    }
#elif BUILD_MODE == 6
    {
        uint8_t n;

        Turn_SampleTrace(1);

        Debug_Str("# TURN DEBUG  angle=");   Debug_Int(TURNDBG_ANGLE);
        Debug_Str(" repeats=");              Debug_Int(TURNDBG_REPEATS);
        Debug_Str(" alt=");                  Debug_Int(TURNDBG_ALTERNATE);
        Debug_Str(" decim=");                Debug_Int(TURNDBG_SAMPLE_EVERY);
        Debug_NL();
        Debug_Str("# S,ms,rate,hdg10  <- per-sample stream (rate is raw LSB, "
                  "65.5 per deg/sec)\r\n");
        Debug_Str("# T <phase> hdg10=..    <- phase boundary\r\n");
        Debug_Str("# SUM ..                <- per-turn summary\r\n");
        Debug_Str("# tape a reference line on the floor, protractor each turn "
                  "during the pause\r\n");
        // These header lines are ~350 bytes pushed back to back, which is more
        // than DEBUG_TX_BUF (192) can hold while the UART drains at 38400 --
        // a measured run lost 165 bytes of them and garbled the legend. Safe
        // to spin here: nothing is moving yet.
        Debug_Flush();
        Timer_WaitMs(STARTUP_DELAY_MS);

        for (n = 0; n < TURNDBG_REPEATS; n++) {
            turn_result_t r;
            turn_dir_t dir = (TURNDBG_ALTERNATE && (n & 1)) ? TURN_LEFT : TURN_RIGHT;

            Debug_Str("=== turn ");
            Debug_Int(n + 1);
            Debug_Str((dir == TURN_RIGHT) ? " RIGHT ===\r\n" : " LEFT ===\r\n");

            Turn_Execute(TURNDBG_ANGLE, dir, &r);

            Debug_Str("SUM ");
            Debug_KV("n",       n + 1);
            Debug_KV("dirR",    (dir == TURN_RIGHT) ? 1 : 0);
            Debug_KV("ang10",   r.achieved_tenths);
            Debug_KV("err10",   r.initial_error_tenths);
            Debug_KV("nudges",  r.nudges_used);
            Debug_KV("peak",    r.peak_rate);
            Debug_KV("coastms", r.coast_ms);
            // fin10 is what was still left when the nudge loop stopped, and
            // conv=0 means it ran out of nudges before reaching the deadband.
            Debug_KV("fin10",   r.final_error_tenths);
            Debug_KV("conv",    r.converged);
            Debug_KV("wrong",   r.wrong_way);
            Debug_KV("recal",   r.recal_ok);
            Debug_KV("to",      r.timed_out);
            // Cumulative since boot, so compare it turn to turn: any increase
            // means the per-sample stream lost bytes during that turn and its
            // S lines cannot be trusted -- raise TURNDBG_SAMPLE_EVERY.
            Debug_KV("drop",    Debug_Dropped());
            Debug_NL();

            Debug_Str("measure the angle now\r\n");
            Timer_WaitMs(TURNDBG_PAUSE_MS);
        }

        Debug_Str("TURN DEBUG DONE\r\n");
        Motors_Stop();
        for (;;) { }
    }
#endif

    for (;;) {
        gyro_xyz_t g;
        int16_t rate;
        uint32_t tick_start;
        static uint16_t overruns = 0;

        // ---- fixed control tick -----------------------------------------
        if ((int32_t)(millis() - next_tick) < 0) continue;
        next_tick += CONTROL_TICK_MS;
        tick_start = millis();
        // If we fell badly behind (a long sonar timeout, say), re-base rather
        // than firing a burst of catch-up ticks with no spacing.
        if ((int32_t)(millis() - next_tick) > (int32_t)(CONTROL_TICK_MS * 3)) {
            next_tick = millis() + CONTROL_TICK_MS;
        }

        // ---- sensors -----------------------------------------------------
        MPU6050_ReadAll(&g);
        Motion_Update(&g);                     // pitch/roll -> rocking flag
        Heading_Add(g.z, CONTROL_TICK_MS);
        rate = Gyro_Rate(g.z);

        Sonar_Task();                          // exactly one ping per tick

        // ---- behaviour ---------------------------------------------------
#if BUILD_MODE == 0 || BUILD_MODE == 4
        // Mode 4: motors permanently off. Sonar/heading above still run every
        // tick, so rotating the chassis by hand is exactly what the cone test
        // needs -- only the telemetry format differs (see below).
        Motors_Stop();
        (void)rate;
#elif BUILD_MODE == 1
        {
            static uint8_t begun      = 0;
            static uint8_t stopped    = 0;
            static uint8_t block_hits = 0;

            if (!begun && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                begun = 1;
            }

            if (begun && !stopped) {
                // Debounced the same way junction detection is in maze.c --
                // a single bad ping should not slam the brakes mid-test.
                if (Sonar_FrontBlocked()) {
                    if (block_hits < 255) block_hits++;
                } else {
                    block_hits = 0;
                }

                if (block_hits >= FRONT_STOP_CONFIRM) {
                    Drive_Stop();
                    stopped = 1;
                    Debug_Str("MODE1: front obstacle -- stopped\r\n");
                } else {
                    Drive_Tick(rate);
                }
            }
            // once stopped, motors stay off; telemetry keeps printing below
        }
#elif BUILD_MODE == 7
        {
            static uint8_t state      = 0;   // 0 not started, 1 approaching, 2 done
            static uint8_t block_hits = 0;

            if (state == 0 && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                state = 1;
                Debug_Str("MODE7: approaching obstacle\r\n");
            }

            if (state == 1) {
                // Same debounce as Mode 1 -- a single bad ping should not
                // trigger the turn early.
                if (Sonar_FrontBlocked()) {
                    if (block_hits < 255) block_hits++;
                } else {
                    block_hits = 0;
                }

                if (block_hits >= FRONT_STOP_CONFIRM) {
                    turn_result_t r;

                    Drive_Stop();
                    Debug_Str("MODE7: obstacle detected, stopping\r\n");

                    // Blocking, same as maze.c's own turn handling -- Turn_90
                    // already flushes the sonar filters and resets heading
                    // when it returns, so nothing pointed the wrong way
                    // carries into the leg below.
                    Debug_Str("MODE7: turning right\r\n");
                    Turn_90(TURN_RIGHT, &r);
                    Debug_KV("ang10", r.achieved_tenths);
                    Debug_KV("wrong", r.wrong_way);
                    Debug_NL();

                    // Forward leg under gyro heading hold, same as Mode 5's
                    // legs, carrying the turn's leftover error in so it gets
                    // steered out rather than baked into the heading.
                    Debug_Str("MODE7: forward leg\r\n");
                    Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS,
                                       r.residual_raw);

                    Debug_Str("MODE7 DONE\r\n");
                    state = 2;
                } else {
                    Drive_Tick(rate);
                }
            }
            // state 2: motors stay off; telemetry keeps printing below
        }
#else
        Maze_Tick(rate);
#endif

        // Measure how long the tick's real work took. If this exceeds the
        // budget the control loop is no longer running at a fixed rate, which
        // silently invalidates the gyro integration and the PD tuning.
        if ((millis() - tick_start) > TICK_OVERRUN_WARN_MS) {
            if (overruns < 0xFFFF) overruns++;
        }

#if BUILD_MODE == 4
        telemetry_sonar_test();
#else
        telemetry(rate, g.x, g.y, overruns);
#endif

        // ---- global safety ----------------------------------------------
        if ((millis() - run_start) > MAX_RUN_MS) {
            Motors_Stop();
            Debug_Str("RUN LIMIT\r\n");
            for (;;) { }
        }
    }
}
