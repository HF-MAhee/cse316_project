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
        Timer_WaitMs(STARTUP_DELAY_MS);
        Debug_Str("SQUARE TEST: ");
        Debug_Int(SQUARE_SIDES);
        Debug_Str(" sides, forward + right turn each\r\n");

        for (leg = 0; leg < SQUARE_SIDES; leg++) {
            turn_result_t r;

            Debug_Str("leg "); Debug_Int(leg + 1); Debug_Str(" forward\r\n");
            // Breakaway kick, same idea as Drive_Begin(): the motors will not
            // start moving from rest at cruise PWM alone.
            Motors_Forward(KICK_PWM, KICK_PWM);
            Timer_WaitMs(KICK_MS);
            Motors_Forward(SQUARE_TEST_PWM, SQUARE_TEST_PWM);
            Timer_WaitMs(SQUARE_LEG_MS - KICK_MS);
            Motors_Stop();
            Timer_WaitMs(SQUARE_TURN_SETTLE_MS);

            Debug_Str("leg "); Debug_Int(leg + 1); Debug_Str(" turn\r\n");
            Turn_90(TURN_RIGHT, &r);
            Debug_KV("ang10", r.achieved_tenths);
            Debug_KV("err10", r.initial_error_tenths);
            Debug_KV("nudges", r.nudges_used);
            Debug_KV("peak", r.peak_rate);
            Debug_KV("wrong", r.wrong_way);
            Debug_NL();
        }

        Debug_Str("SQUARE TEST DONE\r\n");
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

                    // Open-loop forward leg, same kick+cruise shape and the
                    // same fixed speed/duration as the Mode 5 square test --
                    // this is the same "prove the drive works" leg, just
                    // triggered by an obstacle instead of a fixed repeat count.
                    Debug_Str("MODE7: forward leg\r\n");
                    Motors_Forward(KICK_PWM, KICK_PWM);
                    Timer_WaitMs(KICK_MS);
                    Motors_Forward(SQUARE_TEST_PWM, SQUARE_TEST_PWM);
                    Timer_WaitMs(SQUARE_LEG_MS - KICK_MS);
                    Motors_Stop();

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
