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
// ---------------------------------------------------------------------------
#define BUILD_MODE 1

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
    Debug_Str("# st,L,F,R,lok,rok,near,md,br,err,wt,gt,corr,pwmL,pwmR,rock,rate,gx,gy,ovr,drop");
    Debug_NL();
}

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
        Debug_KV("nudges", r.nudges_used);
        Debug_NL();
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
#if BUILD_MODE == 0
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
#else
        Maze_Tick(rate);
#endif

        // Measure how long the tick's real work took. If this exceeds the
        // budget the control loop is no longer running at a fixed rate, which
        // silently invalidates the gyro integration and the PD tuning.
        if ((millis() - tick_start) > TICK_OVERRUN_WARN_MS) {
            if (overruns < 0xFFFF) overruns++;
        }

        telemetry(rate, g.x, g.y, overruns);

        // ---- global safety ----------------------------------------------
        if ((millis() - run_start) > MAX_RUN_MS) {
            Motors_Stop();
            Debug_Str("RUN LIMIT\r\n");
            for (;;) { }
        }
    }
}
