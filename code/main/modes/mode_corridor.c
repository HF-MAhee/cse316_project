#include "config.h"
#include <avr/io.h>
#include "mode.h"
#include "telemetry.h"
#include "resetlog.h"
#include "timer.h"
#include "mpu6050.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "maze.h"
#include "debug.h"
#include "power.h"
#include "gyrodiag.h"

// Build mode 1 -- wall centring in a straight corridor, stopping at the first
// front obstacle. Proves the PD controller before any turning is involved.
void Mode_PreGyro(void) { }
void Mode_Begin(void) { }
void Mode_Header(void) { Telemetry_Header(); }

void Mode_Tick(const tick_ctx_t *t) {
    int16_t  rate      = t->rate;
    uint32_t run_start = t->run_start;
    (void)rate; (void)run_start;
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
                    Debug_P("MODE1: front obstacle -- stopped\r\n");
                } else {
                    Drive_Tick(rate);
                }
            }
            // once stopped, motors stay off; telemetry keeps printing below
        }
}

void Mode_Telemetry(const tick_ctx_t *t) {
    (void)t;
    Telemetry_Tick(t);
}
