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

// Build mode 7 -- drive the corridor until the front is blocked, stop, turn
// right, drive one more leg. One cycle, then halt.
void Mode_PreGyro(void) { }
void Mode_Begin(void) { }
void Mode_Header(void) { Telemetry_Header(); }

void Mode_Tick(const tick_ctx_t *t) {
    int16_t  rate      = t->rate;
    uint32_t run_start = t->run_start;
    (void)rate; (void)run_start;
        {
            static uint8_t state      = 0;   // 0 not started, 1 approaching, 2 done
            static uint8_t block_hits = 0;

            if (state == 0 && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                state = 1;
                Debug_P("MODE7: approaching obstacle\r\n");
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
                    Debug_P("MODE7: obstacle detected, stopping\r\n");

                    // Blocking, same as maze.c's own turn handling -- Turn_90
                    // already flushes the sonar filters and resets heading
                    // when it returns, so nothing pointed the wrong way
                    // carries into the leg below.
                    Debug_P("MODE7: turning right\r\n");
                    Turn_90(TURN_RIGHT, &r);
                    Debug_KVF("ang10", r.achieved_tenths);
                    Debug_KVF("wrong", r.wrong_way);
                    Debug_NL();

                    // Forward leg under gyro heading hold, same as Mode 5's
                    // legs, carrying the turn's leftover error in so it gets
                    // steered out rather than baked into the heading.
                    Debug_P("MODE7: forward leg\r\n");
                    Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS,
                                       r.residual_raw);

                    Debug_P("MODE7 DONE\r\n");
                    state = 2;
                } else {
                    Drive_Tick(rate);
                }
            }
            // state 2: motors stay off; telemetry keeps printing below
        }
}

void Mode_Telemetry(const tick_ctx_t *t) {
    (void)t;
    Telemetry_Tick(t);
}
