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

// Build mode 6 -- repeats one pivot with a measuring pause after each, and
// streams the whole yaw-rate profile. Nothing but the turn.
void Mode_PreGyro(void) { }
void Mode_Header(void) { Telemetry_NoneHeader(); }

void Mode_Begin(void) {
    // Runs to completion here and never returns -- this mode owns the
    // whole run, so there is nothing for the control loop to do.
    {
        uint8_t n;

        Turn_SampleTrace(1);

        Debug_P("# TURN DEBUG  angle=");   Debug_Int(TURNDBG_ANGLE);
        Debug_P(" repeats=");              Debug_Int(TURNDBG_REPEATS);
        Debug_P(" alt=");                  Debug_Int(TURNDBG_ALTERNATE);
        Debug_P(" decim=");                Debug_Int(TURNDBG_SAMPLE_EVERY);
        Debug_NL();
        Debug_P("# S,ms,rate,hdg10  <- per-sample stream (rate is raw LSB, "
                  "65.5 per deg/sec)\r\n");
        Debug_P("# T <phase> hdg10=..    <- phase boundary\r\n");
        Debug_P("# SUM ..                <- per-turn summary\r\n");
        Debug_P("# tape a reference line on the floor, protractor each turn "
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

            Debug_P("=== turn ");
            Debug_Int(n + 1);
            if (dir == TURN_RIGHT) Debug_P(" RIGHT ===\r\n");
            else                   Debug_P(" LEFT ===\r\n");

            Turn_Execute(TURNDBG_ANGLE, dir, &r);

            Debug_P("SUM ");
            Debug_KVF("n",       n + 1);
            Debug_KVF("dirR",    (dir == TURN_RIGHT) ? 1 : 0);
            Debug_KVF("ang10",   r.achieved_tenths);
            Debug_KVF("err10",   r.initial_error_tenths);
            Debug_KVF("nudges",  r.nudges_used);
            Debug_KVF("peak",    r.peak_rate);
            Debug_KVF("coastms", r.coast_ms);
            // fin10 is what was still left when the nudge loop stopped, and
            // conv=0 means it ran out of nudges before reaching the deadband.
            Debug_KVF("fin10",   r.final_error_tenths);
            Debug_KVF("conv",    r.converged);
            Debug_KVF("wrong",   r.wrong_way);
            Debug_KVF("recal",   r.recal_ok);
            Debug_KVF("to",      r.timed_out);
            // Cumulative since boot, so compare it turn to turn: any increase
            // means the per-sample stream lost bytes during that turn and its
            // S lines cannot be trusted -- raise TURNDBG_SAMPLE_EVERY.
            Debug_KVF("drop",    Debug_Dropped());
            Debug_NL();

            Debug_P("measure the angle now\r\n");
            Timer_WaitMs(TURNDBG_PAUSE_MS);
        }

        Debug_P("TURN DEBUG DONE\r\n");
        Motors_Stop();
        for (;;) { }
    }
}
void Mode_Tick(const tick_ctx_t *t) { (void)t; }

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
