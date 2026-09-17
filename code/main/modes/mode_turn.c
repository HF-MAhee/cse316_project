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

// Build mode 2 -- one 90 degree pivot, then halt. The simplest possible read on
// whether TURN_STOP_MARGIN_DEG is guessing the coast correctly.
void Mode_PreGyro(void) { }
void Mode_Header(void) { Telemetry_NoneHeader(); }

void Mode_Begin(void) {
    // Runs to completion here and never returns -- this mode owns the
    // whole run, so there is nothing for the control loop to do.
    {
        turn_result_t r;
        Timer_WaitMs(STARTUP_DELAY_MS);
        Turn_90(TURN_RIGHT, &r);
        Debug_P("turn test ");
        Debug_KVF("ang10", r.achieved_tenths);
        // err10 > 0: the fixed early-stop + coast undershot this run, needed
        // more rotation. err10 < 0: it overshot, needed a reverse nudge.
        // Consistently one sign across repeated runs -> retune
        // TURN_STOP_MARGIN_DEG in that direction.
        Debug_KVF("err10", r.initial_error_tenths);
        Debug_KVF("nudges", r.nudges_used);
        // peak near 32767 = the gyro clipped at +/-500 dps, so the heading
        // under-read and the chassis physically overshot. wrong=1 = it rotated
        // the opposite way to the one commanded.
        Debug_KVF("peak", r.peak_rate);
        Debug_KVF("wrong", r.wrong_way);
        Debug_NL();
        Motors_Stop();
        for (;;) { }
    }
}
void Mode_Tick(const tick_ctx_t *t) { (void)t; }

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
