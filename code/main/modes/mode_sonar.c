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

// Build mode 0 -- sonar telemetry only, motors never energised. The baseline
// every other mode is debugged against: if the numbers are wrong here, nothing
// built on top of them can be right.
void Mode_PreGyro(void) { }
void Mode_Begin(void) { }
void Mode_Header(void) { Telemetry_Header(); }

void Mode_Tick(const tick_ctx_t *t) {
    int16_t  rate      = t->rate;
    uint32_t run_start = t->run_start;
    (void)rate; (void)run_start;
        // Mode 4: motors permanently off. Sonar/heading above still run every
        // tick, so rotating the chassis by hand is exactly what the cone test
        // needs -- only the telemetry format differs (see below).
        Motors_Stop();
        (void)rate;
}

void Mode_Telemetry(const tick_ctx_t *t) {
    (void)t;
    Telemetry_Tick(t);
}
