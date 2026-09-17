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

// Build mode 3 -- the full maze solver. Everything else in modes/ exists to
// de-risk one part of this one.
void Mode_PreGyro(void) { }
void Mode_Header(void) { Telemetry_Header(); }
void Mode_Begin(void)  { Maze_Init(); }

void Mode_Tick(const tick_ctx_t *t) {
    Maze_Tick(t->rate);
}

void Mode_Telemetry(const tick_ctx_t *t) {
    (void)t;
    Telemetry_Tick(t);
}
