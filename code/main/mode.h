#ifndef MODE_H
#define MODE_H
#include <stdint.h>

// ============================================================================
//  THE BUILD-MODE CONTRACT
//
//  Every test mode lives in its own file under modes/, and exactly ONE of them
//  is compiled into any image. The Makefile picks it:
//
//      make MODE=deadend          build modes/mode_deadend.c
//      make MODE=deadend flash    build it and flash it
//      make list-modes            show what is available
//
//  WHY COMPILE-TIME AND NOT A FUNCTION-POINTER TABLE. A table would reference
//  every mode, so the linker would keep all of them: on a part with 32 KB of
//  flash and 2 KB of SRAM, the modes this project already has would not fit
//  together, and each one's string literals and statics would be carried around
//  by the ten builds that never run it. One mode per image keeps every build as
//  small as it was when the selection was a #if chain in main.c.
//
//  Each mode file defines all four entry points below -- empty ones are fine,
//  and cost a single `ret`. main.c calls them in this order:
//
//      Mode_PreGyro()   before MPU6050_Init() and gyro calibration
//      Mode_Header()    after calibration; print the telemetry legend here
//      Mode_Begin()     last thing before the control loop starts
//      Mode_Tick()      once per CONTROL_TICK_MS -- BEHAVIOUR only
//      Mode_Telemetry() same tick, after the overrun deadline is measured
//
//  Tick and Telemetry are separate on purpose. The overrun check sits between
//  them, exactly where it sat when this was one function in main.c: measuring
//  the deadline before the log line keeps telemetry out of the tick budget, so
//  turning the trace up cannot itself manufacture overruns. It also means
//  ctx.overruns is current when the mode prints it.
//
//  A mode that runs to completion by itself (a single turn, a square, a
//  diagnostic) does its work in Mode_Begin() and never returns -- that is the
//  same shape those modes had when they sat above the loop in main.c.
// ============================================================================

// Everything the control loop has already measured this tick, so a mode never
// re-reads a sensor that has just been read.
typedef struct {
    int16_t  rate;       // bias-corrected yaw rate, raw LSB (+ = turning LEFT)
    int16_t  gx, gy;     // raw X/Y gyro, for the rocking columns in telemetry
    uint32_t now;        // millis() sampled once at the top of this tick
    uint32_t run_start;  // millis() when the control loop began
    uint16_t overruns;   // ticks that overran their deadline, cumulative
} tick_ctx_t;

void Mode_PreGyro(void);
void Mode_Header(void);
void Mode_Begin(void);
void Mode_Tick(const tick_ctx_t *t);
void Mode_Telemetry(const tick_ctx_t *t);

#endif
