#ifndef SOLVER_H
#define SOLVER_H
#include <stdint.h>

// ============================================================================
//  THE TWO-RUN MAZE SOLVER
//
//  Run 1 explores with a strict left-hand rule and logs one byte per decision;
//  the log is collapsed into the shortest route and saved to EEPROM. Run 2
//  replays that route. Both runs are started by the panel button. The memory
//  itself -- record format, collapse, replay, persistence -- lives in
//  wallmem.c; this is the state machine that drives the robot through it.
//
//  main.c calls these in order:
//
//      Solver_Init()     after gyro calibration: load any saved route, print
//                        the banner and the telemetry legend
//      Solver_Begin()    last thing before the control loop starts
//      Solver_Tick()     once per CONTROL_TICK_MS -- behaviour only
//
//  Telemetry_Tick() runs after Solver_Tick() in the same tick, once the
//  overrun deadline has been measured, so turning the trace up cannot itself
//  manufacture overruns.
// ============================================================================

// Everything the control loop has already measured this tick, so nothing
// downstream re-reads a sensor that has just been read.
typedef struct {
    int16_t  rate;       // bias-corrected yaw rate, raw LSB (+ = turning LEFT)
    int16_t  gx, gy;     // raw X/Y gyro, for the rocking columns in telemetry
    uint32_t now;        // millis() sampled once at the top of this tick
    uint32_t run_start;  // millis() when the control loop began
    uint16_t overruns;   // ticks that overran their deadline, cumulative
} tick_ctx_t;

void    Solver_Init(void);
void    Solver_Begin(void);
void    Solver_Tick(const tick_ctx_t *t);

// Current state, for the first telemetry column. Values are the order of the
// state enum in solver.c:
//   0 ARMED    1 STARTUP   2 DRIVING    3 APPROACH  4 LOOK          5 STOPPING
//   6 RECAL    7 DECIDE    8 RECOVER    9 DONE     10 DONE_IDLE    11 FAULT
uint8_t Solver_State(void);

#endif
