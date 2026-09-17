#ifndef RESETLOG_H
#define RESETLOG_H
#include <stdint.h>

// ============================================================================
//  RESET FORENSICS -- what happened last time, recovered from .noinit.
//
//  Split out of main.c because it is a self-contained subsystem with its own
//  state: it owns the .noinit boot counter, the previous MCUCSR flags, and the
//  in-motion flag, and nothing else may touch them.
// ============================================================================

// Reads MCUCSR, clears it, updates the .noinit boot record, and prints the
// verdict -- including the supply reading recovered from before the reset.
// Call once, early, and before anything that draws current.
void    ResetLog_Report(void);

// 1 when the previous boot died while the robot was MOVING. Its position and
// heading are therefore unknown, so restarting a mode from scratch would drive
// blind; main.c halts instead. See HALT_ON_UNSAFE_RESTART.
uint8_t ResetLog_UnsafeRestart(void);

// Bracket the moving part of a run. Mark_Moving before the first motor command,
// Mark_Idle once the run has finished cleanly -- that pairing is what lets the
// next boot tell an interrupted run from a normal restart.
void    ResetLog_MarkMoving(void);
void    ResetLog_MarkIdle(void);

#endif
