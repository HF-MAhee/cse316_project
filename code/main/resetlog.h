#ifndef RESETLOG_H
#define RESETLOG_H
#include <stdint.h>

// ============================================================================
//  RESET FORENSICS -- what happened last time, recovered from .noinit.
//
//  Split out of main.c because it is a self-contained subsystem with its own
//  state: it owns the .noinit boot counter and the previous MCUCSR flags, and
//  nothing else may touch them.
//
//  There is deliberately no "halt after a mid-run reset" here any more. The
//  start button gates every run, so a reset part way through simply comes back
//  up waiting for a press -- the robot cannot drive off from an unknown pose.
// ============================================================================

// Reads MCUCSR, clears it, updates the .noinit boot record, and prints the
// verdict -- including the supply reading recovered from before the reset.
// Call once, early, and before anything that draws current.
void    ResetLog_Report(void);

#endif
