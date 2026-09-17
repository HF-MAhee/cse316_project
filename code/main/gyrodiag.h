#ifndef GYRODIAG_H
#define GYRODIAG_H

// ============================================================================
//  Gyro / I2C connection diagnostic (make MODE=gyrodiag).
//
//  Answers one question in plain language: is the MPU6050 actually connected
//  and behaving, or is the link intermittent? Every step prints a verdict, so
//  the log can be read directly rather than decoded.
//
//  Runs BEFORE MPU6050_Init()/Gyro_CalibrateFull() and does its own init, on
//  purpose: calibration is 500 unprotected reads, so on a dead bus it hangs
//  before any diagnostic could speak. This uses bounded reads throughout and
//  reports the fault instead.
//
//  Motors are never driven. Blocking -- it never returns.
// ============================================================================
void GyroDiag_Run(void);

#endif
