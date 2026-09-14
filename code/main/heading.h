#ifndef HEADING_H
#define HEADING_H
#include <stdint.h>
#include "mpu6050.h"

// ============================================================================
//  Heading tracking and gyro bias management.
//
//  Integration is done in LSB*MILLISECONDS, not raw LSB:
//        accum += rate_lsb * dt_ms
//        degrees = accum / GYRO_LSB_MS_PER_DEGREE
//
//  Why: the old scheme accumulated bare LSB and divided by a constant that
//  silently encoded the loop period, so changing the sample rate broke the
//  calibration. Carrying dt explicitly makes the constant depend only on the
//  datasheet sensitivity, so the 5 ms turn loop and the 20 ms drive loop can
//  share one constant.
// ============================================================================

// --- bias / calibration ---
// Long calibration, power-up. Returns 1 if the chassis held still long enough
// for the bias to be VALIDATED, 0 if every attempt was rejected and an
// unchecked average was used instead.
//
// A 0 is serious and must be reported, not ignored: it means the bias was
// measured while the robot was moving, so the heading zero is wrong and every
// turn and straight-line correction for the rest of the run inherits that
// error. The usual cause is starting up on a chassis that is still coasting --
// e.g. immediately after a brown-out reset mid-maneuver.
uint8_t Gyro_CalibrateFull(void);
uint8_t Gyro_CalibrateQuick(void);   // short re-calibration between moves.
                                     // Returns 1 on success, 0 if the chassis
                                     // would not hold still (offset unchanged).
int32_t Gyro_GetOffset(void);
int32_t Gyro_GetOffsetX(void);       // X/Y bias, used only by Motion_Update()
int32_t Gyro_GetOffsetY(void);       // (Z is the one heading integration uses)

// --- heading accumulator ---
void    Heading_Reset(void);
void    Heading_Add(int16_t raw_z, uint16_t dt_ms);
int32_t Heading_Raw(void);           // LSB*ms
int32_t Heading_Degrees(void);
int32_t Heading_DegreesTenths(void); // one decimal place, integer-only

// Bias-corrected instantaneous yaw rate.
int16_t Gyro_Rate(int16_t raw_z);

// --- chassis rocking detection (point #5) -------------------------------
// Feed every full 3-axis sample in. Pitch/roll rate above ROCK_RATE_THRESHOLD
// starts a blanking window during which sonar data is untrustworthy.
void    Motion_Update(const gyro_xyz_t *g);
uint8_t Motion_IsSuspect(void);      // 1 while the blanking window is open
uint8_t Motion_IsStill(void);        // 1 if the chassis is genuinely at rest
#endif
