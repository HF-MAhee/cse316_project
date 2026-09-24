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
// Same, with dt measured from the previous sample of EITHER kind. Use this
// anywhere the sampling is not on a fixed schedule: the control tick (which
// stretches when something inside it blocks), and the brake and drive-off
// pulses, which used to be blind waits and lost whatever yaw they caused.
void    Heading_AddNow(int16_t raw_z);
int32_t Heading_Raw(void);           // LSB*ms
int32_t Heading_Degrees(void);
int32_t Heading_DegreesTenths(void); // one decimal place, integer-only

// --- maze-grid heading --------------------------------------------------
// A second accumulator that Heading_Reset() does NOT clear. It is zeroed once,
// when a run starts with the robot square in the start cell, and from then on
// it is the robot's heading relative to the maze. The maze is a grid, so the
// only headings the robot should ever settle on are multiples of 90 degrees;
// s_grid is the one it is currently supposed to be on.
//
// This is what the relative accumulator could not give: a turn that ended
// 2 degrees short, or a leg that started with the chassis yawed 5 degrees by
// the drive-off, left a permanent angle that nothing remembered. Every turn
// now aims at the next grid heading (not "90 from wherever I happen to be"),
// and the straight-line controller steers back onto the grid heading.
void    Heading_GridReset(void);           // run start: here is square to the maze
void    Heading_GridStep(int8_t quarters); // +1 = one 90 to the LEFT, -1 = right
void    Heading_GridResync(void);          // the grid is wherever the robot points now
void    Heading_GridAdjust(int32_t raw);   // move the grid target by raw LSB*ms
int16_t Heading_ScaleCorrection(void);     // learned gyro scale trim, parts per 10000
int32_t Heading_GridError(void);           // LSB*ms; + = robot is LEFT of the grid
int32_t Heading_GridErrorTenths(void);

// Bias-corrected instantaneous yaw rate.
int16_t Gyro_Rate(int16_t raw_z);

// --- chassis rocking detection (point #5) -------------------------------
// Feed every full 3-axis sample in. Pitch/roll rate above ROCK_RATE_THRESHOLD
// starts a blanking window during which sonar data is untrustworthy.
void    Motion_Update(const gyro_xyz_t *g);
uint8_t Motion_IsSuspect(void);      // 1 while the blanking window is open
uint8_t Motion_IsStill(void);        // 1 if the chassis is genuinely at rest
#endif
