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

// Build mode 8 -- gyro and I2C connection diagnostic.
//
// The only mode that uses Mode_PreGyro(). It has to run BEFORE MPU6050_Init()
// and Gyro_CalibrateFull(): calibration is 500 reads on the unprotected I2C
// path, so on a dead bus the firmware hangs there and never reaches a
// diagnostic placed any later. This one runs on a cold bus and does its own
// init.
void Mode_PreGyro(void) { GyroDiag_Run(); }

void Mode_Begin(void) { }
void Mode_Tick(const tick_ctx_t *t) { (void)t; }
// Never actually reached: Mode_PreGyro() above does not return. Kept cheap on
// purpose -- calling Telemetry_Header() here would drag the whole telemetry,
// maze and drive chain into a diagnostic image that cannot use any of it.
void Mode_Header(void) { Telemetry_NoneHeader(); }

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
