#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>

typedef struct {
    int16_t x;   // pitch rate  (raw LSB)
    int16_t y;   // roll rate   (raw LSB)
    int16_t z;   // yaw rate    (raw LSB) -- the axis we steer on
} gyro_xyz_t;

void MPU6050_Init(void);

// Yaw only. Cheapest read (2 bytes) -- used inside the tight turn loop.
int16_t MPU6050_ReadZ(void);

// All three axes in ONE burst transaction (6 bytes). Barely more expensive
// than reading Z alone, and X/Y are what reveal chassis pitch/roll rocking.
void MPU6050_ReadAll(gyro_xyz_t *out);

// Both reads above return the last good sample if the bus fails, and clear
// the bus / re-wake the sensor while it keeps failing. This turns 0 once the
// failure has lasted GYRO_FAIL_MS -- the gyro is gone, not glitching.
uint8_t  MPU6050_Healthy(void);
uint16_t MPU6050_FailCount(void);   // failed transactions since boot
#endif
