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
#endif
