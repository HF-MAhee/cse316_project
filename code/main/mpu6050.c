#include "config.h"
#include <avr/io.h>
#include "i2c.h"
#include "timer.h"
#include "mpu6050.h"

#define MPU_WRITE   0xD0
#define MPU_READ    0xD1
#define REG_PWR1    0x6B
#define REG_GYROCFG 0x1B
#define REG_GYRO_X  0x43   // X_H, X_L, Y_H, Y_L, Z_H, Z_L are consecutive
#define REG_GYRO_Z  0x47

void MPU6050_Init(void) {
    Timer_WaitMs(150);                  // let the sensor power up

    I2C_Start();
    I2C_Write(MPU_WRITE);
    I2C_Write(REG_PWR1);
    I2C_Write(0x00);                    // wake from sleep
    I2C_Stop();

    I2C_Start();
    I2C_Write(MPU_WRITE);
    I2C_Write(REG_GYROCFG);
    I2C_Write(0x08);                    // FS_SEL=1 -> +/-500 dps -> 65.5 LSB/dps
    I2C_Stop();                         // this choice is what fixes
                                        // GYRO_LSB_MS_PER_DEGREE in config.h
}

int16_t MPU6050_ReadZ(void) {
    uint8_t hi, lo;
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(REG_GYRO_Z); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_READ);
    hi = I2C_ReadAck();
    lo = I2C_ReadNack();
    I2C_Stop();
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

void MPU6050_ReadAll(gyro_xyz_t *out) {
    uint8_t b[6];
    uint8_t i;

    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(REG_GYRO_X); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_READ);
    for (i = 0; i < 5; i++) b[i] = I2C_ReadAck();
    b[5] = I2C_ReadNack();
    I2C_Stop();

    out->x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    out->y = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    out->z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
}
