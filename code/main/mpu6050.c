#include "config.h"
#include <avr/io.h>
#include "i2c.h"
#include "timer.h"
#include "debug.h"
#include "mpu6050.h"

#define MPU_WRITE   0xD0
#define MPU_READ    0xD1
#define REG_PWR1    0x6B
#define REG_GYROCFG 0x1B
#define REG_GYRO_X  0x43   // X_H, X_L, Y_H, Y_L, Z_H, Z_L are consecutive
#define REG_GYRO_Z  0x47

// ---- failure handling -------------------------------------------------------
// A failed read (see i2c.c: bounded waits, NACKs) hands back the LAST GOOD
// sample instead of garbage, so one glitch costs one repeated reading. While
// reads keep failing, the bus is cleared and the sensor re-woken at most every
// GYRO_RECOVER_INTERVAL_MS -- a sensor that browned out comes back asleep, and
// a slave stuck mid-byte holds SDA low until it is clocked free.
//
// MPU6050_Healthy() turns 0 once reads have failed continuously for
// GYRO_FAIL_MS. The solver stops the robot then: with no gyro, the heading --
// every turn and all straight-line damping -- is fiction.
static gyro_xyz_t s_last;
static uint8_t    s_failing      = 0;
static uint32_t   s_fail_since   = 0;
static uint32_t   s_last_recover = 0;
static uint16_t   s_fail_count   = 0;

static void configure(void) {
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

void MPU6050_Init(void) {
    Timer_WaitMs(150);                  // let the sensor power up
    I2C_ClearError();
    configure();
    if (I2C_Failed()) {
        Debug_P("*** MPU6050 NOT RESPONDING on I2C -- check its VCC, GND,"
                " SCL (PC0) and SDA (PC1) wires\r\n");
        Debug_Flush();
        I2C_Recover();
    }
}

// Called after every transaction.
static uint8_t check(void) {
    uint32_t now;
    if (!I2C_Failed()) {
        if (s_failing) {
            s_failing = 0;
            Debug_P("gyro back after ");
            Debug_Int((int32_t)(millis() - s_fail_since));
            Debug_P(" ms\r\n");
        }
        return 1;
    }

    now = millis();
    if (s_fail_count < 0xFFFF) s_fail_count++;
    if (!s_failing) {
        s_failing    = 1;
        s_fail_since = now;
        s_last_recover = now - GYRO_RECOVER_INTERVAL_MS;   // recover right away
        Debug_P("*** gyro I2C error -- holding last reading, clearing bus\r\n");
    }
    if ((uint32_t)(now - s_last_recover) >= GYRO_RECOVER_INTERVAL_MS) {
        s_last_recover = now;
        I2C_Recover();
        configure();                    // if it browned out, it woke up asleep
    }
    I2C_ClearError();
    return 0;
}

uint8_t MPU6050_Healthy(void) {
    return (uint8_t)!(s_failing && (uint32_t)(millis() - s_fail_since) >= GYRO_FAIL_MS);
}

uint16_t MPU6050_FailCount(void) { return s_fail_count; }

int16_t MPU6050_ReadZ(void) {
    uint8_t hi, lo;
    I2C_ClearError();
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(REG_GYRO_Z); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_READ);
    hi = I2C_ReadAck();
    lo = I2C_ReadNack();
    I2C_Stop();
    if (check()) s_last.z = (int16_t)(((uint16_t)hi << 8) | lo);
    return s_last.z;
}

void MPU6050_ReadAll(gyro_xyz_t *out) {
    uint8_t b[6];
    uint8_t i;

    I2C_ClearError();
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(REG_GYRO_X); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_READ);
    for (i = 0; i < 5; i++) b[i] = I2C_ReadAck();
    b[5] = I2C_ReadNack();
    I2C_Stop();

    if (check()) {
        s_last.x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
        s_last.y = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
        s_last.z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
    }
    *out = s_last;
}
