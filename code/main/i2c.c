#include "config.h"
#include <avr/io.h>
#include "i2c.h"
#include "timer.h"

// Unchanged from the working straight-line firmware.
//
// NOTE: every wait loop here is unbounded. If the MPU6050 loses power
// mid-transaction the CPU parks in one of these forever. That is the
// freeze mode seen during the earlier power-rail investigation; the real
// fix is soldered connections, not a software timeout.

void I2C_Init(void) {
    TWSR = 0x00;          // prescaler 1
    TWBR = 72;            // 100 kHz at 16 MHz
    TWCR = (1 << TWEN);
}

void I2C_Start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_ReadAck(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_ReadNack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// ---------------------------------------------------------------------------
//  Bounded-wait register read, for the gyro diagnostic only.
// ---------------------------------------------------------------------------
static uint8_t wait_twint(void) {
    uint32_t t0 = micros();
    while (!(TWCR & (1 << TWINT))) {
        if ((micros() - t0) > I2C_TIMEOUT_US) return 0;
    }
    return 1;
}

static uint8_t bus_release(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    return 0;
}

uint8_t I2C_ReadRegs(uint8_t dev_write_addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    uint8_t i;

    // Same sequence MPU6050_ReadAll() uses (STOP then START rather than a
    // repeated START) -- that is what is proven to work on this board.
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!wait_twint()) return bus_release();
    TWDR = dev_write_addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!wait_twint()) return bus_release();
    TWDR = reg;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!wait_twint()) return bus_release();
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!wait_twint()) return bus_release();
    TWDR = (uint8_t)(dev_write_addr | 1);          // SLA+R
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!wait_twint()) return bus_release();

    for (i = 0; i < n; i++) {
        if ((uint8_t)(i + 1) < n) TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
        else                      TWCR = (1 << TWINT) | (1 << TWEN);
        if (!wait_twint()) return bus_release();
        buf[i] = TWDR;
    }

    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    return 1;
}
