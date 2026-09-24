#include "config.h"
#include <avr/io.h>
#include "i2c.h"

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
