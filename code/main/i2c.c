#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"

// Every wait here is BOUNDED. They used to spin until TWINT, forever: when the
// MPU6050 dropped off the bus (a loose wire, a jolt, its supply blinking) the
// CPU parked in one of those loops until the 250 ms watchdog rebooted it --
// mid-run, run lost. Logs usart_20260927_213259/213408: two watchdog resets
// with the rail healthy at 5.0-5.1 V, one of them with the motors OFF and the
// last gyro row reading x = y = 0 exactly.
//
// Now a wait gives up after I2C_TIMEOUT_US, and so does an unexpected TWI
// status (no ACK, lost arbitration). Either one latches s_err, and every later
// call in the same transaction returns at once -- so a dead bus costs at most
// one timeout per transaction, not one per byte. The caller checks
// I2C_Failed() once at the end and decides what to do (see mpu6050.c).
//
// This does NOT fix a loose wire. It turns "the chip reboots" into "a few
// milliseconds without a gyro reading", and lets the solver stop the robot
// cleanly if the gyro stays gone.

#define SCL_BIT PC0   // hardware TWI pins on the ATmega32 -- see config.h
#define SDA_BIT PC1

// TWSR status codes (prescaler bits masked off) that mean "that worked".
#define TW_START_OK     0x08
#define TW_REP_START_OK 0x10
#define TW_SLA_W_ACK    0x18
#define TW_DATA_W_ACK   0x28
#define TW_SLA_R_ACK    0x40
#define TW_DATA_R_ACK   0x50
#define TW_DATA_R_NACK  0x58

static uint8_t s_err = 0;

static uint8_t status(void) { return (uint8_t)(TWSR & 0xF8); }

// 1 once TWINT is set, 0 (and s_err latched) if it never comes.
static uint8_t wait_twint(void) {
    uint16_t n = I2C_TIMEOUT_US;
    while (!(TWCR & (1 << TWINT))) {
        if (--n == 0) { s_err = 1; return 0; }
        _delay_us(1);
    }
    return 1;
}

void I2C_Init(void) {
    TWSR = 0x00;          // prescaler 1
    TWBR = 72;            // 100 kHz at 16 MHz
    TWCR = (1 << TWEN);
}

void    I2C_ClearError(void) { s_err = 0; }
uint8_t I2C_Failed(void)     { return s_err; }

void I2C_Start(void) {
    uint8_t st;
    if (s_err) return;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!wait_twint()) return;
    st = status();
    if (st != TW_START_OK && st != TW_REP_START_OK) s_err = 1;
}

void I2C_Stop(void) {
    // Always issued, even after an error: it is what releases the bus. It does
    // not wait, so it cannot hang.
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_Write(uint8_t data) {
    uint8_t st;
    if (s_err) return;
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!wait_twint()) return;
    st = status();
    // One routine sends both addresses and data, so accept any of the three
    // ACKed outcomes; every NACK or arbitration-lost code is a failure.
    if (st != TW_SLA_W_ACK && st != TW_DATA_W_ACK && st != TW_SLA_R_ACK) s_err = 1;
}

uint8_t I2C_ReadAck(void) {
    if (s_err) return 0;
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    if (!wait_twint()) return 0;
    if (status() != TW_DATA_R_ACK) s_err = 1;
    return TWDR;
}

uint8_t I2C_ReadNack(void) {
    if (s_err) return 0;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!wait_twint()) return 0;
    if (status() != TW_DATA_R_NACK) s_err = 1;
    return TWDR;
}

// Standard I2C bus clear. A slave that lost power or saw a glitch part way
// through a read can be left holding SDA low, waiting for clocks that never
// come; the TWI hardware then cannot even send a START. Up to nine clocks let
// it finish shifting out whatever byte it thinks it is in, then a STOP resets
// its state machine.
//
// Open-drain by hand: PORT bits stay 0, so setting a DDR bit pulls the line
// low and clearing it lets the module's pull-up raise it. PORTC's other bits
// are the motor direction pins and are not touched.
void I2C_Recover(void) {
    uint8_t i;
    TWCR = 0;                                       // TWI off: pins revert to PORTC/DDRC
    PORTC &= (uint8_t)~((1 << SCL_BIT) | (1 << SDA_BIT));
    DDRC  &= (uint8_t)~((1 << SCL_BIT) | (1 << SDA_BIT));
    _delay_us(5);
    for (i = 0; i < 9; i++) {
        if (PINC & (1 << SDA_BIT)) break;           // slave has let go
        DDRC |=  (1 << SCL_BIT); _delay_us(5);      // SCL low
        DDRC &= (uint8_t)~(1 << SCL_BIT); _delay_us(5);  // SCL high
    }
    // STOP: SDA rises while SCL is high.
    DDRC |=  (1 << SCL_BIT); _delay_us(5);
    DDRC |=  (1 << SDA_BIT); _delay_us(5);
    DDRC &= (uint8_t)~(1 << SCL_BIT); _delay_us(5);
    DDRC &= (uint8_t)~(1 << SDA_BIT); _delay_us(5);
    I2C_Init();
    s_err = 0;
}
