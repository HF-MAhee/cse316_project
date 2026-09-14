#ifndef I2C_H
#define I2C_H
#include <stdint.h>

void    I2C_Init(void);
void    I2C_Start(void);
void    I2C_Stop(void);
void    I2C_Write(uint8_t data);
uint8_t I2C_ReadAck(void);
uint8_t I2C_ReadNack(void);

// Read n consecutive registers with BOUNDED waits. Returns 1 on success, 0 if
// the bus stopped responding (and issues a STOP so the bus is not left wedged
// for the next attempt).
//
// The primitives above spin forever on a dead bus -- deliberately, since the
// real fix for that is soldered connections, not a software timeout. But a
// diagnostic whose whole job is to REPORT a dropped connection cannot hang on
// one, so it uses this instead. Nothing on the normal control path does.
uint8_t I2C_ReadRegs(uint8_t dev_write_addr, uint8_t reg, uint8_t *buf, uint8_t n);
#endif
