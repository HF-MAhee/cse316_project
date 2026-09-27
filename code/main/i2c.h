#ifndef I2C_H
#define I2C_H
#include <stdint.h>

void    I2C_Init(void);
void    I2C_Start(void);
void    I2C_Stop(void);
void    I2C_Write(uint8_t data);
uint8_t I2C_ReadAck(void);
uint8_t I2C_ReadNack(void);

// Error latch. Every wait is bounded by I2C_TIMEOUT_US, and a missing ACK or
// any other unexpected bus status counts too. Once latched, the calls above
// return immediately until I2C_ClearError(), so clear it before a transaction
// and check I2C_Failed() after it.
void    I2C_ClearError(void);
uint8_t I2C_Failed(void);

// Bus clear: clock a stuck slave free, send a STOP, re-initialise the TWI.
void    I2C_Recover(void);

#endif
