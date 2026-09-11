#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// --- USART Definitions ---
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init(void) {
    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)UBRR_VALUE;
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void USART_SendString(const char* str) {
    while (*str) {
        while (!(UCSRA & (1 << UDRE)));
        UDR = *str++;
    }
}

void USART_SendInteger(int32_t value) {
    char buffer[12];
    itoa(value, buffer, 10);
    USART_SendString(buffer);
}

// --- I2C (TWI) Functions ---
void I2C_Init(void) {
    TWSR = 0x00; // Prescaler = 1
    TWBR = 72;   // 100kHz I2C clock speed at 16MHz CPU
    TWCR = (1 << TWEN); // Enable TWI
}

void I2C_Start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Wait for start to transmit
}

void I2C_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Wait for data to transmit
}

uint8_t I2C_Read_Ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_Read_Nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// --- MPU6050 Setup ---
#define MPU_WRITE 0xD0
#define MPU_READ  0xD1

void MPU6050_Init(void) {
    _delay_ms(150); // Let sensor power up
    
    // 1. Wake up the sensor (Write 0x00 to register 0x6B)
    I2C_Start();
    I2C_Write(MPU_WRITE);
    I2C_Write(0x6B); // Power Management 1
    I2C_Write(0x00); // Wake up
    I2C_Stop();
    
    // 2. Set Gyro Range to +/- 500 degrees/sec (Write 0x08 to register 0x1B)
    I2C_Start();
    I2C_Write(MPU_WRITE);
    I2C_Write(0x1B); // Gyro Configuration
    I2C_Write(0x08); 
    I2C_Stop();
}

int16_t MPU6050_Read_Gyro_Z(void) {
    uint8_t z_high, z_low;
    
    // Point to the Gyro Z High Byte register (0x47)
    I2C_Start();
    I2C_Write(MPU_WRITE);
    I2C_Write(0x47);
    I2C_Stop();
    
    // Request 2 bytes (High and Low)
    I2C_Start();
    I2C_Write(MPU_READ);
    z_high = I2C_Read_Ack();
    z_low = I2C_Read_Nack();
    I2C_Stop();
    
    // Combine the two 8-bit registers into one 16-bit integer
    return (int16_t)((z_high << 8) | z_low);
}

int32_t gyro_z_offset = 0;

void Calibrate_Gyro(void) {
    USART_SendString("Calibrating Gyro... DO NOT MOVE CHASSIS!\r\n");
    
    int32_t sum = 0;
    
    // Take 500 readings very quickly
    for (int i = 0; i < 500; i++) {
        sum += MPU6050_Read_Gyro_Z();
        _delay_ms(2); 
    }
    
    // Find the average baseline noise
    gyro_z_offset = sum / 500;
    
    USART_SendString("Calibration Complete! Offset is: ");
    USART_SendInteger(gyro_z_offset);
    USART_SendString("\r\n\r\n");
}

int main(void) {
    USART_Init();
    USART_SendString("\r\nInitializing I2C and MPU6050...\r\n");
    
    I2C_Init();
    MPU6050_Init();
    
    // 1. Run the calibration BEFORE moving
    Calibrate_Gyro();

    while (1) {
        // 2. Read the raw data
        int16_t raw_z = MPU6050_Read_Gyro_Z();
        
        // 3. Subtract the offset to get the true reading
        int16_t corrected_z = raw_z - gyro_z_offset;
        
        // 4. (Optional) Convert raw data to actual Degrees Per Second
        // At +/- 500 deg/sec range, the MPU6050 scale factor is 65.5
        int16_t degrees_per_second = corrected_z / 65; 
        
        USART_SendString("Corrected Z: ");
        USART_SendInteger(corrected_z);
        USART_SendString(" | Deg/Sec: ");
        USART_SendInteger(degrees_per_second);
        USART_SendString("\r\n");
        
        _delay_ms(100); 
    }

    return 0;
}