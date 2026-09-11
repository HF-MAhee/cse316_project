#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// --- I2C (TWI) Drivers ---
void I2C_Init(void) {
    TWSR = 0x00; // Prescaler = 1
    TWBR = 72;   // 100kHz I2C clock speed at 16MHz CPU
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

// --- MPU6050 Driver ---
#define MPU_WRITE 0xD0
#define MPU_READ  0xD1
int32_t gyro_z_offset = 0;

void MPU6050_Init(void) {
    _delay_ms(150); 
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(0x6B); I2C_Write(0x00); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(0x1B); I2C_Write(0x08); I2C_Stop();
}

int16_t MPU6050_Read_Gyro_Z(void) {
    uint8_t z_high, z_low;
    I2C_Start(); I2C_Write(MPU_WRITE); I2C_Write(0x47); I2C_Stop();
    I2C_Start(); I2C_Write(MPU_READ);
    z_high = I2C_Read_Ack();
    z_low = I2C_Read_Nack();
    I2C_Stop();
    return (int16_t)((z_high << 8) | z_low);
}

void Calibrate_Gyro(void) {
    int32_t sum = 0;
    for (int i = 0; i < 500; i++) {
        sum += MPU6050_Read_Gyro_Z();
        _delay_ms(2); 
    }
    gyro_z_offset = sum / 500;
}

// --- Motor & Encoder Setup ---
#define LEFT_FWD()   do { PORTC |= (1<<PC2); PORTC &= ~(1<<PC3); } while (0)
#define LEFT_STOP()  do { PORTC &= ~(1<<PC2); PORTC &= ~(1<<PC3); } while (0)
#define RIGHT_FWD()  do { PORTC |= (1<<PC4); PORTC &= ~(1<<PC5); } while (0)
#define RIGHT_STOP() do { PORTC &= ~(1<<PC4); PORTC &= ~(1<<PC5); } while (0)

volatile uint32_t left_ticks = 0;
volatile uint32_t right_ticks = 0;

ISR(INT0_vect) { if (!(PIND & (1<<PD2))) left_ticks++; GIFR |= (1<<INTF0); }
ISR(INT1_vect) { if (!(PIND & (1<<PD3))) right_ticks++; GIFR |= (1<<INTF1); }

int main(void) {
    // 1. Init Hardware
    I2C_Init();
    MPU6050_Init();
    
    DDRC |= (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5);
    DDRD |= (1<<PD4) | (1<<PD5);
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
    TCCR1B = (1<<CS11) | (1<<CS10);
    
    DDRD &= ~((1<<PD2) | (1<<PD3)); 
    PORTD |= (1<<PD2) | (1<<PD3);   
    MCUCR |= (1<<ISC01) | (1<<ISC11);
    MCUCR &= ~((1<<ISC00) | (1<<ISC10));
    GICR |= (1<<INT0) | (1<<INT1); 
    sei();

    Calibrate_Gyro();
    gyro_z_offset += 3;
    _delay_ms(2000);

    // 2. The Aggressive Kick-Start
    LEFT_FWD(); RIGHT_FWD();
    OCR1B = 160; OCR1A = 160; 
    _delay_ms(40); 
    cli(); left_ticks = 0; right_ticks = 0; sei();

    // 3. Fusion Variables
    uint8_t MASTER_SPEED = 60;
    OCR1B = MASTER_SPEED; 
    
    uint8_t MIN_PWM = 40; 
    uint8_t MAX_PWM = 130;

    // The giant integer memory bank tracking our heading
    int32_t heading_accum = 0; 
    
    // Instead of a float Kp = 2.5, we use a large divider for integers.
    // Decrease this number to make the steering MORE aggressive.
    // Increase this number to make the steering LESS aggressive.
    int16_t Kp_divider = 2600; 
    int16_t Kd_divider = 100;

    int loop_counter = 0;

    // Run for roughly 5 seconds
    while (loop_counter < 400) {
        
        // A. Read Gyroscope
        int16_t raw_z = MPU6050_Read_Gyro_Z();
        int16_t gyro_rate = raw_z - gyro_z_offset; 
        
        // B. INTEGRATE: Add raw rotational data directly into memory
        heading_accum += gyro_rate;

        // C. STEERING MATH
        // Divide our massive raw number down into a usable PWM adjustment
        // int16_t slave_pwm = MASTER_SPEED - (heading_accum / Kp_divider);
        int16_t slave_pwm = MASTER_SPEED - (heading_accum / Kp_divider) - (gyro_rate / Kd_divider);

        // D. Constrain and Apply
        if (slave_pwm > MAX_PWM) slave_pwm = MAX_PWM;
        if (slave_pwm < MIN_PWM) slave_pwm = MIN_PWM;
        
        OCR1A = slave_pwm;

        _delay_ms(10);
        loop_counter++;
    }

    LEFT_STOP(); RIGHT_STOP();
    OCR1A = 0; OCR1B = 0;
    while(1);
}