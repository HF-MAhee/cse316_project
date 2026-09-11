#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Set to 1 to print the achieved angle over USART (9600 baud, PD0=RXD/PD1=TXD).
// You need this ON while tuning TICKS_PER_DEGREE. Set to 0 for the final build.
#define TURN_DEBUG 1

#if TURN_DEBUG
#include <stdlib.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART_Init(void) {
    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)UBRR_VALUE;
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}
void USART_SendString(const char* str) {
    while (*str) { while (!(UCSRA & (1 << UDRE))); UDR = *str++; }
}
void USART_SendInteger(int32_t value) {
    char buffer[12];
    ltoa(value, buffer, 10);
    USART_SendString(buffer);
}
#endif

// ============================================================
//  I2C (TWI) Drivers  -- unchanged from your working code
// ============================================================
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

// ============================================================
//  MPU6050 Driver  -- unchanged from your working code
// ============================================================
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

// ============================================================
//  Motor Setup  -- REV macros added for pivot turning
// ============================================================
#define LEFT_FWD()   do { PORTC |= (1<<PC2); PORTC &= ~(1<<PC3); } while (0)
#define LEFT_REV()   do { PORTC &= ~(1<<PC2); PORTC |= (1<<PC3); } while (0)
#define LEFT_STOP()  do { PORTC &= ~(1<<PC2); PORTC &= ~(1<<PC3); } while (0)
#define RIGHT_FWD()  do { PORTC |= (1<<PC4); PORTC &= ~(1<<PC5); } while (0)
#define RIGHT_REV()  do { PORTC &= ~(1<<PC4); PORTC |= (1<<PC5); } while (0)
#define RIGHT_STOP() do { PORTC &= ~(1<<PC4); PORTC &= ~(1<<PC5); } while (0)

// OCR1B -> PD4 -> OC1B -> LEFT wheel
// OCR1A -> PD5 -> OC1A -> RIGHT wheel

volatile uint32_t left_ticks = 0;
volatile uint32_t right_ticks = 0;
ISR(INT0_vect) { if (!(PIND & (1<<PD2))) left_ticks++; GIFR |= (1<<INTF0); }
ISR(INT1_vect) { if (!(PIND & (1<<PD3))) right_ticks++; GIFR |= (1<<INTF1); }

// ============================================================
//  TURN TUNING CONSTANTS
// ============================================================

// Sample period. One loop = this delay + ~0.5ms of I2C transaction time.
#define TURN_SAMPLE_MS   5

// THE ONE CONSTANT THAT SETS ACCURACY.
// Derivation: gyro is at +/-500dps range, so 65.5 LSB per degree/sec.
// Accumulating raw readings every T seconds gives:
//     degrees = accumulator * T / 65.5
// Rearranged, raw units per degree = 65.5 / T
// With T ~= 0.0055s  ->  65.5 / 0.0055 ~= 11900
// This is a STARTING ESTIMATE. Tune it physically (see notes at bottom).
#define TICKS_PER_DEGREE 7934L // tuned

#define KICK_PWM         120   // breakaway pulse (same value your straight code uses)
#define KICK_SAMPLES     8     // 8 * ~5.5ms ~= 44ms, matches your proven kick length

#define TURN_PWM         85    // slow steady turn speed. Above your 60 floor, with
                               // margin because pivoting skids and needs more torque.
                               // Raise if the car stalls mid-turn.

#define BRAKE_PWM        100   // reverse pulse strength to kill momentum
#define BRAKE_SAMPLES    4     // ~22ms

#define SETTLE_SAMPLES   50    // ~275ms of standing still, still counting coast

#define STOP_MARGIN_DEG  4     // cut the main turn loop this many degrees early,
                               // letting coast carry it the rest of the way
#define DEADBAND_DEG     2     // "close enough" - stop correcting inside this band

#define NUDGE_PWM        140   // correction pulse (must be strong enough to
                               // break static friction from a dead stop)
#define NUDGE_SAMPLES    4     // ~22ms per correction pulse
#define MAX_NUDGES       5     // cap so it can never loop forever

#define TURN_TIMEOUT     1200  // safety: max samples in the main loop (~6.6s).
                               // Prevents a permanent freeze if a motor stalls.

// ============================================================
//  Turn helpers
// ============================================================
typedef enum { TURN_LEFT = 0, TURN_RIGHT = 1 } turn_dir_t;

static int32_t Abs32(int32_t v) { return (v < 0) ? -v : v; }

// Apply a rotation direction to both motors.
static void Set_Pivot(uint8_t clockwise) {
    if (clockwise) { LEFT_FWD();  RIGHT_REV(); }
    else           { LEFT_REV();  RIGHT_FWD(); }
}

static void Motors_Off(void) {
    LEFT_STOP(); RIGHT_STOP();
    OCR1A = 0; OCR1B = 0;
}

// ONE sample: read gyro, remove bias, add into the accumulator, wait.
// Every phase of the turn calls this, so no rotation is ever left uncounted.
static void Turn_Sample(int32_t *accum) {
    int16_t raw = MPU6050_Read_Gyro_Z();
    *accum += (int32_t)raw - gyro_z_offset;
    _delay_ms(TURN_SAMPLE_MS);
}

// Run motors in the given direction for n samples, still counting rotation.
static void Pulse_Tracked(uint8_t clockwise, uint8_t pwm, uint8_t n, int32_t *accum) {
    Set_Pivot(clockwise);
    OCR1A = pwm; OCR1B = pwm;
    for (uint8_t i = 0; i < n; i++) Turn_Sample(accum);
}

// Stand still for n samples, but KEEP counting: the chassis coasts a little
// after power is cut, and that coast is real rotation we must not miss.
static void Settle_Tracked(uint8_t n, int32_t *accum) {
    Motors_Off();
    for (uint8_t i = 0; i < n; i++) Turn_Sample(accum);
}

// ============================================================
//  THE TURN
// ============================================================
void Turn_Degrees(uint8_t degrees, turn_dir_t dir) {
    const uint8_t cw          = (dir == TURN_RIGHT);
    const int32_t target      = (int32_t)degrees * TICKS_PER_DEGREE;
    const int32_t stop_target = target - ((int32_t)STOP_MARGIN_DEG * TICKS_PER_DEGREE);
    const int32_t deadband    = (int32_t)DEADBAND_DEG * TICKS_PER_DEGREE;

    int32_t  accum = 0;
    uint16_t guard = 0;

    // ---- PHASE 1: KICKSTART (tracked) ----
    // Your motors cannot start below PWM ~60, and a pivot skids the tyres
    // sideways, so it needs even more breakaway torque than driving straight.
    // Unlike your straight-line kick (which happens BEFORE counting starts),
    // this one is counted, because with wheels spinning opposite directions
    // the kick itself produces real rotation.
    Pulse_Tracked(cw, KICK_PWM, KICK_SAMPLES, &accum);

    // ---- PHASE 2: SLOW STEADY TURN ----
    // Slow is what buys accuracy: less momentum to coast, and finer angular
    // resolution per sample. Stops STOP_MARGIN_DEG early on purpose.
    Set_Pivot(cw);
    OCR1A = TURN_PWM; OCR1B = TURN_PWM;
    while (Abs32(accum) < stop_target && guard < TURN_TIMEOUT) {
        Turn_Sample(&accum);
        guard++;
    }

    // ---- PHASE 3: ACTIVE BRAKE (tracked) ----
    // A short reverse pulse cancels momentum instead of relying on friction.
    Pulse_Tracked(!cw, BRAKE_PWM, BRAKE_SAMPLES, &accum);

    // ---- PHASE 4: SETTLE (tracked) ----
    Settle_Tracked(SETTLE_SAMPLES, &accum);

    // ---- PHASE 5: CLOSED-LOOP CORRECTION ----
    // Now the car is stopped and we know exactly how far it actually went.
    // If we are outside the deadband, fire a short pulse in whichever
    // direction reduces the error, then re-measure. Capped so it cannot
    // oscillate forever.
    for (uint8_t i = 0; i < MAX_NUDGES; i++) {
        int32_t error = target - Abs32(accum);   // >0 = short, <0 = overshot
        if (Abs32(error) <= deadband) break;

        uint8_t nudge_cw = (error > 0) ? cw : !cw;
        Pulse_Tracked(nudge_cw, NUDGE_PWM, NUDGE_SAMPLES, &accum);
        Settle_Tracked(SETTLE_SAMPLES, &accum);
    }

    Motors_Off();

#if TURN_DEBUG
    // Print the achieved angle in tenths of a degree (e.g. 897 = 89.7 deg),
    // so we never need floating point just to show one decimal place.
    USART_SendString("Turn done. Actual angle (deg x10): ");
    USART_SendInteger((Abs32(accum) * 10L) / TICKS_PER_DEGREE);
    USART_SendString("  raw accum: ");
    USART_SendInteger(Abs32(accum));
    USART_SendString("\r\n");
#endif
}

// ============================================================
//  Your original straight-line driver, kept as a function
// ============================================================
void Drive_Straight(int loops) {
    LEFT_FWD(); RIGHT_FWD();
    OCR1B = 160; OCR1A = 160;
    _delay_ms(40);

    uint8_t MASTER_SPEED = 60;
    OCR1B = MASTER_SPEED;
    uint8_t MIN_PWM = 40;
    uint8_t MAX_PWM = 130;

    int32_t heading_accum = 0;
    int16_t Kp_divider = 2600;
    int16_t Kd_divider = 100;
    int loop_counter = 0;

    while (loop_counter < loops) {
        int16_t raw_z = MPU6050_Read_Gyro_Z();
        int16_t gyro_rate = raw_z - gyro_z_offset;
        heading_accum += gyro_rate;

        int16_t slave_pwm = MASTER_SPEED - (heading_accum / Kp_divider) - (gyro_rate / Kd_divider);
        if (slave_pwm > MAX_PWM) slave_pwm = MAX_PWM;
        if (slave_pwm < MIN_PWM) slave_pwm = MIN_PWM;
        OCR1A = slave_pwm;

        _delay_ms(10);
        loop_counter++;
    }
    Motors_Off();
}

int main(void) {
#if TURN_DEBUG
    USART_Init();
#endif

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

    // --- TURN TEST ---
    // Test the turn ALONE first so you can measure it against a floor mark.
    Turn_Degrees(90, TURN_RIGHT);

    // Once the turn is tuned, chain moves like this:
    // Drive_Straight(400);
    // _delay_ms(300);
    // Turn_Degrees(90, TURN_RIGHT);
    // _delay_ms(300);
    // Drive_Straight(400);

    while (1);
}