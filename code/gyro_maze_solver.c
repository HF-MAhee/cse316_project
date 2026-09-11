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
//  Timer0-based millisecond clock
// ============================================================
// Timer1 is already committed to motor PWM (OC1A/OC1B), so timekeeping
// uses Timer0 instead -- it's otherwise unused on this board.
//
// CTC mode, prescaler 64: 16,000,000 / 64 = 250,000 counts/sec.
// OCR0 = 249 means the timer counts 0..249 (250 counts) before resetting,
// so the compare-match interrupt fires every 250 / 250,000 = 1.000 ms,
// exactly, with no rounding error.
volatile uint32_t millis_count = 0;

void Timer0_Init(void) {
    TCCR0 = (1<<WGM01) | (1<<CS01) | (1<<CS00); // CTC, prescaler 64
    OCR0  = 249;                                 // 1ms per compare match
    TIMSK |= (1<<OCIE0);                         // enable the interrupt
}

ISR(TIMER0_COMP_vect) {
    millis_count++;
}

// millis_count is 32 bits but AVR registers are 8 bits, so a read can be
// interrupted halfway through and return a torn/garbage value. Briefly
// disabling interrupts around the read prevents that; SREG is saved and
// restored rather than blindly calling sei(), so this stays safe to call
// from inside another interrupt-disabled section too.
uint32_t millis(void) {
    uint32_t m;
    uint8_t sreg_backup = SREG;
    cli();
    m = millis_count;
    SREG = sreg_backup;
    return m;
}

// Microsecond resolution, needed for timing the sonar echo pulse.
// TCNT0 counts 0..249 at 250kHz, so each count is exactly 4us.
// The OCF0 check handles the race where the compare match has already
// fired but the ISR hasn't incremented millis_count yet -- without it,
// readings taken right at a millisecond boundary would jump backwards.
uint32_t micros(void) {
    uint32_t m;
    uint8_t  t;
    uint8_t sreg_backup = SREG;
    cli();
    m = millis_count;
    t = TCNT0;
    if ((TIFR & (1<<OCF0)) && (t < 249)) m++;
    SREG = sreg_backup;
    return (m * 1000UL) + ((uint32_t)t * 4UL);
}

// ============================================================
//  HC-SR04 Front Sonar  (PA2 = Trig, PA3 = Echo, per your PDF pin map)
// ============================================================
#define FRONT_TRIG      PA2
#define FRONT_ECHO      PA3

// 25ms of echo window ~= 4.3 metres, far beyond anything useful indoors.
// Keeping this short matters: every failed reading blocks the main loop
// for this long, so a huge timeout would stall the steering PID.
#define SONAR_TIMEOUT_US  25000UL

// Returned when no echo comes back. Deliberately a LARGE value meaning
// "nothing ahead", NOT 0. The reference code in your PDF returns 0 on
// timeout, which would read as "obstacle at 0cm" and make the robot brake
// permanently the moment a sensor glitched or a wall was out of range.
#define SONAR_NO_ECHO   999

void Sonar_Init(void) {
    DDRA |=  (1<<FRONT_TRIG);   // Trig is an output
    DDRA &= ~(1<<FRONT_ECHO);   // Echo is an input
    PORTA &= ~(1<<FRONT_TRIG);  // start low
}

// Returns distance in cm, or SONAR_NO_ECHO if nothing came back.
uint16_t Sonar_Read_Front(void) {
    uint32_t start, echo_start, duration;

    // 10us trigger pulse, per the HC-SR04 datasheet
    PORTA &= ~(1<<FRONT_TRIG);
    _delay_us(2);
    PORTA |=  (1<<FRONT_TRIG);
    _delay_us(10);
    PORTA &= ~(1<<FRONT_TRIG);

    // Wait for the echo line to go high (start of the return pulse)
    start = micros();
    while (!(PINA & (1<<FRONT_ECHO))) {
        if (micros() - start > SONAR_TIMEOUT_US) return SONAR_NO_ECHO;
    }

    // Measure how long it stays high
    echo_start = micros();
    while (PINA & (1<<FRONT_ECHO)) {
        if (micros() - echo_start > SONAR_TIMEOUT_US) return SONAR_NO_ECHO;
    }
    duration = micros() - echo_start;

    // Sound travels ~343 m/s; the pulse makes a round trip, so
    // distance_cm = duration_us / 58
    return (uint16_t)(duration / 58UL);
}

// ============================================================
//  TURN TUNING CONSTANTS
// ============================================================

// Sample period, now enforced by Timer0 rather than _delay_ms().
// This is an EXACT 5.000ms wall-clock spacing: the loop waits until the
// scheduled millisecond, so I2C transaction time no longer pads the period.
#define TURN_SAMPLE_MS   5

// THE ONE CONSTANT THAT SETS ACCURACY.
// Derivation: gyro is at +/-500dps range, so 65.5 LSB per degree/sec.
// Accumulating raw readings every T seconds gives:
//     degrees = accumulator * T / 65.5
// Rearranged, raw units per degree = 65.5 / T
//
// With the timer enforcing an exact T = 0.005s:
//     65.5 / 0.005 = 13100
// This is now a CALCULATED value, not an estimate -- the old 11900 existed
// only to absorb the unknown I2C overhead in the _delay_ms() version.
#define TICKS_PER_DEGREE 13100L

#define KICK_PWM         160   // breakaway pulse (same value your straight code uses)
#define KICK_SAMPLES     8     // 8 * 5ms = exactly 40ms, matching your proven kick

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

// Bundles the two things every turn phase must carry: the running rotation
// total, and the timestamp of the next scheduled sample. Keeping them
// together means the sample clock can't get out of sync between phases.
typedef struct {
    int32_t  accum;
    uint32_t next_ms;
} turn_state_t;

// ONE sample, on an exact schedule.
// Instead of "read, then delay 5ms" (which really means 5ms PLUS however
// long the I2C read took), this waits until the next scheduled millisecond
// and then reads. The period is therefore exactly TURN_SAMPLE_MS regardless
// of I2C jitter -- which is what makes TICKS_PER_DEGREE calculable instead
// of a fudge factor.
static void Turn_Sample(turn_state_t *st) {
    while (millis() < st->next_ms);
    int16_t raw = MPU6050_Read_Gyro_Z();
    st->accum += (int32_t)raw - gyro_z_offset;
    st->next_ms += TURN_SAMPLE_MS;
}

// Run motors in the given direction for n samples, still counting rotation.
static void Pulse_Tracked(uint8_t clockwise, uint8_t pwm, uint8_t n, turn_state_t *st) {
    Set_Pivot(clockwise);
    OCR1A = pwm; OCR1B = pwm;
    for (uint8_t i = 0; i < n; i++) Turn_Sample(st);
}

// Stand still for n samples, but KEEP counting: the chassis coasts a little
// after power is cut, and that coast is real rotation we must not miss.
static void Settle_Tracked(uint8_t n, turn_state_t *st) {
    Motors_Off();
    for (uint8_t i = 0; i < n; i++) Turn_Sample(st);
}

// ============================================================
//  THE TURN
// ============================================================
void Turn_Degrees(uint8_t degrees, turn_dir_t dir) {
    const uint8_t cw          = (dir == TURN_RIGHT);
    const int32_t target      = (int32_t)degrees * TICKS_PER_DEGREE;
    const int32_t stop_target = target - ((int32_t)STOP_MARGIN_DEG * TICKS_PER_DEGREE);
    const int32_t deadband    = (int32_t)DEADBAND_DEG * TICKS_PER_DEGREE;

    turn_state_t st;
    st.accum   = 0;
    st.next_ms = millis() + TURN_SAMPLE_MS;  // start the sample clock now
    uint16_t guard = 0;

    // ---- PHASE 1: KICKSTART (tracked) ----
    // Your motors cannot start below PWM ~60, and a pivot skids the tyres
    // sideways, so it needs even more breakaway torque than driving straight.
    // Unlike your straight-line kick (which happens BEFORE counting starts),
    // this one is counted, because with wheels spinning opposite directions
    // the kick itself produces real rotation.
    Pulse_Tracked(cw, KICK_PWM, KICK_SAMPLES, &st);

    // ---- PHASE 2: SLOW STEADY TURN ----
    // Slow is what buys accuracy: less momentum to coast, and finer angular
    // resolution per sample. Stops STOP_MARGIN_DEG early on purpose.
    Set_Pivot(cw);
    OCR1A = TURN_PWM; OCR1B = TURN_PWM;
    while (Abs32(st.accum) < stop_target && guard < TURN_TIMEOUT) {
        Turn_Sample(&st);
        guard++;
    }

    // ---- PHASE 3: ACTIVE BRAKE (tracked) ----
    // A short reverse pulse cancels momentum instead of relying on friction.
    Pulse_Tracked(!cw, BRAKE_PWM, BRAKE_SAMPLES, &st);

    // ---- PHASE 4: SETTLE (tracked) ----
    Settle_Tracked(SETTLE_SAMPLES, &st);

    // ---- PHASE 5: CLOSED-LOOP CORRECTION ----
    // Now the car is stopped and we know exactly how far it actually went.
    // If we are outside the deadband, fire a short pulse in whichever
    // direction reduces the error, then re-measure. Capped so it cannot
    // oscillate forever.
    for (uint8_t i = 0; i < MAX_NUDGES; i++) {
        int32_t error = target - Abs32(st.accum);   // >0 = short, <0 = overshot
        if (Abs32(error) <= deadband) break;

        uint8_t nudge_cw = (error > 0) ? cw : !cw;
        Pulse_Tracked(nudge_cw, NUDGE_PWM, NUDGE_SAMPLES, &st);
        Settle_Tracked(SETTLE_SAMPLES, &st);
    }

    Motors_Off();

#if TURN_DEBUG
    // Print the achieved angle in tenths of a degree (e.g. 897 = 89.7 deg),
    // so we never need floating point just to show one decimal place.
    USART_SendString("Turn done. Actual angle (deg x10): ");
    USART_SendInteger((Abs32(st.accum) * 10L) / TICKS_PER_DEGREE);
    USART_SendString("  raw accum: ");
    USART_SendInteger(Abs32(st.accum));
    USART_SendString("\r\n");
#endif
}

// ============================================================
//  Your original straight-line driver -- now timed off Timer0
//  instead of counting loop iterations
// ============================================================
#define STRAIGHT_SAMPLE_MS 10   // same spacing your original _delay_ms(10) gave

// --- Obstacle detection settings ---
#define OBSTACLE_CM        15   // stop when the wall is closer than this
#define SONAR_EVERY_N       5   // ping every 5th loop (~50ms), not every loop:
                                // a sonar read can block for milliseconds and
                                // would disturb the steering PID's timing
#define SONAR_CONFIRM       2   // require this many CONSECUTIVE close readings
                                // before braking. Ultrasonic sensors throw
                                // occasional bogus short readings; one stray
                                // ping shouldn't stop the robot dead.

// Return values
#define STRAIGHT_TIMEOUT    0   // ran the full duration without seeing anything
#define STRAIGHT_OBSTACLE   1   // stopped early because of a wall ahead

uint8_t Drive_Straight(uint32_t duration_ms) {
    LEFT_FWD(); RIGHT_FWD();
    OCR1B = 160; OCR1A = 160;
    _delay_ms(40); // brief untracked kickstart, same as before -- fine as a
                    // blocking delay since it's short and doesn't need timing

    uint8_t MASTER_SPEED = 60;
    OCR1B = MASTER_SPEED;
    uint8_t MIN_PWM = 40;
    uint8_t MAX_PWM = 130;

    int32_t heading_accum = 0;
    int16_t Kp_divider = 2600;
    int16_t Kd_divider = 100;

    uint32_t start_time  = millis();
    uint32_t next_sample = start_time + STRAIGHT_SAMPLE_MS;

    uint8_t sonar_countdown = SONAR_EVERY_N;
    uint8_t close_hits      = 0;
    uint8_t result          = STRAIGHT_TIMEOUT;

    while (millis() - start_time < duration_ms) {
        // Wait for the next scheduled sample instant rather than adding a
        // flat 10ms after each read. This self-corrects: if the I2C read
        // took 0.6ms, we only wait 9.4ms, so samples stay exactly
        // STRAIGHT_SAMPLE_MS apart on average instead of drifting later
        // and later, and the loop as a whole exits at the real requested
        // duration_ms rather than duration_ms plus accumulated I2C overhead.
        while (millis() < next_sample);

        int16_t raw_z = MPU6050_Read_Gyro_Z();
        int16_t gyro_rate = raw_z - gyro_z_offset;
        heading_accum += gyro_rate;

        int16_t slave_pwm = MASTER_SPEED - (heading_accum / Kp_divider) - (gyro_rate / Kd_divider);
        if (slave_pwm > MAX_PWM) slave_pwm = MAX_PWM;
        if (slave_pwm < MIN_PWM) slave_pwm = MIN_PWM;
        OCR1A = slave_pwm;

        next_sample += STRAIGHT_SAMPLE_MS;

        // --- Periodic obstacle check ---
        if (--sonar_countdown == 0) {
            sonar_countdown = SONAR_EVERY_N;

            uint16_t d = Sonar_Read_Front();
            // SONAR_NO_ECHO is a big number, so it correctly fails this test
            if (d < OBSTACLE_CM) {
                if (++close_hits >= SONAR_CONFIRM) {
                    result = STRAIGHT_OBSTACLE;
                    break;
                }
            } else {
                close_hits = 0; // must be consecutive
            }

            // A sonar read can overrun the next scheduled sample. If we've
            // already missed the deadline, re-base the clock instead of
            // letting the loop try to "catch up" by firing several samples
            // back-to-back with no delay -- that would corrupt the PID's
            // assumption of evenly spaced readings.
            if (millis() > next_sample) next_sample = millis() + STRAIGHT_SAMPLE_MS;
        }
    }
    Motors_Off();
    return result;
}

int main(void) {
#if TURN_DEBUG
    USART_Init();
#endif

    I2C_Init();
    MPU6050_Init();
    Timer0_Init(); // must run before sei() below so millis() starts counting
    Sonar_Init();

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

    // ========================================================
    //  SIMPLE MAZE SOLVER
    //  Drive forward -> wall ahead -> stop -> turn right -> repeat
    // ========================================================
    //
    // TUNE THE TURN FIRST. Set MAZE_MODE to 0, measure a single 90 turn
    // against a floor mark, fix TICKS_PER_DEGREE, and only then set this
    // back to 1. Turn errors compound: a 5-degree error per corner leaves
    // the robot 20 degrees crooked after four turns.
    #define MAZE_MODE 1

#if MAZE_MODE
    // Longest a single straight leg may run before giving up and turning
    // anyway. Acts as a safety net if the sonar misses a wall entirely
    // (soft/angled surfaces reflect sound away and can read as "clear").
    #define MAX_LEG_MS 8000UL

    while (1) {
        uint8_t why = Drive_Straight(MAX_LEG_MS);

        Motors_Off();
        _delay_ms(300);   // let the chassis fully settle before pivoting,
                          // so leftover forward momentum doesn't corrupt
                          // the turn's very first gyro samples

#if TURN_DEBUG
        USART_SendString(why == STRAIGHT_OBSTACLE ? "Wall ahead -> turning\r\n"
                                                  : "Leg timeout -> turning\r\n");
#endif
        (void)why;  // both cases turn right in this simple version

        Turn_Degrees(90, TURN_RIGHT);
        _delay_ms(300);
    }
#else
    // --- TURN TEST ---
    // Test the turn ALONE so you can measure it against a floor mark.
    Turn_Degrees(90, TURN_RIGHT);
#endif

    while (1);
}
