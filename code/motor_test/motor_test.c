// ============================================================================
//  MOTOR TEST -- standalone image, no solver, no sensors.
//
//  Answers one question: is one side of the drivetrain weaker than the other,
//  and is it the motor or the driver channel?
//
//  Uses the same pins, PWM timer setup and baud rate as the maze firmware
//  (pins from ../main/config.h), but writes OCR1A/OCR1B directly so PWM values
//  under MOTOR_MIN_PWM can be tested too.
//
//  LIFT THE ROBOT so both wheels spin freely, open the serial monitor at
//  38400, press the button. Three tests:
//    1. Start-up PWM: each channel, each direction, ramps up slowly. Press the
//       button the moment the wheel starts turning; the PWM is recorded.
//    2. Each channel alone at 80 / 120 / 160 -- compare speed and sound.
//    3. Both channels together at the same steps -- the weak wheel is the
//       slower one.
//  Then a summary. Press the button again to repeat.
// ============================================================================
#include "config.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define UBRR_VAL   ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)

#define RAMP_FROM      20
#define RAMP_TO        200
#define RAMP_STEP_MS   100   // 1 PWM per step: 20 -> 200 takes 18 s at most
#define HOLD_MS        3000
#define GAP_MS         1000

typedef enum { CH_A = 0, CH_B = 1 } chan_t;       // A = left (OC1B), B = right (OC1A)
typedef enum { FWD = 0, REV = 1 } dir_t;

// ---- serial ----------------------------------------------------------------
static void uart_init(void) {
    UBRRH = (uint8_t)(UBRR_VAL >> 8);
    UBRRL = (uint8_t)UBRR_VAL;
    UCSRB = (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   // 8N1
}
static void putc_(char c) { while (!(UCSRA & (1 << UDRE))); UDR = (uint8_t)c; }
static void puts_P_(const char *s) { char c; while ((c = (char)pgm_read_byte(s++))) putc_(c); }
#define P(s) puts_P_(PSTR(s))
static void putu(uint16_t v) {
    char b[6]; uint8_t i = 0;
    do { b[i++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (i) putc_(b[--i]);
}

// ---- button (PD6, active low, internal pull-up) ------------------------------
static uint8_t btn_down(void) { return (PIND & (1 << BUTTON_BIT)) ? 0 : 1; }
// Debounced: 3 consecutive low samples 10 ms apart.
static uint8_t btn_pressed(void) {
    uint8_t i;
    for (i = 0; i < 3; i++) { if (!btn_down()) return 0; _delay_ms(10); }
    return 1;
}
static void wait_release(void) { while (btn_down()) _delay_ms(10); _delay_ms(50); }
static void wait_press(void)   { while (!btn_pressed()) ; wait_release(); }

// ---- motors -------------------------------------------------------------------
static void motors_init(void) {
    MOTOR_DIR_DDR |= (1 << LEFT_IN1_BIT) | (1 << LEFT_IN2_BIT)
                   | (1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT);
    PWM_DDR       |= (1 << LEFT_PWM_BIT) | (1 << RIGHT_PWM_BIT);
    // Identical to motors.c: 8-bit phase-correct PWM, prescaler 64, ~490 Hz.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << CS11) | (1 << CS10);
    OCR1A = 0;
    OCR1B = 0;
}

static void set_dir(chan_t ch, dir_t d) {
    uint8_t in_a = (ch == CH_A) ? LEFT_IN1_BIT : RIGHT_IN3_BIT;
    uint8_t in_b = (ch == CH_A) ? LEFT_IN2_BIT : RIGHT_IN4_BIT;
    if (d == FWD) { MOTOR_DIR_PORT |=  (1 << in_a); MOTOR_DIR_PORT &= (uint8_t)~(1 << in_b); }
    else          { MOTOR_DIR_PORT &= (uint8_t)~(1 << in_a); MOTOR_DIR_PORT |=  (1 << in_b); }
}
static void set_pwm(chan_t ch, uint8_t pwm) { if (ch == CH_A) OCR1B = pwm; else OCR1A = pwm; }

static void stop_all(void) {
    OCR1A = 0; OCR1B = 0;
    MOTOR_DIR_PORT &= (uint8_t)~((1 << LEFT_IN1_BIT) | (1 << LEFT_IN2_BIT)
                               | (1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT));
}

static void name(chan_t ch, dir_t d) {
    if (ch == CH_A) P("channel A = LEFT  (ENA<-PD4, IN1/IN2<-PC2/PC3, OUT1/OUT2)");
    else            P("channel B = RIGHT (ENB<-PD5, IN3/IN4<-PC4/PC5, OUT3/OUT4)");
    if (d == FWD) P("  FORWARD"); else P("  REVERSE");
}

// ---- test 1: start-up PWM ----------------------------------------------------
// Returns the PWM at which the button was pressed, or 0 if it never was.
static uint8_t ramp(chan_t ch, dir_t d) {
    uint16_t pwm;
    uint16_t t;
    P("\r\n"); name(ch, d);
    P("\r\n  ramping up -- PRESS THE BUTTON the moment the wheel starts turning\r\n  ");
    stop_all();
    set_dir(ch, d);
    for (pwm = RAMP_FROM; pwm <= RAMP_TO; pwm++) {
        set_pwm(ch, (uint8_t)pwm);
        if (pwm % 10 == 0) { putu(pwm); P(" "); }
        for (t = 0; t < RAMP_STEP_MS; t += 10) {
            if (btn_pressed()) {
                stop_all();
                P("\r\n  -> starts turning at PWM "); putu(pwm); P("\r\n");
                wait_release();
                _delay_ms(GAP_MS);
                return (uint8_t)pwm;
            }
            _delay_ms(10);
        }
    }
    stop_all();
    P("\r\n  -> no press up to PWM "); putu(RAMP_TO);
    P(": the wheel never started, or the button was missed\r\n");
    _delay_ms(GAP_MS);
    return 0;
}

// ---- tests 2 and 3: fixed steps ------------------------------------------------
static const uint8_t STEPS[] = { 80, 120, 160 };

static void hold(uint16_t ms) { while (ms >= 10) { _delay_ms(10); ms -= 10; } }

static void single_steps(chan_t ch) {
    uint8_t i;
    P("\r\n"); name(ch, FWD); P("\r\n");
    for (i = 0; i < sizeof STEPS; i++) {
        P("  PWM "); putu(STEPS[i]); P(" for 3 s\r\n");
        stop_all(); set_dir(ch, FWD); set_pwm(ch, STEPS[i]);
        hold(HOLD_MS);
        stop_all(); hold(GAP_MS);
    }
}

static void both_steps(void) {
    uint8_t i;
    P("\r\nBOTH channels FORWARD, same PWM -- the weak wheel is the slower one\r\n");
    for (i = 0; i < sizeof STEPS; i++) {
        P("  both at PWM "); putu(STEPS[i]); P(" for 3 s\r\n");
        stop_all(); set_dir(CH_A, FWD); set_dir(CH_B, FWD);
        set_pwm(CH_A, STEPS[i]); set_pwm(CH_B, STEPS[i]);
        hold(HOLD_MS);
        stop_all(); hold(GAP_MS);
    }
}

static void print_start(const char *label_flash, uint8_t v) {
    puts_P_(label_flash);
    if (v) putu(v); else P("none (did not start / no press)");
    P("\r\n");
}

int main(void) {
    uint8_t a_f, b_f, a_r, b_r;

    // Motors off before anything else, same as the maze firmware.
    motors_init();
    stop_all();
    wdt_disable();
    DDRD  &= (uint8_t)~(1 << BUTTON_BIT);
    PORTD |=  (1 << BUTTON_BIT);           // pull-up
    LED_DDR |= (1 << LED_BIT);
    uart_init();

    for (;;) {
        LED_PORT |= (1 << LED_BIT);
        P("\r\n\r\n=== MOTOR TEST ===\r\n");
        P("LIFT THE ROBOT so both wheels spin freely.\r\n");
        P("Motor battery ON. Press the button to start.\r\n");
        wait_press();
        LED_PORT &= (uint8_t)~(1 << LED_BIT);

        P("\r\n--- TEST 1: start-up PWM (lower = healthier) ---");
        a_f = ramp(CH_A, FWD);
        b_f = ramp(CH_B, FWD);
        a_r = ramp(CH_A, REV);
        b_r = ramp(CH_B, REV);

        P("\r\n--- TEST 2: each channel alone -- compare speed and sound ---");
        single_steps(CH_A);
        single_steps(CH_B);

        P("\r\n--- TEST 3 ---");
        both_steps();

        P("\r\n=== SUMMARY: PWM where each wheel started ===\r\n");
        print_start(PSTR("  A / LEFT  forward : "), a_f);
        print_start(PSTR("  B / RIGHT forward : "), b_f);
        print_start(PSTR("  A / LEFT  reverse : "), a_r);
        print_start(PSTR("  B / RIGHT reverse : "), b_r);
        P("A gap of more than ~10 between A and B = that side is weak.\r\n");
        P("Weak in ONE direction only -> mechanical (binding, rubbing wheel).\r\n");
        P("To tell motor from driver: swap the two motors' wires between\r\n");
        P("OUT1/OUT2 and OUT3/OUT4 and run again. If the weakness moves with\r\n");
        P("the MOTOR it is the motor/gearbox; if it stays on channel A it is\r\n");
        P("the L298N channel, the ENA/IN wires, or PD4/PC2/PC3.\r\n");
    }
}
