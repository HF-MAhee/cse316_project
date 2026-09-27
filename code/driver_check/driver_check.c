// ============================================================================
//  L298N DRIVER CHECK -- motors DISCONNECTED, multimeter on DC volts.
//
//  Steps through fixed output states and HOLDS each one until the button is
//  pressed, so every pin can be measured at leisure. Serial (38400) prints
//  what is being driven and what to expect.
//
//  Everything is at 100% duty (OCR = 255, the enable pin held steadily high)
//  except the last step. With no motor connected an L298N output FLOATS during
//  the PWM off-time, so a meter reading of a PWM'd output is meaningless
//  unloaded; a steady state is not. The last step checks only the PWM on the
//  ENABLE pins, where a meter's average is meaningful.
//
//  Same pins, timer setup and baud as the maze firmware (../main/config.h).
// ============================================================================
#include "config.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define UBRR_VAL   ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)

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

// ---- button (PD6, active low, pull-up) ------------------------------------------
static uint8_t btn_down(void) { return (PIND & (1 << BUTTON_BIT)) ? 0 : 1; }
static void wait_press(void) {
    uint8_t n = 0;
    while (n < 3) { n = btn_down() ? (uint8_t)(n + 1) : 0; _delay_ms(10); }
    while (btn_down()) _delay_ms(10);
    _delay_ms(50);
}

// ---- driver outputs ---------------------------------------------------------------
static void hw_init(void) {
    MOTOR_DIR_DDR |= (1 << LEFT_IN1_BIT) | (1 << LEFT_IN2_BIT)
                   | (1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT);
    PWM_DDR       |= (1 << LEFT_PWM_BIT) | (1 << RIGHT_PWM_BIT);
    // Identical to motors.c: 8-bit phase-correct PWM, prescaler 64, ~490 Hz.
    // In this mode OCR = 255 (TOP) holds the pin HIGH continuously.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << CS11) | (1 << CS10);
    OCR1A = 0;
    OCR1B = 0;
}

// dir: 0 = off (both IN low), 1 = forward (IN1/IN3 high), 2 = reverse.
static void chan_a(uint8_t dir, uint8_t pwm) {
    MOTOR_DIR_PORT &= (uint8_t)~((1 << LEFT_IN1_BIT) | (1 << LEFT_IN2_BIT));
    if (dir == 1) MOTOR_DIR_PORT |= (1 << LEFT_IN1_BIT);
    if (dir == 2) MOTOR_DIR_PORT |= (1 << LEFT_IN2_BIT);
    OCR1B = dir ? pwm : 0;          // ENA <- PD4 / OC1B
}
static void chan_b(uint8_t dir, uint8_t pwm) {
    MOTOR_DIR_PORT &= (uint8_t)~((1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT));
    if (dir == 1) MOTOR_DIR_PORT |= (1 << RIGHT_IN3_BIT);
    if (dir == 2) MOTOR_DIR_PORT |= (1 << RIGHT_IN4_BIT);
    OCR1A = dir ? pwm : 0;          // ENB <- PD5 / OC1A
}

static void step(const char *title, const char *expect) {
    P("\r\n----------------------------------------------------------\r\n");
    puts_P_(title);
    P("\r\n");
    puts_P_(expect);
    P("  [press the button for the next step]\r\n");
    wait_press();
}

int main(void) {
    hw_init();
    chan_a(0, 0);
    chan_b(0, 0);
    wdt_disable();
    DDRD  &= (uint8_t)~(1 << BUTTON_BIT);
    PORTD |=  (1 << BUTTON_BIT);
    LED_DDR |= (1 << LED_BIT);
    uart_init();

    for (;;) {
        chan_a(0, 0); chan_b(0, 0);
        LED_PORT |= (1 << LED_BIT);
        P("\r\n\r\n=== L298N DRIVER CHECK (motors DISCONNECTED) ===\r\n");
        P("Meter on DC volts. Black probe on GND unless it says OUTx-OUTy.\r\n");
        P("Motor battery ON. Press the button to begin.\r\n");
        wait_press();
        LED_PORT &= (uint8_t)~(1 << LED_BIT);

        chan_a(0, 0); chan_b(0, 0);
        step(PSTR("STEP 1: everything OFF"),
             PSTR("  ENA ~0 V, ENB ~0 V, IN1-IN4 ~0 V.\r\n"
                  "  L298N 5V pin ~5 V (regulator, jumper ON).\r\n"
                  "  L298N 12V pin = battery (~11.7 V).\r\n"
                  "  OUT1-OUT2 and OUT3-OUT4: ~0 V (floating, may wander).\r\n"));

        chan_a(1, 255); chan_b(0, 0);
        step(PSTR("STEP 2: channel A (LEFT) FORWARD, 100%"),
             PSTR("  ENA ~5 V, IN1 ~5 V, IN2 ~0 V.\r\n"
                  "  OUT1-OUT2 (red OUT1): about +9.5..+11 V.  WRITE IT DOWN.\r\n"
                  "  OUT3-OUT4: ~0 V.\r\n"));

        chan_a(0, 0); chan_b(1, 255);
        step(PSTR("STEP 3: channel B (RIGHT) FORWARD, 100%"),
             PSTR("  ENB ~5 V, IN3 ~5 V, IN4 ~0 V.\r\n"
                  "  OUT3-OUT4 (red OUT3): about +9.5..+11 V -- should match\r\n"
                  "  STEP 2 within ~0.3 V.  OUT1-OUT2: ~0 V.\r\n"));

        chan_a(2, 255); chan_b(0, 0);
        step(PSTR("STEP 4: channel A (LEFT) REVERSE, 100%"),
             PSTR("  ENA ~5 V, IN1 ~0 V, IN2 ~5 V.\r\n"
                  "  OUT1-OUT2 (red OUT1): same size as STEP 2, NEGATIVE.\r\n"));

        chan_a(0, 0); chan_b(2, 255);
        step(PSTR("STEP 5: channel B (RIGHT) REVERSE, 100%"),
             PSTR("  ENB ~5 V, IN3 ~0 V, IN4 ~5 V.\r\n"
                  "  OUT3-OUT4 (red OUT3): same size as STEP 3, NEGATIVE.\r\n"));

        chan_a(1, 255); chan_b(1, 255);
        step(PSTR("STEP 6: BOTH FORWARD, 100%"),
             PSTR("  OUT1-OUT2 and OUT3-OUT4 both as in STEPS 2/3, and equal.\r\n"
                  "  L298N 5V pin still ~5 V.\r\n"));

        chan_a(1, 128); chan_b(1, 128);
        step(PSTR("STEP 7: BOTH FORWARD, 50% PWM -- measure ENA and ENB ONLY"),
             PSTR("  ENA ~2.5 V and ENB ~2.5 V (the meter averages the PWM);\r\n"
                  "  they should match within ~0.1 V. Ignore the OUT pins here:\r\n"
                  "  unloaded, they float during the off-time.\r\n"));

        chan_a(0, 0); chan_b(0, 0);
        P("\r\nDone -- all outputs OFF. Press the button to run it again.\r\n");
    }
}
