#include "config.h"
#include "panel.h"

#ifdef __AVR__
#  include <avr/io.h>
#  include "timer.h"
static void     pins_init(void) {
    // The LED and the button are on different ports, so these are four separate
    // read-modify-writes rather than two. Both ports already carry other
    // peripherals -- PORTD has the USART and the motor PWM -- so every one of
    // these has to be |= or &=, never a plain assignment.
    LED_DDR     |=  (1 << LED_BIT);          // LED is an output...
    LED_PORT    &= ~(1 << LED_BIT);          // ...and starts off
    BUTTON_DDR  &= ~(1 << BUTTON_BIT);       // button is an input...
    BUTTON_PORT |=  (1 << BUTTON_BIT);       // ...with the internal pull-up on
}
static uint8_t  btn_level(void) { return (uint8_t)((BUTTON_PIN & (1 << BUTTON_BIT)) ? 1u : 0u); }
static void     led_write(uint8_t on) {
    if (on) LED_PORT |=  (1 << LED_BIT);
    else    LED_PORT &= ~(1 << LED_BIT);
}
static uint32_t now_ms(void) { return millis(); }
#else
/* Host build: the debounce and the short/long split are exactly the kind of
   logic that is wrong by one tick and costs a lab session to find, so they are
   testable off-target. Only these three shims change; everything below is the
   same code that runs on the robot. */
uint8_t  Panel_TestLevel = 1;    /* 1 = released, matching the pull-up */
uint8_t  Panel_TestLed   = 0;
uint32_t Panel_TestMs    = 0;
static void     pins_init(void) { }
static uint8_t  btn_level(void) { return Panel_TestLevel; }
static void     led_write(uint8_t on) { Panel_TestLed = on; }
static uint32_t now_ms(void) { return Panel_TestMs; }
#endif

static led_mode_t s_mode    = LED_OFF;
static uint8_t    s_raw     = 1;   // last sample, 1 = released
static uint8_t    s_stable  = 1;   // debounced level
static uint8_t    s_count   = 0;   // consecutive samples agreeing with s_raw
static uint8_t    s_primed  = 0;   // have we ever seen a settled RELEASED state?
static uint8_t    s_event   = 0;   // latched short press, cleared when read
static uint8_t    s_long    = 0;   // latched long press, cleared when read
static uint32_t   s_down_at = 0;   // when the debounced level went low

void Panel_Init(void) {
    pins_init();
    s_mode    = LED_OFF;
    s_raw     = 1;
    s_stable  = 1;
    s_count   = 0;
    s_primed  = 0;
    s_event   = 0;
    s_long    = 0;
    s_down_at = 0;
    led_write(0);
}

void Panel_SetLed(led_mode_t m) {
    s_mode = m;
    if (m == LED_OFF)     led_write(0);
    else if (m == LED_ON) led_write(1);
    /* the two blink modes are driven from Panel_Task() */
}

led_mode_t Panel_Led(void)      { return s_mode; }
uint8_t Panel_ButtonDown(void)  { return (uint8_t)(s_stable ? 0u : 1u); }

uint8_t Panel_ButtonPressed(void) { uint8_t e = s_event; s_event = 0; return e; }
uint8_t Panel_ButtonHeld(void)    { uint8_t e = s_long;  s_long  = 0; return e; }

void Panel_Task(void) {
    uint8_t now = btn_level();

    // ---- debounce ---------------------------------------------------------
    // A big panel button bounces for several milliseconds. The debounced level
    // does not move until BUTTON_DEBOUNCE_TICKS consecutive samples agree, so
    // no single bounce can start a run.
    if (now == s_raw) {
        if (s_count < 255) s_count++;
    } else {
        s_raw   = now;
        s_count = 1;
    }

    if (s_count >= BUTTON_DEBOUNCE_TICKS && s_raw != s_stable) {
        s_stable = s_raw;
        if (s_stable == 0) {
            s_down_at = now_ms();          // pressed: start the clock
        } else if (s_primed) {
            // Released. Classify by how long it was held -- doing it here
            // rather than on the press edge is what lets one button mean two
            // things without the short action firing first.
            if ((uint32_t)(now_ms() - s_down_at) >= BUTTON_LONG_PRESS_MS) s_long  = 1;
            else                                                         s_event = 1;
        }
    }

    // s_primed gates the very first press: at power-up the pull-up takes a
    // moment to charge the line, and without this a boot-time low reading would
    // launch the robot before anyone had touched it.
    if (s_stable == 1 && s_count >= BUTTON_DEBOUNCE_TICKS) s_primed = 1;

    // ---- blink ------------------------------------------------------------
    if (s_mode == LED_BLINK_SLOW || s_mode == LED_BLINK_FAST) {
        uint16_t period = (uint16_t)((s_mode == LED_BLINK_FAST)
                                      ? LED_BLINK_FAST_MS : LED_BLINK_SLOW_MS);
        led_write((uint8_t)(((now_ms() / period) & 1u) ? 1u : 0u));
    }
}
