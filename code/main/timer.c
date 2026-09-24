#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timer.h"

volatile uint32_t g_millis = 0;

void Timer_Init(void) {
    // CTC mode, prescaler 64: 16 MHz / 64 = 250 kHz.
    // OCR0 = 249 -> counts 0..249 = 250 ticks -> interrupt every 1.000 ms
    // exactly, with no rounding error.
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0  = 249;
    TIMSK |= (1 << OCIE0);
}

ISR(TIMER0_COMP_vect) {
    g_millis++;
}

uint32_t millis(void) {
    uint32_t m;
    uint8_t s = SREG;
    cli();
    m = g_millis;
    SREG = s;
    return m;
}

uint32_t micros(void) {
    uint32_t m;
    uint8_t  t;
    uint8_t s = SREG;
    cli();
    m = g_millis;
    t = TCNT0;
    // If the compare match has fired but the ISR has not run yet, g_millis is
    // one behind. Without this correction, readings taken exactly on a
    // millisecond boundary would appear to go backwards.
    if ((TIFR & (1 << OCF0)) && (t < 249)) m++;
    SREG = s;
    // TCNT0 advances at 250 kHz, so each count is exactly 4 us.
    return (m * 1000UL) + ((uint32_t)t * 4UL);
}

void Timer_WaitMs(uint32_t ms) {
    uint32_t start;

    // With interrupts off, millis() never advances and the loop below would
    // spin forever -- which is exactly how Power_Init() once hung every build
    // at boot with no output of any kind. Rather than trust every future
    // caller to know whether sei() has run yet, degrade to a cycle-counted
    // wait. Same duration, no interrupts needed, and it cannot brick the boot.
    if (!(SREG & (1 << SREG_I))) {
        while (ms--) _delay_ms(1);
        return;
    }

    start = millis();
    while ((millis() - start) < ms) { /* spin */ }
}

uint8_t Timer_Elapsed(uint32_t start, uint32_t ms) {
    return ((millis() - start) >= ms) ? 1 : 0;
}
