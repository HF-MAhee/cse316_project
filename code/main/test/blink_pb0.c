// ============================================================================
//  The smallest possible "is the LED alive" test.
//
//  Blinks PB0 at 1 Hz. Nothing else: no timer, no USART, no interrupts, no
//  config.h, no other file from this project. That is the entire point -- if
//  this does not blink, the fault is the LED, the resistor, the pin or the
//  power, and no amount of firmware debugging will help. If it DOES blink but
//  MODE=panel does not, the fault is in the firmware and the hardware is fine.
//
//      make blink          build it
//      make blink-flash    build and flash it
//
//  Wiring:  PB0 (DIP pin 1) --[330R]--|>|-- GND (DIP pin 11 or 31)
//                                      ^
//                          LED anode (long leg) to the resistor,
//                          cathode (short leg, flat side) to GND
// ============================================================================
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRB |= (1 << PB0);          // PB0 is an output

    for (;;) {
        PORTB |=  (1 << PB0);    // LED on
        _delay_ms(500);
        PORTB &= ~(1 << PB0);    // LED off
        _delay_ms(500);
    }
}
