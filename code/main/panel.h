#ifndef PANEL_H
#define PANEL_H
#include <stdint.h>

// ============================================================================
//  OPERATOR PANEL -- one LED and one push button on PORTB.
//
//  The two are on different ports, which is why config.h declares LED_* and
//  BUTTON_* separately instead of one PANEL_PORT.
//
//  WIRING
//    PB0 --[330R]--|>|-- GND        LED, active high
//    PD6 -----------o o-- GND       button to ground, internal pull-up on
//
//  PB0 is clear of PB5/PB6/PB7 (MOSI/MISO/SCK), which matters in practice --
//  the ISP programmer can stay plugged in while the panel is wired.
//
//  The button needs no external resistor: the internal pull-up holds the pin
//  high and pressing it pulls the pin to ground, so a PRESS READS LOW. Panel_
//  ButtonPressed() hides that inversion so no caller has to remember it.
// ============================================================================

typedef enum {
    LED_OFF = 0,
    LED_ON,            // solid   -- a route is loaded, the robot is ready to go
    LED_BLINK_SLOW,    // ~1 Hz   -- finished
    LED_BLINK_FAST     // ~4 Hz   -- something is wrong, do not expect a run
} led_mode_t;

void Panel_Init(void);
void Panel_Task(void);          // once per control tick; drives blink + debounce
void Panel_SetLed(led_mode_t m);
led_mode_t Panel_Led(void);

// Both fire exactly once per physical press and then self-clear, and exactly
// one of them fires per press. They are decided ON RELEASE, not on the press
// edge -- which is what makes "short or long" unambiguous, and has the useful
// side effect that a run begins once the operator's hand is already off the
// chassis.
uint8_t Panel_ButtonPressed(void);   // held for less than BUTTON_LONG_PRESS_MS
uint8_t Panel_ButtonHeld(void);      // held for at least BUTTON_LONG_PRESS_MS

// Live debounced level, for a caller that wants "is it held".
uint8_t Panel_ButtonDown(void);
#endif
