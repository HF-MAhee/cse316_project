#include "config.h"
#include <avr/io.h>
#include "mode.h"
#include "timer.h"
#include "motors.h"
#include "panel.h"
#include "debug.h"

// ============================================================================
//  Build mode: PANEL TEST.
//
//      make MODE=panel flash
//
//  Press the button: the LED starts blinking. Press it again: it stops.
//
//  The whole test lives in Mode_PreGyro(), which main() calls BEFORE
//  MPU6050_Init(). That matters: it means this runs on a board with nothing
//  wired but the LED and the button -- no gyro, no sonar, no motors. If the
//  panel were tested from Mode_Tick() instead, a missing MPU6050 would hang the
//  I2C bus and you would never reach it, and the panel would look dead when the
//  fault was somewhere else entirely.
//
//  It exercises the real panel.c the firmware ships, not a copy of it, so what
//  passes here is what runs in MODE=wallmem.
// ============================================================================

static void report_level(uint8_t lvl) {
    Debug_P("  raw PD6 = ");
    Debug_Int((int32_t)lvl);
    if (lvl) Debug_P("  (HIGH = released, pull-up working)\r\n");
    else     Debug_P("  (LOW  = pressed)\r\n");
}

void Mode_PreGyro(void) {
    uint8_t  blinking = 0;
    uint8_t  last_raw;
    uint32_t next = millis();
    uint32_t down_at = 0;
    uint16_t presses = 0, holds = 0;

    Debug_P("\r\n=== MODE panel: LED and button test ===\r\n");
    Debug_P("LED on PB0, button on PD6.\r\n");
    Debug_P("Press the button to start the LED blinking; press again to"
            " stop.\r\n");
    Debug_P("Hold it for 2 s to fire the long-press event instead.\r\n\r\n");
    // The debug TX ring is DEBUG_TX_BUF (192) bytes and DROPS on overflow rather
    // than blocking -- right for the control loop, wrong for a banner. This
    // block plus the resting-level report below is ~255 bytes pushed far
    // faster than 38400 baud drains, so without this flush the line that
    // matters most, "raw PD6 = ...", was the one silently thrown away. This is
    // start-up, before any timing matters, so blocking here costs nothing.
    Debug_Flush();

    // Resting level, before anyone touches anything. With the internal pull-up
    // on and a normally-open switch this MUST read 1. Reading 0 here is the
    // single most likely wiring fault and it is worth naming rather than
    // leaving you to wonder why nothing responds.
    last_raw = (uint8_t)((BUTTON_PIN & (1 << BUTTON_BIT)) ? 1u : 0u);
    Debug_P("at rest:\r\n");
    report_level(last_raw);
    if (!last_raw) {
        Debug_P("\r\n*** The button reads PRESSED while untouched.\r\n");
        Debug_P("    Either it is wired to VCC instead of GND, it is a"
                " normally-CLOSED\r\n");
        Debug_P("    switch, or PD6 is shorted. The firmware expects: one leg"
                " to PD6,\r\n");
        Debug_P("    the other to GND, no external resistor.\r\n");
        Debug_P("    Nothing below will work until that is fixed.\r\n\r\n");
    }
    Debug_Flush();

    Panel_SetLed(LED_OFF);

    for (;;) {
        uint8_t raw;

        // Same fixed cadence the real firmware runs Panel_Task() at --
        // BUTTON_DEBOUNCE_TICKS is counted in these, so testing at any other
        // rate would be testing a different filter.
        if ((int32_t)(millis() - next) < 0) continue;
        next += CONTROL_TICK_MS;

        Motors_Stop();          // belt and braces: nothing should move in here
        Panel_Task();

        // Raw pin, reported only on a change. This is the layer below the
        // debounce: if these lines scroll past while the press events below
        // stay silent, the switch is chattering longer than the filter allows
        // and BUTTON_DEBOUNCE_TICKS wants raising.
        raw = (uint8_t)((BUTTON_PIN & (1 << BUTTON_BIT)) ? 1u : 0u);
        if (raw != last_raw) {
            last_raw = raw;
            report_level(raw);
            if (!raw) down_at = millis();
        }

        if (Panel_ButtonPressed()) {
            presses++;
            blinking = (uint8_t)(blinking ? 0u : 1u);
            Panel_SetLed(blinking ? LED_BLINK_SLOW : LED_OFF);

            Debug_P("PRESS #");
            Debug_Int((int32_t)presses);
            Debug_P("  held ");
            Debug_Int((int32_t)(millis() - down_at));
            Debug_P(" ms  ->  LED ");
            if (blinking) Debug_P("BLINKING\r\n");
            else          Debug_P("OFF\r\n");
            Debug_Flush();
        }

        if (Panel_ButtonHeld()) {
            holds++;
            Debug_P("HOLD #");
            Debug_Int((int32_t)holds);
            Debug_P("  held ");
            Debug_Int((int32_t)(millis() - down_at));
            Debug_P(" ms (>= ");
            Debug_Int((int32_t)BUTTON_LONG_PRESS_MS);
            Debug_P(" ms) -- in MODE=wallmem this discards the saved"
                    " route\r\n");
            Debug_Flush();
        }
    }
}

// Never reached: Mode_PreGyro() above does not return. These exist because
// mode.h requires them.
void Mode_Header(void) { }
void Mode_Begin(void) { }
void Mode_Tick(const tick_ctx_t *t) { (void)t; }
void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
