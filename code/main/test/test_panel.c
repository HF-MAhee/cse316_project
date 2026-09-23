// ============================================================================
//  Host-side test for panel.c -- the LED and the start button.
//
//  Compiles the REAL panel.c against shimmed pins and clock, so the debounce,
//  the short/long split and the boot-time guard under test are the same code
//  that runs on the robot.
//
//      cc -I.. -o /tmp/tp test/test_panel.c panel.c && /tmp/tp
// ============================================================================
#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "panel.h"

extern uint8_t  Panel_TestLevel;   /* 1 = released, 0 = pressed */
extern uint8_t  Panel_TestLed;
extern uint32_t Panel_TestMs;

static int fails = 0;
static void check(int cond, const char *what) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}
/* one control tick */
static void tick(void) { Panel_TestMs += CONTROL_TICK_MS; Panel_Task(); }
static void ticks(int n) { while (n--) tick(); }
static void settle(void) { Panel_TestLevel = 1; ticks(10); }

int main(void) {
    /* PB0/PB1 are avr/io macros, so the pin numbers are not available here --
       the shimmed build never touches a register. The timings are. */
    printf("panel: debounce %d ticks (%d ms), long press %lu ms\n\n",
           BUTTON_DEBOUNCE_TICKS, BUTTON_DEBOUNCE_TICKS * CONTROL_TICK_MS,
           (unsigned long)BUTTON_LONG_PRESS_MS);

    printf("-- boot with the button reading LOW --\n");
    Panel_TestMs = 0; Panel_TestLevel = 0;      /* line not yet charged */
    Panel_Init();
    ticks(20);
    check(Panel_ButtonPressed() == 0, "a low line at power-up does NOT start a run");
    check(Panel_ButtonHeld() == 0,    "...and does not register as a hold either");

    printf("\n-- a short press --\n");
    settle();
    (void)Panel_ButtonPressed(); (void)Panel_ButtonHeld();
    Panel_TestLevel = 0; ticks(5);
    check(Panel_ButtonPressed() == 0, "nothing fires while the button is still down");
    Panel_TestLevel = 1; ticks(5);
    check(Panel_ButtonPressed() == 1, "short press fires once, on release");
    check(Panel_ButtonPressed() == 0, "...and self-clears");
    check(Panel_ButtonHeld() == 0,    "...and did not also count as a hold");

    printf("\n-- a long press --\n");
    settle();
    Panel_TestLevel = 0;
    ticks((int)(BUTTON_LONG_PRESS_MS / CONTROL_TICK_MS) + 5);
    Panel_TestLevel = 1; ticks(5);
    check(Panel_ButtonHeld() == 1,    "long press fires the hold event");
    check(Panel_ButtonPressed() == 0, "...and NOT the short one -- exactly one of the two");

    printf("\n-- bounce on the way in --\n");
    settle();
    (void)Panel_ButtonPressed(); (void)Panel_ButtonHeld();
    for (int i = 0; i < 6; i++) {        /* chatter for one tick each way */
        Panel_TestLevel = (uint8_t)(i & 1); tick();
    }
    check(Panel_ButtonPressed() == 0, "chatter shorter than the debounce fires nothing");
    Panel_TestLevel = 0; ticks(5);
    Panel_TestLevel = 1; ticks(5);
    check(Panel_ButtonPressed() == 1, "...and a real press straight after still registers once");

    printf("\n-- holding it down does not repeat --\n");
    settle();
    (void)Panel_ButtonPressed(); (void)Panel_ButtonHeld();
    Panel_TestLevel = 0;
    ticks(200);
    check(Panel_ButtonPressed() == 0, "held down: no short presses stream out");
    check(Panel_ButtonDown() == 1,    "...but the live level reads down");
    Panel_TestLevel = 1; ticks(5);
    check(Panel_ButtonHeld() == 1,    "one hold event on release");
    ticks(200);
    check(Panel_ButtonHeld() == 0,    "and nothing more after that");

    printf("\n-- the LED --\n");
    Panel_SetLed(LED_OFF);  check(Panel_TestLed == 0, "LED_OFF drives the pin low");
    Panel_SetLed(LED_ON);   check(Panel_TestLed == 1, "LED_ON drives it high");
    ticks(50);              check(Panel_TestLed == 1, "LED_ON is steady, not blinking");
    {
        int seen_on = 0, seen_off = 0;
        Panel_SetLed(LED_BLINK_SLOW);
        for (int i = 0; i < (int)(4 * LED_BLINK_SLOW_MS / CONTROL_TICK_MS); i++) {
            tick();
            if (Panel_TestLed) seen_on = 1; else seen_off = 1;
        }
        check(seen_on && seen_off, "LED_BLINK_SLOW actually toggles");
    }
    {
        int edges = 0, last = Panel_TestLed;
        Panel_SetLed(LED_BLINK_FAST);
        for (int i = 0; i < (int)(4 * LED_BLINK_FAST_MS / CONTROL_TICK_MS); i++) {
            tick();
            if (Panel_TestLed != last) { edges++; last = Panel_TestLed; }
        }
        check(edges >= 3, "LED_BLINK_FAST is visibly faster than slow");
    }

    printf("\n%s  (%d failures)\n", fails ? "*** FAILED ***" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
