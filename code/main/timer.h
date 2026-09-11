#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// ============================================================================
//  Timer0-based system clock.
//
//  PROJECT RULE: _delay_ms()/_delay_us() are forbidden except for
//    (a) one-time power-up settling before this module is running, and
//    (b) the 10us HC-SR04 trigger pulse (shorter than one timer tick).
//  Everything else schedules against millis().
//
//  Timer0 -> 1 ms tick (this module)
//  Timer1 -> motor PWM (motors.c)     -- do not reuse
//  Timer2 -> reserved / unused
// ============================================================================

void     Timer_Init(void);     // call BEFORE sei()
uint32_t millis(void);
uint32_t micros(void);

// Blocking wait that still uses the timer (not _delay_ms).
void     Timer_WaitMs(uint32_t ms);

// Deadline helper. Returns 1 once `ms` has elapsed since `start`.
// Written as (now - start) so it stays correct across the 32-bit rollover
// at ~49.7 days.
uint8_t  Timer_Elapsed(uint32_t start, uint32_t ms);

#endif
