#ifndef POWER_H
#define POWER_H
#include <stdint.h>

// ============================================================================
//  SUPPLY RAIL MONITOR -- measures VCC using nothing but the MCU itself.
//
//  WHY THIS EXISTS. Every failed early dead-end test run reset with the brown-out flag set
//  at a PWM-120 motor kick, but a brown-out is invisible after the fact: the
//  flag says "the rail went too low" and nothing says how low, for how long, or
//  how close the surviving runs came to the same edge. A multimeter cannot see
//  a 1 ms dip either. So the firmware measures it.
//
//  HOW IT WORKS, with no extra components. The ATmega32A has an internal 1.22 V
//  bandgap reference that can be selected as an ADC *input*. Measure that fixed
//  1.22 V against AVCC as the *reference* and the ratio gives VCC:
//
//      adc = 1024 * 1.22 / VCC        ->      VCC = 1.22 * 1024 / adc
//
//  So a rail that sags makes the reading RISE. No divider, no pin, no wiring.
//
//  ACCURACY, and what to trust. The bandgap is specified 1.15-1.35 V, so the
//  ABSOLUTE number can be off by up to ~10% -- do not read "4820 mV" as
//  calibrated. What it is very good at is RELATIVE change, because the error is
//  a fixed scale factor: "the rail fell 900 mV when the motors kicked" and
//  "the minimum this run was 700 mV below the idle value" are both trustworthy,
//  and they are exactly the numbers that diagnose a brown-out.
//
//  The 16 MHz line matters here: the ATmega32A datasheet requires VCC >= 4.5 V
//  at 16 MHz. Anything below that is out of spec even if the chip appears to
//  keep running, so POWER_MIN_SAFE_MV is a real limit, not a preference.
// ============================================================================

// ----------------------------------------------------------------------------
//  CRASH FORENSICS -- what the rail was doing when the MCU died.
//
//  The running minimum above lives in ordinary RAM, so a reset wipes it and the
//  most interesting reading in the whole run -- the last one before the CPU
//  stopped executing -- is exactly the one that gets lost. These copies live in
//  .noinit, so they survive the reset and can be reported on the next boot.
//
//  This is the measurement that separates the two candidate faults, which need
//  completely different fixes:
//
//    Rail declined progressively, last reading 4.0-4.5 V
//        -> genuine SAG under load. The supply cannot hold up against motor
//           current: decoupling, bulk capacitance, wiring resistance.
//
//    Rail was still healthy (4.8-5.0 V) at the last sample, then gone
//        -> abrupt COLLAPSE, not sag. The regulator shut off (over-current
//           hiccup, thermal, or two regulators fighting) or a connection
//           momentarily opened. No amount of capacitance fixes that.
//
//  The activity code says WHICH operation was running, so the answer is not
//  "somewhere in the run" but "during the pivot kick".
// ----------------------------------------------------------------------------
#define ACT_UNKNOWN      0xFF
#define ACT_IDLE         0
#define ACT_DRIVE_KICK   1
#define ACT_DRIVING      2
#define ACT_DRIVE_BRAKE  3
#define ACT_TURN_KICK    4
#define ACT_TURN_SWEEP   5
#define ACT_TURN_BRAKE   6
#define ACT_TURN_NUDGE   7
#define ACT_REVERSING    8

// Record what is happening now. Cheap (one byte to .noinit), so it can be
// called at every phase boundary without thought.
void     Power_SetActivity(uint8_t act);

// Readings recovered from BEFORE the last reset. Valid only when
// Power_CrashValid() is 1 -- i.e. SRAM held its contents, so a true power cycle
// reports nothing rather than garbage.
uint8_t  Power_CrashValid(void);
uint16_t Power_CrashMinMv(void);
uint8_t  Power_CrashActivity(void);

// Must run before report_reset_cause(): it recovers the pre-reset values and
// then re-arms the .noinit copies for this run.
void     Power_Init(void);

// One fresh reading of the supply rail, in millivolts. Blocking, ~0.5 ms
// (conversion plus the bandgap settling the ADC needs after a MUX change).
uint16_t Power_VccMv(void);

// Sample and fold into the running minimum. Call once per control tick; it is
// the minimum that matters, since the dip that resets the MCU lasts a few
// milliseconds and an average hides it completely.
void     Power_Task(void);

uint16_t Power_MinMv(void);   // lowest reading since the last reset of the
uint16_t Power_LastMv(void);  // most recent reading
void     Power_ResetMin(void);

// 1 once any reading has gone below POWER_MIN_SAFE_MV -- i.e. the rail has
// been out of the datasheet's safe operating area for 16 MHz at least once,
// whether or not the brown-out detector actually fired. This is the early
// warning that a run is about to start losing resets.
uint8_t  Power_SagSeen(void);
#endif
