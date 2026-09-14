#ifndef POWER_H
#define POWER_H
#include <stdint.h>

// ============================================================================
//  SUPPLY RAIL MONITOR -- measures VCC using nothing but the MCU itself.
//
//  WHY THIS EXISTS. Every failed Mode 10 run reset with the brown-out flag set
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
