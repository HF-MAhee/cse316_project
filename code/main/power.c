#include "config.h"
#include <avr/io.h>
#include "power.h"
#include "timer.h"

static uint16_t s_min_mv  = 0xFFFF;
static uint16_t s_last_mv = 0;
static uint8_t  s_sag     = 0;

// ADMUX MUX4:0 = 11110 selects the internal 1.22V bandgap as the ADC INPUT.
// REFS1:0 = 01 selects AVCC as the ADC REFERENCE. Measuring a fixed input
// against the supply is what makes the supply itself the unknown.
#define ADMUX_BANDGAP_VS_AVCC  ((1 << REFS0) | 0x1E)

void Power_Init(void) {
    ADMUX  = ADMUX_BANDGAP_VS_AVCC;
    // Prescaler 128 -> 16MHz/128 = 125kHz. The ADC needs 50-200kHz for full
    // 10-bit accuracy; faster trades resolution away, and resolution is the
    // whole point here.
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // The bandgap needs time to start up, and the first conversion after
    // enabling the ADC is always the least accurate. Throw both away.
    Timer_WaitMs(2);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) { }
    (void)ADCW;

    s_min_mv  = 0xFFFF;
    s_last_mv = 0;
    s_sag     = 0;
}

uint16_t Power_VccMv(void) {
    uint16_t adc;

    // Re-assert the MUX every time. Nothing else in the project uses the ADC
    // today, but a future user of it would otherwise silently corrupt this
    // reading -- and a wrong supply number is worse than none.
    ADMUX = ADMUX_BANDGAP_VS_AVCC;

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) { }
    adc = ADCW;

    if (adc == 0) return 0;               // no conversion / ADC disabled

    // VCC = Vbandgap * 1024 / adc, in millivolts.
    // POWER_BANDGAP_MV * 1024 is 1249280 at the nominal 1220, so this must be
    // 32-bit. Trim POWER_BANDGAP_MV per board if an absolute figure is ever
    // needed; the relative sag does not depend on it.
    return (uint16_t)(((uint32_t)POWER_BANDGAP_MV * 1024UL) / adc);
}

void Power_Task(void) {
    uint16_t mv = Power_VccMv();
    if (mv == 0) return;
    s_last_mv = mv;
    if (mv < s_min_mv) s_min_mv = mv;
    if (mv < POWER_MIN_SAFE_MV) s_sag = 1;
}

uint16_t Power_MinMv(void)  { return (s_min_mv == 0xFFFF) ? 0 : s_min_mv; }
uint16_t Power_LastMv(void) { return s_last_mv; }
uint8_t  Power_SagSeen(void){ return s_sag; }

void Power_ResetMin(void) {
    s_min_mv = 0xFFFF;
    s_sag    = 0;
}
