#ifndef SIM_DELAY_H
#define SIM_DELAY_H
void sim_delay_us(double us);
#define _delay_us(x) sim_delay_us(x)
#define _delay_ms(x) sim_delay_us((x) * 1000.0)
#endif
