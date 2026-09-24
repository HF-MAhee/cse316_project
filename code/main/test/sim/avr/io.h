/* Host shim for <avr/io.h>: just enough registers and bit names for the
   firmware sources the simulator compiles. Ports are plain variables; the
   sonar echo pins are computed by the simulator from the robot's pose. */
#ifndef SIM_AVR_IO_H
#define SIM_AVR_IO_H
#include <stdint.h>
extern uint8_t PORTA, DDRA, PORTB, DDRB, PINB, PORTC, DDRC, PORTD, DDRD, PIND;
uint8_t sim_pina(void);
#define PINA (sim_pina())
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PB0 0
#define PB1 1
#define PC2 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PD4 4
#define PD5 5
#define PD6 6
#endif
