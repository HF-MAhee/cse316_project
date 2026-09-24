#ifndef SIM_PGMSPACE_H
#define SIM_PGMSPACE_H
#define PSTR(s) (s)
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#endif
