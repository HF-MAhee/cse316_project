#ifndef SONAR_H
#define SONAR_H
#include <stdint.h>

typedef enum { SONAR_LEFT = 0, SONAR_FRONT = 1, SONAR_RIGHT = 2, SONAR_COUNT = 3 } sonar_id_t;

void Sonar_Init(void);

// Pings exactly ONE sensor, cycling L -> F -> R on successive calls.
// Call once per control tick. One-at-a-time satisfies the crosstalk warning
// in the course notes for free, and keeps the tick budget small.
void Sonar_Task(void);

uint16_t Sonar_Median(sonar_id_t id);  // filtered, for smooth steering
uint16_t Sonar_Latest(sonar_id_t id);  // most recent raw, for fast detection

// 1 when the value can be trusted: fresh, in range, and not taken while the
// chassis was rocking.
uint8_t  Sonar_IsValid(sonar_id_t id);

// 1 when the side is open (beyond OPENING_THRESHOLD_CM, or no echo at all).
uint8_t  Sonar_IsOpen(sonar_id_t id);

// 1 when the front is closer than FRONT_BLOCKED_CM.
uint8_t  Sonar_FrontBlocked(void);

// 1 when a wall is present but nearer than the sensor can measure. This is a
// collision-imminent signal -- never confuse it with "no echo".
uint8_t  Sonar_IsTooClose(sonar_id_t id);

// Throw away all history. Call after any pivot: the filters hold readings
// taken while the robot was pointing somewhere else entirely.
void     Sonar_Flush(void);
#endif
