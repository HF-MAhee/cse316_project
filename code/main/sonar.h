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

// Each sensor is pinged only every third control tick (round-robin), so
// "the same answer on N consecutive ticks" is really ONE ping read N times.
// Anything that wants N independent confirmations must count Sonar_Seq()
// changes instead. Sonar_HasSample() is 0 from Sonar_Flush() until that
// sensor's first ping -- until then its "reading" is a placeholder, and the
// placeholder is NO_ECHO, which Sonar_IsOpen() reports as open.
uint8_t  Sonar_Seq(sonar_id_t id);
// The latest ping in millimetres, or 0 when it was not a usable distance (no
// echo, too close, or an implausible jump). Same ping as Sonar_Latest().
uint16_t Sonar_LatestMm(sonar_id_t id);
uint16_t Sonar_PingNow(sonar_id_t id);   // blocking single ping, filters untouched
uint8_t  Sonar_HasSample(sonar_id_t id);

// 1 when the value can be trusted: fresh, in range, and not taken while the
// chassis was rocking.
uint8_t  Sonar_IsValid(sonar_id_t id);

// 1 when the side is open (beyond OPENING_THRESHOLD_CM, or no echo at all).
uint8_t  Sonar_IsOpen(sonar_id_t id);

// 1 when the front is closer than FRONT_BLOCKED_CM.
uint8_t  Sonar_FrontBlocked(void);

// Same voting rule, at a caller-chosen distance: 1 when at least
// FRONT_VOTE_THRESHOLD of the last FRONT_VOTE_WINDOW front pings came back
// closer than cm. Use this when a caller needs to close in nearer than the
// junction classifier's FRONT_BLOCKED_CM without changing what counts as a
// junction everywhere else -- lowering that shared constant would make a
// T-junction's front wall register too late and read as the maze exit.
uint8_t  Sonar_FrontCloserThan(uint16_t cm);

// How many of the last FRONT_VOTE_WINDOW front pings saw an obstacle.
uint8_t  Sonar_FrontVotes(void);

// 1 when a wall is present but nearer than the sensor can measure. This is a
// collision-imminent signal -- never confuse it with "no echo".
uint8_t  Sonar_IsTooClose(sonar_id_t id);

// Throw away all history. Call after any pivot: the filters hold readings
// taken while the robot was pointing somewhere else entirely.
void     Sonar_Flush(void);
#endif
