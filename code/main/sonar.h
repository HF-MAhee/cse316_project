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

// Same voting rule, at a caller-chosen distance: 1 when at least
// FRONT_VOTE_THRESHOLD of the last FRONT_VOTE_WINDOW front pings came back
// closer than cm. Use this when a mode needs to close in nearer than the
// junction classifier's FRONT_BLOCKED_CM without changing what counts as a
// junction everywhere else -- lowering that shared constant would make a
// T-junction's front wall register too late and read as the maze exit.
uint8_t  Sonar_FrontCloserThan(uint16_t cm);

// Raw vote count at a caller-chosen distance -- the un-thresholded version of
// the above, for telemetry. Seeing this sit at 1 for a long stretch says the
// obstacle is being detected intermittently and the stop is about to be late.
uint8_t  Sonar_FrontVotesBelow(uint16_t cm);

// Front pings discarded by the plausibility gate since boot (or the last
// Sonar_Flush()). A discarded ping cannot vote, so each costs a full 60 ms
// front refresh of detection delay. Climbing during an approach means the beam
// is being walked off the target and the stop will come late.
uint16_t Sonar_FrontGatedCount(void);

// How many of the last FRONT_VOTE_WINDOW front pings saw an obstacle.
uint8_t  Sonar_FrontVotes(void);

// 1 when a wall is present but nearer than the sensor can measure. This is a
// collision-imminent signal -- never confuse it with "no echo".
uint8_t  Sonar_IsTooClose(sonar_id_t id);

// Throw away all history. Call after any pivot: the filters hold readings
// taken while the robot was pointing somewhere else entirely.
void     Sonar_Flush(void);
#endif
