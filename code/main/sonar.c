#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include "sonar.h"
#include "timer.h"
#include "heading.h"

#define HIST 3

typedef struct {
    uint8_t  trig_bit;
    uint8_t  echo_bit;
    uint16_t hist[HIST];
    uint8_t  hist_n;
    uint8_t  hist_i;
    uint16_t latest;
    uint16_t last_good;   // last in-range measurement, for NO_ECHO disambiguation
    uint8_t  has_good;
    uint8_t  too_close;   // wall present but closer than the sensor can measure
    uint32_t stamp;
    uint8_t  confident;   // 0 if taken during a rocking window
} sonar_t;

static sonar_t s[SONAR_COUNT];
static uint8_t s_turn = 0;

// Rolling record of the last FRONT_VOTE_WINDOW front pings. Used instead of
// "N consecutive" because a yawing chassis swings the front beam off a real
// obstacle for several pings at a time.
//
// This stores the DISTANCE, not a blocked/clear flag against one fixed
// threshold. Storing the flag meant every caller was locked to
// FRONT_BLOCKED_CM, so a mode that wants to close in further than the
// junction classifier does (Mode 10's dead-end stop) had no way to ask
// without also changing what counts as a junction. A ping that must not vote
// at all -- no echo, or an implausible jump -- is stored as SONAR_NO_ECHO, so
// it is above every threshold and counts as clear at any distance.
static uint16_t s_front_dist[FRONT_VOTE_WINDOW];
static uint8_t  s_front_vi = 0;

// How many front pings the plausibility gate has thrown away. A ping it
// discards cannot vote, so each one delays an obstacle stop by a full 60 ms
// front refresh. If this climbs during an approach, the front sensor is being
// walked off the target (usually by yaw) and detection is running late --
// which is the difference between stopping short of a wall and hitting it.
static uint16_t s_front_gated = 0;

// How many of the last FRONT_VOTE_WINDOW front pings saw something closer
// than cm.
static uint8_t front_votes_below(uint16_t cm) {
    uint8_t i, votes = 0;
    for (i = 0; i < FRONT_VOTE_WINDOW; i++) {
        if (s_front_dist[i] < cm) votes++;
    }
    return votes;
}

void Sonar_Init(void) {
    uint8_t i, k;

    s[SONAR_LEFT].trig_bit  = LEFT_TRIG_BIT;
    s[SONAR_LEFT].echo_bit  = LEFT_ECHO_BIT;
    s[SONAR_FRONT].trig_bit = FRONT_TRIG_BIT;
    s[SONAR_FRONT].echo_bit = FRONT_ECHO_BIT;
    s[SONAR_RIGHT].trig_bit = RIGHT_TRIG_BIT;
    s[SONAR_RIGHT].echo_bit = RIGHT_ECHO_BIT;

    SONAR_DDR |=  (1 << LEFT_TRIG_BIT) | (1 << FRONT_TRIG_BIT) | (1 << RIGHT_TRIG_BIT);
    SONAR_DDR &= ~((1 << LEFT_ECHO_BIT) | (1 << FRONT_ECHO_BIT) | (1 << RIGHT_ECHO_BIT));
    SONAR_PORT &= ~((1 << LEFT_TRIG_BIT) | (1 << FRONT_TRIG_BIT) | (1 << RIGHT_TRIG_BIT));

    for (i = 0; i < SONAR_COUNT; i++) {
        s[i].hist_n = 0;
        s[i].hist_i = 0;
        s[i].latest = SONAR_NO_ECHO;
        s[i].last_good = 0;
        s[i].has_good  = 0;
        s[i].too_close = 0;
        s[i].stamp  = 0;
        s[i].confident = 0;
        for (k = 0; k < HIST; k++) s[i].hist[k] = SONAR_NO_ECHO;
    }
    for (i = 0; i < FRONT_VOTE_WINDOW; i++) s_front_dist[i] = SONAR_NO_ECHO;
    s_front_vi = 0;
    s_turn = 0;
}

// One blocking ping, bounded by SONAR_TIMEOUT_US (~4 ms at 70 cm).
static uint16_t ping(sonar_t *p) {
    uint32_t t0, echo_start, dur;

    SONAR_PORT &= ~(1 << p->trig_bit);
    _delay_us(2);                       // permitted exception: sub-tick pulse
    SONAR_PORT |=  (1 << p->trig_bit);
    _delay_us(10);                      // HC-SR04 requires a 10 us trigger
    SONAR_PORT &= ~(1 << p->trig_bit);

    t0 = micros();
    while (!(SONAR_PIN & (1 << p->echo_bit))) {
        if ((micros() - t0) > SONAR_TIMEOUT_US) return SONAR_NO_ECHO;
    }

    echo_start = micros();
    while (SONAR_PIN & (1 << p->echo_bit)) {
        if ((micros() - echo_start) > SONAR_TIMEOUT_US) return SONAR_NO_ECHO;
    }
    dur = micros() - echo_start;

    {   // round trip at ~343 m/s -> cm = us / 58
        uint16_t cm = (uint16_t)(dur / 58UL);
        // A sub-minimum reading is a wall ABOUT TO BE HIT, not empty space.
        // Returning NO_ECHO here (as this originally did) made the two
        // indistinguishable, so Sonar_IsOpen() reported "open" at the moment
        // of impending collision.
        if (cm < SONAR_MIN_VALID_CM)  return SONAR_TOO_CLOSE;
        if (cm > SONAR_MAX_RANGE_CM)  return SONAR_NO_ECHO;
        return cm;
    }
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

void Sonar_Task(void) {
    sonar_t *p = &s[s_turn];
    uint16_t v = ping(p);
    uint8_t  implausible_jump = 0;

    // --- Disambiguate a lost echo -----------------------------------------
    // Below ~3 cm the echo can return while the sensor is still transmitting,
    // so a very near wall often produces a plain timeout that looks exactly
    // like open space. History resolves it: if the last good reading was
    // close, the wall did not vanish in 60 ms -- it got nearer.
    if (v == SONAR_NO_ECHO && p->has_good && p->last_good <= SONAR_NEAR_LATCH_CM) {
        v = SONAR_TOO_CLOSE;
    }

    if (v == SONAR_TOO_CLOSE) {
        p->too_close = 1;
        // Report the closest measurable distance so the controller sees a
        // real number to steer away from, not a sentinel.
        v = SONAR_MIN_VALID_CM;
        p->last_good = SONAR_MIN_VALID_CM;
        p->has_good  = 1;
    } else {
        p->too_close = 0;
        if (v != SONAR_NO_ECHO) {
            p->last_good = v;
            p->has_good  = 1;
        }
    }

    // Plausibility gate. A wall cannot appear to jump more than the robot can
    // physically travel between refreshes; a big step means the beam tilted
    // off the wall or caught the floor. Discard rather than feed the filter.
    if (p->hist_n > 0 && v != SONAR_NO_ECHO && p->latest != SONAR_NO_ECHO) {
        int16_t d = (int16_t)v - (int16_t)p->latest;
        if (d < 0) d = -d;
        if (d > SONAR_MAX_JUMP_CM) {
            // keep the sample but flag it; two agreeing outliers will still
            // get through, so a genuine step change is not blocked forever
            implausible_jump = 1;
        }
    }

    p->latest = v;
    p->hist[p->hist_i] = v;
    p->hist_i = (uint8_t)((p->hist_i + 1) % HIST);
    if (p->hist_n < HIST) p->hist_n++;
    p->stamp = millis();

    // Readings taken while the chassis was pitching/rolling are noisier, but
    // they are NOT worthless -- and marking them invalid here proved to be a
    // trap: it forced Drive_Tick() into its gyro-only fallback (md=3) and
    // zeroed the wall term, which is exactly the "centring silently disabled"
    // failure seen in earlier logs. Outliers are already handled by the
    // median-of-3 and the jump gate below. Rocking now only REDUCES GAIN in
    // drive.c (BRANCH_ROCKING); it no longer discards measurements.
    // A too-close reading always stays confident: it is a safety signal and
    // must never be filtered out as noise.
    p->confident = p->too_close ? 1 : (implausible_jump ? 0 : 1);

    // Record this front ping's distance for the vote window. A real obstacle
    // gets CLOSER gradually as the robot advances; a step bigger than
    // SONAR_MAX_JUMP_CM in one 60ms refresh is the front beam catching the
    // side wall during a correction turn, not the corridor suddenly
    // shrinking, so it must not vote -- that gates it the same way
    // implausible_jump already gates p->confident above. Such a ping is stored
    // as SONAR_NO_ECHO so it reads as clear at every threshold.
    // A too-close reading always votes regardless, at distance 0: it is a
    // safety signal, not steering noise, and 0 is below any threshold.
    if (s_turn == SONAR_FRONT) {
        uint16_t d = (p->too_close) ? 0
                   : ((!implausible_jump && v != SONAR_NO_ECHO) ? v
                                                                : SONAR_NO_ECHO);
        if (implausible_jump && !p->too_close && s_front_gated < 0xFFFF) {
            s_front_gated++;
        }
        s_front_dist[s_front_vi] = d;
        s_front_vi = (uint8_t)((s_front_vi + 1) % FRONT_VOTE_WINDOW);
    }

    s_turn = (uint8_t)((s_turn + 1) % SONAR_COUNT);
}

uint16_t Sonar_Median(sonar_id_t id) {
    sonar_t *p = &s[id];
    if (p->hist_n < HIST) return p->latest;
    return median3(p->hist[0], p->hist[1], p->hist[2]);
}

uint16_t Sonar_Latest(sonar_id_t id) { return s[id].latest; }

uint8_t Sonar_IsValid(sonar_id_t id) {
    sonar_t *p = &s[id];
    if (p->hist_n == 0) return 0;
    if ((millis() - p->stamp) > SONAR_STALE_MS) return 0;
    if (!p->confident) return 0;
    if (p->latest == SONAR_NO_ECHO) return 0;
    return 1;
}

uint8_t Sonar_IsOpen(sonar_id_t id) {
    uint16_t v;
    // A wall too close to measure is the OPPOSITE of an opening. This must be
    // checked before anything else -- it is the case that previously reported
    // "open" at the instant of impending collision.
    if (s[id].too_close) return 0;
    v = Sonar_Latest(id);
    if (v == SONAR_NO_ECHO) return 1;            // nothing within range = open
    return (v > OPENING_THRESHOLD_CM) ? 1 : 0;
}

uint8_t Sonar_IsTooClose(sonar_id_t id) {
    return s[id].too_close;
}

uint8_t Sonar_FrontCloserThan(uint16_t cm) {
    // An obstacle seen in ANY FRONT_VOTE_THRESHOLD of the last
    // FRONT_VOTE_WINDOW pings counts as real. The old version tested only the
    // single most recent ping, so a correction turn that walked the beam off
    // target hid the obstacle completely -- the robot drove into a wall while
    // the front sonar reported 51..65 cm of clear space.
    return (front_votes_below(cm) >= FRONT_VOTE_THRESHOLD) ? 1 : 0;
}

uint8_t Sonar_FrontBlocked(void) {
    return Sonar_FrontCloserThan(FRONT_BLOCKED_CM);
}

// Number of the last FRONT_VOTE_WINDOW pings that saw an obstacle. Exposed
// for telemetry so a near-miss (1 vote) is visible before it becomes a stop.
uint8_t Sonar_FrontVotes(void) {
    return front_votes_below(FRONT_BLOCKED_CM);
}

uint8_t Sonar_FrontVotesBelow(uint16_t cm) {
    return front_votes_below(cm);
}

uint16_t Sonar_FrontGatedCount(void) { return s_front_gated; }

void Sonar_Flush(void) {
    uint8_t i, k;
    for (i = 0; i < FRONT_VOTE_WINDOW; i++) s_front_dist[i] = SONAR_NO_ECHO;
    s_front_vi = 0;
    for (i = 0; i < SONAR_COUNT; i++) {
        s[i].hist_n = 0;
        s[i].hist_i = 0;
        s[i].latest = SONAR_NO_ECHO;
        s[i].last_good = 0;
        s[i].has_good  = 0;
        s[i].too_close = 0;
        s[i].confident = 0;
        s[i].stamp = 0;
        for (k = 0; k < HIST; k++) s[i].hist[k] = SONAR_NO_ECHO;
    }
}
