#include "config.h"
#include <avr/io.h>
#include "solver.h"
#include "telemetry.h"
#include "timer.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "wallmem.h"
#include "panel.h"
#include "debug.h"

// ============================================================================
//  THE TWO-RUN SOLVER: wall follower with memory.
//
//  RUN 1 (explore)  left-hand rule, L > F > R > U. One byte logged per
//                   decision point. On reaching the exit the log is collapsed
//                   -- every dead-end excursion folds into the single turn it
//                   was equivalent to -- and the result is written to EEPROM.
//
//  RUN 2 (speed)    the collapsed route is replayed. Every junction is checked
//                   against the signature stored for it before its turn is
//                   acted on.
//
//  BOTH RUNS HAPPEN IN ONE POWER-UP, gated by the panel button:
//
//    LED off    -> press starts RUN 1 (explore)
//    LED solid  -> a collapsed route is loaded; press starts RUN 2 (speed)
//    LED fast   -> the explore log did not collapse to a route; do not expect
//                  a fast run, the next press explores again
//    LED slow   -> run 2 finished
//
//  So the sequence on the day is: press, watch it explore, wait for the LED to
//  come on, carry the robot back to the start cell, press again.
//
//  The route is also written to EEPROM at the end of run 1, so a power cycle
//  between the runs works too -- the robot comes back up with the LED already
//  on, waiting for the button. That matters on this chassis, where the power
//  cycle is sometimes not the operator's choice.
//
//  To explore again: hold the button 2 s, or set WALLMEM_FORCE_EXPLORE to 1.
// ============================================================================

typedef enum {
    WM_ARMED = 0,      // stopped, LED showing which run is next, waiting for the button
    WM_STARTUP,
    WM_DRIVING,        // centring forward, watching for a junction
    WM_APPROACH,       // decided; drive on so the AXLE reaches the turn point
    WM_LOOK,           // something other than corridor seen: drive on, look, then classify
    WM_STOPPING,
    WM_RECAL,
    WM_DECIDE,         // execute the pivot
    WM_RECOVER,        // gyro-only while the sonar filters refill
    WM_DONE,
    WM_DONE_IDLE,      // run 2 over; slow blink until the button is pressed again
    WM_FAULT
} wm_state_t;

static wm_state_t s_state = WM_STARTUP;
static uint32_t   s_entered = 0;
static uint32_t   s_leg_started = 0;
static uint32_t   s_run_started = 0;

static uint8_t    s_exploring = 1;      // 0 = replaying a saved route
static uint8_t    s_degraded  = 0;      // replay failed; finishing on left-hand
static uint8_t    s_pending_turn = WM_TURN_F;

// ---------------------------------------------------------------------------
//  What the sonars say, debounced.
//
//  Counted in FRESH PINGS, not control ticks. Each sensor is pinged every
//  third tick, so the old per-tick counters met "2 confirmations" with one
//  ping read twice -- a single stray echo was enough to decide a turn, and in
//  the real log a single bad right-hand ping called a dead end in E1.
// ---------------------------------------------------------------------------
static uint8_t  s_open_l = 0, s_open_r = 0;     // of the last OPENING_WINDOW pings, how many open
static uint8_t  s_mask_l = 0, s_mask_r = 0;     // those pings, 1 bit each, newest lowest
static uint16_t s_ptime_l[OPENING_WINDOW], s_ptime_r[OPENING_WINDOW];  // when (ms, low 16)
static uint8_t  s_shut_l = 0, s_shut_r = 0;     // ... reading a wall
static uint8_t  s_block_f = 0;                  // consecutive front pings blocked
static uint8_t  s_close_f = 0, s_close_seq = 0; // last 3 front pings within FRONT_STOP_CM
static uint8_t  s_seq_l, s_seq_f, s_seq_r;      // last ping counted, per sensor
// If nonzero, where the next opening on that side really starts (millis):
// set when the robot is known to be at a cell edge -- leaving the turn cell,
// or leaving a junction cell it went straight through -- and consumed by the
// next ping. An opening already open on that ping began at the edge, not
// where it happened to be first seen.
static uint32_t s_hint_l = 0, s_hint_r = 0;
static uint32_t s_open_since_l, s_open_since_r; // first ping of the current open run
static uint32_t s_leg_edge_t = 0;               // side sonars crossed out of the turn cell
static uint32_t s_side_guard_t = 0;             // side readings ignored until then

// A side opening the robot has just driven straight past. It stays masked --
// reported as a wall -- until the junction cell has been crossed (see
// expire_latch). Without this the same opening re-triggered a FWD decision
// every 40 ms for as long as it was in view: 11 records for one junction in
// the real log.
static uint8_t  s_latch_l = 0, s_latch_r = 0;
static uint32_t s_latch_until_l = 0, s_latch_until_r = 0;

// Evidence gathered while LOOKing. A side counts as open if it read open at
// ANY point in the look: near the start of an opening the end of the wall
// stub beside it still echoes from off-axis (the left sonar read ~18 cm with
// C1 wide open in the real log), so the first readings are the least reliable.
static uint8_t  s_acc_l = 0, s_acc_r = 0;
static uint8_t  s_look_n_l, s_look_n_r, s_look_o_l, s_look_o_r;   // pings, open pings
static uint32_t s_junction_t0 = 0;  // when the side sonar reached the opening
static uint8_t  s_to_wall = 0;      // approach ends at the front wall, not on time
static turn_dir_t s_uturn_dir = UTURN_TIE_DIR;

static uint8_t s_recover_begun = 0;

// Run statistics -- this is what the demo is measured with. "segments" counts
// junction-to-junction runs, not 40 cm cells: the robot has no encoders, so it
// genuinely does not know how many cells it crossed, and a number that looked
// like a cell count would be a number nobody could trust.
static uint16_t s_segments = 0, s_turns90 = 0, s_turns180 = 0;
static uint8_t  s_finished = 0;
// Sticky across runs: a log that would not collapse leaves the LED blinking
// rather than dark, so "not ready" never looks like "not finished yet".
static uint8_t  s_collapse_failed = 0;

static void enter(wm_state_t st) {
    s_state = st;
    s_entered = millis();
    s_recover_begun = 0;
}
static uint8_t in_state_for(uint32_t ms) { return ((millis() - s_entered) >= ms) ? 1 : 0; }

static uint8_t inc(uint8_t c) { return (uint8_t)((c < 255u) ? (c + 1u) : c); }

// Start of a leg (a run starting, or a turn finished): nothing seen so far
// counts, and any latch belonged to the corridor the robot has just left.
static void clear_debounce(void) {
    s_open_l = s_open_r = s_shut_l = s_shut_r = s_block_f = 0;
    s_mask_l = s_mask_r = 0;
    s_latch_l = s_latch_r = 0;
    s_hint_l = s_hint_r = s_leg_edge_t;
    s_to_wall = 0;
    s_seq_l = Sonar_Seq(SONAR_LEFT);
    s_seq_f = Sonar_Seq(SONAR_FRONT);
    s_seq_r = Sonar_Seq(SONAR_RIGHT);
    s_close_f = 0;
    s_close_seq = s_seq_f;
}

// One side sonar, one fresh ping. A side counts as open when OPENING_VOTES of
// its last OPENING_WINDOW pings read open -- a vote, not an unbroken run.
// HC-SR04 dropouts read "open" and come in bursts, so a run of 2 or 3 is not
// evidence; but one dropout's opposite -- a lone echo off a wall end in the
// middle of a real opening -- must not reset the count either, or an opening
// with a wall end at each side can go unconfirmed all the way across.
static uint8_t popcount_window(uint8_t m) {
    uint8_t k, n = 0;
    for (k = 0; k < OPENING_WINDOW; k++) if (m & (1u << k)) n++;
    return n;
}

static void side_ping(uint8_t open, uint8_t *mask, uint16_t *ptime, uint8_t *n_open,
                      uint8_t *n_shut, uint32_t *hint, uint32_t *since) {
    uint8_t m = *mask, k;
    if (open && m == 0) *since = *hint ? *hint : (millis() - OPENING_DETECT_LAG_MS);
    m = (uint8_t)(((m << 1) | (open ? 1u : 0u)) & ((1u << OPENING_WINDOW) - 1u));
    for (k = OPENING_WINDOW - 1; k > 0; k--) ptime[k] = ptime[k - 1];
    ptime[0] = (uint16_t)millis();
    *mask = m;
    *n_open = popcount_window(m);
    *n_shut = open ? 0 : inc(*n_shut);
    *hint = 0;
}

// Going straight through a junction masks its openings (s_latch_*) until the
// junction cell has been crossed: CORRIDOR_WIDTH_CM after the opening began.
// Not "until a wall is seen": in the demo maze D0 and C0 sit side by side,
// separated only by the END of the C0|D0 stub, so there is no wall between
// them to see -- C0 stayed masked and C1 was logged as FORCED_RIGHT.
//
// On expiry, the pings already taken past the cell edge still count toward
// the next cell's opening; only the ones from before it are dropped. On a
// robot a little faster than TRAVEL_SPEED_CMS the latch expires well into
// the next cell, and starting its count from zero there left too few pings
// to confirm the opening before the junction was classified.
static void expire_latch(uint8_t *latch, uint32_t until, uint8_t *mask, uint16_t *ptime,
                         uint8_t *n_open, uint32_t *hint) {
    if (*latch && (int32_t)(millis() - until) >= 0) {
        uint16_t edge = (uint16_t)(until - FWD_PASS_MARGIN_MS);   // the next cell starts here
        uint8_t  k;
        *latch = 0;
        for (k = 0; k < OPENING_WINDOW; k++)
            if ((int16_t)(ptime[k] - edge) < 0) *mask &= (uint8_t)~(1u << k);
        *n_open = popcount_window(*mask);
        if (*mask == 0) *hint = until - FWD_PASS_MARGIN_MS;
        else            *hint = 0;
    }
}

static void update_debounce(void) {
    // Nothing counts until every sonar has been pinged since the last flush.
    // Until then a sensor's reading is the NO_ECHO placeholder, which reads
    // as OPEN -- that is what produced a junction on the very first tick
    // after GO in both real logs (a LEFT that did not exist in one of them).
    if (!Sonar_HasSample(SONAR_LEFT) || !Sonar_HasSample(SONAR_FRONT) ||
        !Sonar_HasSample(SONAR_RIGHT)) return;

    // Straight after a turn the side sonars are still inside the turn cell,
    // looking down whatever branches it has -- including the corridor the
    // robot has just come out of. None of that is the next junction. Ignore
    // the sides until they are out of the cell and past the echo off the end
    // of its walls; the first ping after that dates any opening back to the
    // cell edge (see side_ping).
    if ((int32_t)(millis() - s_side_guard_t) < 0) {
        s_seq_l = Sonar_Seq(SONAR_LEFT);
        s_seq_r = Sonar_Seq(SONAR_RIGHT);
    }
    expire_latch(&s_latch_l, s_latch_until_l, &s_mask_l, s_ptime_l, &s_open_l, &s_hint_l);
    expire_latch(&s_latch_r, s_latch_until_r, &s_mask_r, s_ptime_r, &s_open_r, &s_hint_r);
    if (Sonar_Seq(SONAR_LEFT) != s_seq_l) {
        s_seq_l = Sonar_Seq(SONAR_LEFT);
        s_look_n_l = inc(s_look_n_l);
        if (Sonar_IsOpen(SONAR_LEFT)) s_look_o_l = inc(s_look_o_l);
        side_ping(Sonar_IsOpen(SONAR_LEFT), &s_mask_l, s_ptime_l, &s_open_l, &s_shut_l,
                  &s_hint_l, &s_open_since_l);
    }
    if (Sonar_Seq(SONAR_RIGHT) != s_seq_r) {
        s_seq_r = Sonar_Seq(SONAR_RIGHT);
        s_look_n_r = inc(s_look_n_r);
        if (Sonar_IsOpen(SONAR_RIGHT)) s_look_o_r = inc(s_look_o_r);
        side_ping(Sonar_IsOpen(SONAR_RIGHT), &s_mask_r, s_ptime_r, &s_open_r, &s_shut_r,
                  &s_hint_r, &s_open_since_r);
    }
    if (Sonar_Seq(SONAR_FRONT) != s_seq_f) {
        s_seq_f = Sonar_Seq(SONAR_FRONT);
        s_block_f = Sonar_FrontBlocked() ? inc(s_block_f) : 0;
    }
}

// The three bits everything downstream is decided from. Note that FORWARD is
// reported as OPEN, i.e. the complement of "front blocked" -- wallmem.h stores
// openness, not obstruction, so that the signature is directly comparable with
// what the classifier saw in the other run.
static void read_openness(uint8_t *f, uint8_t *l, uint8_t *r) {
    *l = (uint8_t)((s_open_l >= OPENING_VOTES && !s_latch_l) ? 1u : 0u);
    *r = (uint8_t)((s_open_r >= OPENING_VOTES && !s_latch_r) ? 1u : 0u);
    *f = (uint8_t)((s_block_f >= OPENING_CONFIRM) ? 0u : 1u);
}

// Close enough to the wall ahead to stop and turn: 2 of the last 3 front
// pings under FRONT_STOP_CM. Not one ping -- a single garbage echo (crosstalk,
// a reflection) stopped the robot dead in open space, just outside the exit,
// where it then "turned" at a wall that was not there. And not two in a row
// either -- close to a wall a sonar can lose every other echo, and in
// simulation "two in a row" then never happened and the robot drove into it.
// FRONT_STOP_CM allows for the extra ping of travel.
static uint8_t front_close(void) {
    if (Sonar_Seq(SONAR_FRONT) != s_close_seq) {
        uint8_t near = (Sonar_IsValid(SONAR_FRONT) && Sonar_Latest(SONAR_FRONT) < FRONT_STOP_CM)
                       ? 1u : 0u;
        s_close_seq = Sonar_Seq(SONAR_FRONT);
        s_close_f = (uint8_t)(((s_close_f << 1) | near) & 0x07u);   // last 3 pings
    }
    return ((s_close_f & 1u) + ((s_close_f >> 1) & 1u) + ((s_close_f >> 2) & 1u) >= 2) ? 1u : 0u;
}
// Rotate AWAY from the nearer wall. The front corners sweep ~17 cm into the
// side being turned towards while the rear corners only reach ~10.6 cm the
// other way, so the turning side needs about 6 cm more room.
//
// "Nearer" is judged at the AXLE, which is what the robot pivots about. The
// side sonars sit SIDE_SONAR_TO_AXLE_CM ahead of it, so a robot skewed a few
// degrees reads centred at the sonars while its axle is well off to one side
// -- and in a 40 cm dead end that decides whether a corner touches the wall.
static turn_dir_t pick_180_dir(void) {
    uint16_t l_cm = Sonar_Median(SONAR_LEFT);
    uint16_t r_cm = Sonar_Median(SONAR_RIGHT);
    uint8_t  l_ok = Sonar_IsValid(SONAR_LEFT);
    uint8_t  r_ok = Sonar_IsValid(SONAR_RIGHT);

    if (l_ok && r_ok) {
        // Axle offset to the LEFT of centre, mm: the sonars' offset, less the
        // part that is only the skew swinging them sideways.
        int32_t off_mm = ((int32_t)r_cm - (int32_t)l_cm) * 5L;
        off_mm -= ((int32_t)Heading_GridErrorTenths() * SIDE_SONAR_TO_AXLE_CM * 10L) / 573L;
        if (off_mm <= -(int32_t)UTURN_DECIDE_MARGIN_CM * 5L) return TURN_LEFT;   // nearer the right
        if (off_mm >=  (int32_t)UTURN_DECIDE_MARGIN_CM * 5L) return TURN_RIGHT;  // nearer the left
    } else if (l_ok) {
        return TURN_RIGHT;      // only the left wall is visible -- turn away
    } else if (r_ok) {
        return TURN_LEFT;
    }
    return UTURN_TIE_DIR;
}

// ---------------------------------------------------------------------------
//  The decision. This is the only place the two runs differ.
// ---------------------------------------------------------------------------
static uint8_t decide_turn(uint8_t f, uint8_t l, uint8_t r) {
    uint8_t turn;

    if (s_exploring || s_degraded) {
        turn = WallMem_LeftHand(f, l, r);
        if (s_exploring && !WallMem_Record(f, l, r, turn)) {
            Debug_P("*** LOG FULL -- explore run is no longer usable.\r\n");
            Debug_P("    Raise WALLMEM_MAX_RECORDS and run it again.\r\n");
        }
        return turn;
    }

    switch (WallMem_Play(f, l, r, &turn)) {
    case WM_PLAY_OK:
        return turn;

    case WM_PLAY_MISMATCH:
        Debug_P("*** REPLAY MISMATCH at record ");
        Debug_Int((int32_t)WallMem_PlaybackIndex());
        Debug_P(": stored ");
        {
            uint8_t rec = WallMem_At(WallMem_PlaybackIndex());
            Debug_StrP(WallMem_TypeName((uint8_t)((rec & WM_OPEN_F) ? 1u : 0u),
                                        (uint8_t)((rec & WM_OPEN_L) ? 1u : 0u),
                                        (uint8_t)((rec & WM_OPEN_R) ? 1u : 0u)));
        }
        Debug_P(", standing in ");
        Debug_StrP(WallMem_TypeName(f, l, r));
        Debug_P("\r\n");
        break;

    case WM_PLAY_EXHAUSTED:
    default:
        Debug_P("*** ROUTE EXHAUSTED before the exit appeared\r\n");
        break;
    }

    // The stored route no longer describes where the robot is standing, so
    // acting on it would be worse than not having it. Finish on the rule that
    // needs no memory at all.
#if WALLMEM_HALT_ON_MISMATCH
    Debug_P("    halting (WALLMEM_HALT_ON_MISMATCH=1)\r\n");
    Drive_Stop();
    enter(WM_FAULT);
    return WM_TURN_F;
#else
    Debug_P("    falling back to the left-hand rule for the rest of the run\r\n");
    s_degraded = 1;
    return WallMem_LeftHand(f, l, r);
#endif
}

// Put the robot in its waiting state and say -- on the LED and on the wire --
// which run the button is about to start. Everything that ends a run comes
// through here, so there is exactly one place that decides what "ready" means.
static void arm(void) {
    Motors_Stop();
    s_finished = 0;
    s_degraded = 0;
    s_segments = 0;
    s_turns90  = 0;
    s_turns180 = 0;
    WallMem_RewindPlayback();

    if (WallMem_HaveSolution() && WallMem_Count() > 0) {
        s_exploring = 0;
        Panel_SetLed(LED_ON);
        Debug_P("\r\n>>> LED ON -- READY FOR RUN 2.\r\n");
        Debug_P("    Put the robot back in the START cell facing in, then press"
                " the button.\r\n");
    } else {
        s_exploring = 1;
        // Clear the log before an explore run, not after. A collapse that
        // failed leaves a short, half-folded string in RAM with s_solved clear,
        // and WallMem_Record() appends at s_count -- so without this the next
        // explore run would log on top of the wreckage of the last one.
        WallMem_Reset();
        // A failed collapse leaves the LED blinking rather than dark, because
        // "not ready" and "not finished yet" otherwise look the same.
        Panel_SetLed(s_collapse_failed ? LED_BLINK_FAST : LED_OFF);
        Debug_P("\r\n>>> READY FOR RUN 1 (explore). Press the button to"
                " start.\r\n");
    }
    Debug_Flush();
    enter(WM_ARMED);
}

static void report_run(void) {
    Debug_P("\r\n---- run summary ----\r\n");
    Debug_KVF("mode", (int32_t)(s_exploring ? 1 : 2));
    Debug_KVF("segments", (int32_t)s_segments);
    Debug_KVF("turn90", (int32_t)s_turns90);
    Debug_KVF("turn180", (int32_t)s_turns180);
    Debug_KVF("ms", (int32_t)(millis() - s_run_started));
    Debug_KVF("degraded", (int32_t)s_degraded);
    // The gyro scale trim learned from the walls. Anything past ~+/-15
    // (1.5%) run after run: recalibrate GYRO_LSB_MS_PER_DEGREE instead.
    Debug_KVF("gyro_trim_ppt10", (int32_t)Heading_ScaleCorrection());
    Debug_NL();
    Debug_Flush();
}

// Classify, decide, log. Returns 1 if the robot must stop and turn, 0 if it
// carries straight on (or the reading turned out to be a plain corridor).
static uint8_t commit(uint8_t f, uint8_t l, uint8_t r) {
    if (!WallMem_IsDecision(f, l, r)) {
        // It was a flicker -- the look found a plain corridor. Said on the
        // wire, because a real junction ending up here is a missed record.
        Debug_P("look: corridor after all\r\n");
        enter(WM_DRIVING);
        return 0;
    }
    s_pending_turn = decide_turn(f, l, r);
    if (s_state == WM_FAULT) return 0;          // decide_turn halted us

    Debug_P("junction ");
    Debug_StrP(WallMem_TypeName(f, l, r));
    Debug_P(" -> ");
    Debug_StrP(WallMem_TurnName(s_pending_turn));
    Debug_P("\r\n");

    if (s_pending_turn == WM_TURN_F) {
        // Straight on. Mask the openings being passed until the junction
        // cell has been crossed, so this junction is logged exactly once.
        s_latch_l = l;
        s_latch_r = r;
        s_latch_until_l = s_open_since_l + FWD_PASS_MS;
        s_latch_until_r = s_open_since_r + FWD_PASS_MS;
        enter(WM_DRIVING);
        return 0;
    }
    return 1;
}

// ===========================================================================
//  Entry points
// ===========================================================================
void Solver_Init(void) {
    Debug_P("\r\n=== two-run maze solver: wall follower with memory ===\r\n");
    // The debug TX ring is DEBUG_TX_BUF (192) bytes and DROPS on overflow
    // rather than blocking. This banner plus the run-selection lines below is
    // just over that, so without a flush here the tail of the start-up text --
    // the placement instruction -- is the part that silently disappears. This
    // is start-up, before any timing matters, so blocking costs nothing.
    Debug_Flush();

    WallMem_Reset();

#if WALLMEM_FORCE_EXPLORE
    Debug_P("WALLMEM_FORCE_EXPLORE=1 -- ignoring any saved route.\r\n");
    s_exploring = 1;
#else
    if (WallMem_Load() && WallMem_HaveSolution() && WallMem_Count() > 0) {
        s_exploring = 0;
        Debug_P("RUN 2 (speed): loaded a solved route from EEPROM.\r\n");
        Debug_P("route:\r\n");
        WallMem_Dump();
    } else {
        s_exploring = 1;
        Debug_P("RUN 1 (explore): no usable saved route.\r\n");
        Debug_P("Left-hand rule; every decision is logged.\r\n");
    }
#endif
    Debug_P("Place the robot in the START cell facing INTO the maze.\r\n");
    Debug_Flush();

    Telemetry_Header();
}

void Solver_Begin(void) {
    clear_debounce();
    s_leg_started = millis();
    s_run_started = millis();
    // Nothing moves until the button is pressed -- including run 1. A robot
    // that drives off on a timer while it is still being positioned is the
    // failure this removes, and it makes both runs start the same way.
    arm();
}

void Solver_Tick(const tick_ctx_t *t) {
    int16_t rate = t->rate;
    uint8_t f, l, r;

    // Per-RUN time limit, counted from the button press (s_run_started is set
    // on GO), and only while the robot is actually moving.
    //
    // It used to be a global limit in main.c counted from BOOT, which fought
    // the two-run flow: the robot legitimately sits armed and waiting between
    // runs, so explore + carry-back + speed run could easily pass five minutes
    // since power-on, at which point the whole firmware froze in a for(;;) --
    // mid-run-2 if unlucky, with the LED frozen too. A run that really does go
    // on this long is lost, so stop it and say so, but keep the loop alive.
    if (s_state >= WM_DRIVING && s_state <= WM_RECOVER &&
        (millis() - s_run_started) > MAX_RUN_MS) {
        Drive_Stop();
        Debug_P("*** RUN LIMIT: this run has been moving for over ");
        Debug_Int((int32_t)(MAX_RUN_MS / 1000UL));
        Debug_P(" s -- stopped.\r\n");
        Debug_Flush();
        enter(WM_FAULT);
        return;
    }

    switch (s_state) {

    case WM_ARMED:
        Motors_Stop();
        if (Panel_ButtonHeld()) {
            // Long press: throw the route away and explore again. Checked
            // before the short press, and only one of the two can be latched.
            Debug_P("\r\nbutton held -- discarding the saved route\r\n");
            WallMem_Erase();
            WallMem_Reset();
            s_collapse_failed = 0;
            arm();
            break;
        }
        if (Panel_ButtonPressed()) {
            Debug_P("\r\nbutton pressed -- starting RUN ");
            Debug_Int((int32_t)(s_exploring ? 1 : 2));
            Debug_P(", hold still\r\n");
            Debug_Flush();
            Panel_SetLed(LED_OFF);
            enter(WM_STARTUP);
        }
        break;

    case WM_STARTUP:
        if (in_state_for(STARTUP_DELAY_MS)) {
            // The operator has just handled the chassis -- carried it across
            // the room and set it down. Re-zero on the spot: the bias is still
            // good but the accumulated heading is meaningless, and the sonar
            // filters are full of readings taken while it was in mid-air.
            if (!Gyro_CalibrateQuick()) Debug_P("recal SKIPPED -- not still\r\n");
            Heading_Reset();
            // The robot was placed square in the start cell: that direction
            // IS the maze grid, for the rest of the run.
            Heading_GridReset();
            Sonar_Flush();
            s_leg_edge_t   = 0;          // the start cell: no edge to date from
            s_side_guard_t = millis();
            clear_debounce();
            Debug_P("GO\r\n");
            Drive_Begin();
            s_run_started = millis();
            s_leg_started = millis();
            enter(WM_DRIVING);
        }
        break;

    case WM_DRIVING:
        Drive_Tick(rate);
        update_debounce();
        read_openness(&f, &l, &r);

        // A leg far longer than any cell means a wall the sonar never saw --
        // angled or soft surfaces reflect the pulse away and read as clear.
        //
        // It is forced through the SAME decision path as a sonar-detected dead
        // end, and that matters more than it looks: handling it separately
        // would execute a turn that the log does not contain, and every record
        // after it would then describe a junction the robot is no longer
        // standing in. A desynchronised log replays into a wall.
        if ((millis() - s_leg_started) > MAX_LEG_MS) {
            Debug_P("leg timeout -- treating as a dead end\r\n");
            s_pending_turn = decide_turn(0, 0, 0);
            if (s_state == WM_FAULT) break;
            Drive_Stop();
            enter(WM_STOPPING);
            break;
        }
        if (!WallMem_IsDecision(f, l, r)) break;   // plain corridor: drive on

        // Something other than a corridor. Do NOT classify it yet: at this
        // moment the side sonar has only just reached the opening and the
        // front sonar is still a corridor-width short of whatever wall ends
        // this cell. Both of the real logs' misreads came from deciding here:
        //   - the E1 corner logged as FWD_OR_LEFT (front wall still 33 cm off)
        //   - the D1 T junction read as FWD_OR_RIGHT, because the left sonar
        //     was still getting an echo off the end of the C0|D0 stub.
        // So drive on and look first. Timing is taken from when the opening
        // was first seen, so the extra look costs no positioning accuracy.
        s_acc_l = l;
        s_acc_r = r;
        s_look_n_l = s_look_n_r = s_look_o_l = s_look_o_r = 0;
        s_junction_t0 = millis();
        if (l && (int32_t)(s_open_since_l - s_junction_t0) < 0) s_junction_t0 = s_open_since_l;
        if (r && (int32_t)(s_open_since_r - s_junction_t0) < 0) s_junction_t0 = s_open_since_r;
        enter(WM_LOOK);
        break;

    case WM_LOOK:
        // Drive on JUNCTION_LOOK_CM and gather evidence, then classify once.
        // A side counts as open if it read open at any point; the front is
        // judged at the end, when a T or corner's wall has had time to come
        // within FRONT_BLOCKED_CM. All three still open after the look is
        // either the exit or a T whose front wall has not closed in yet, and
        // that is settled by driving on to EXIT_CONFIRM_CM -- a T's wall
        // always turns up inside EXIT_FALSE_WINDOW_CM, the exit's never does.
        Drive_Tick(rate);
        update_debounce();
        read_openness(&f, &l, &r);
        s_acc_l |= l;
        s_acc_r |= r;

        if (front_close()) {
            // Reached the wall ahead: this is the turning point.
            Drive_Stop();
            s_to_wall = 1;
            if (commit(0, s_acc_l, s_acc_r)) enter(WM_STOPPING);
            break;
        }
        if (!in_state_for(JUNCTION_LOOK_MS)) break;
        if (f && s_acc_l && s_acc_r) {
            if (!in_state_for(EXIT_CONFIRM_MS)) break;
            // The exit is final, so it is judged on what the sonars say NOW,
            // not on everything they ever said during the look -- a burst of
            // lost echoes on one side must not end the run.
            if (l && r) {
                Drive_Stop();
                enter(WM_DONE);
                break;
            }
            // Not the exit: one side's "open" was a burst of dropouts. Keep a
            // side that read open for most of the look -- judging on this
            // instant alone dropped a real opening the robot had already
            // driven past.
            s_to_wall = 0;
            if (commit(f, (uint8_t)(s_look_o_l * 2u >= s_look_n_l && s_look_n_l),
                          (uint8_t)(s_look_o_r * 2u >= s_look_n_r && s_look_n_r)))
                enter(WM_APPROACH);
            break;
        }
        s_to_wall = (uint8_t)(!f);
        if (commit(f, s_acc_l, s_acc_r)) enter(WM_APPROACH);
        break;

    case WM_APPROACH:
        // Drive on until the AXLE is where the pivot belongs: the middle of
        // the opening (on time, from when the side sonar reached it), or the
        // middle of the cell (at the front wall).
        Drive_Tick(rate);
        if (front_close() ||
            (s_to_wall ? in_state_for(APPROACH_WALL_TIMEOUT_MS)
                       : ((millis() - s_junction_t0) >= APPROACH_TIME_MS))) {
            Drive_Stop();
            enter(WM_STOPPING);
        }
        break;

    case WM_STOPPING:
        Motors_Stop();
        if (in_state_for(GYRO_SETTLE_MS)) enter(WM_RECAL);
        break;

    case WM_RECAL:
        // Choose the U-turn direction NOW, from the sonar readings taken while
        // stopped. This used to happen after the flush below, when every
        // sensor was invalid, so it always fell through to UTURN_TIE_DIR.
        if (s_pending_turn == WM_TURN_U) {
            s_uturn_dir = pick_180_dir();
            // Which way, and why: the pivot sweeps ~17 cm and a dead end only
            // has ~20 cm each side of centre, so this choice is what keeps
            // the front corners off the nearer wall.
            Debug_KVF("uturn L", (int32_t)Sonar_Median(SONAR_LEFT));
            Debug_KVF("R", (int32_t)Sonar_Median(SONAR_RIGHT));
            if (s_uturn_dir == TURN_LEFT) Debug_P("-> left\r\n");
            else                          Debug_P("-> right\r\n");
        }
        // Refresh the gyro bias while genuinely stationary; thermal drift over
        // a long explore run otherwise creeps into every later turn.
        if (!Gyro_CalibrateQuick()) Debug_P("recal SKIPPED\r\n");
        Heading_Reset();
        Sonar_Flush();
        enter(WM_DECIDE);
        break;

    case WM_DECIDE: {
        turn_result_t res;
        // Stopped at a front wall: put the pivot point in the middle of the
        // cell before turning. A robot that rolled 3 cm further than usual
        // would otherwise sweep its front corners into the wall ahead.
        if (s_to_wall) Turn_CentreOnWall();
        if (s_pending_turn == WM_TURN_U) {
            Turn_180(s_uturn_dir, &res);
            s_turns180++;
        } else {
            Turn_90((s_pending_turn == WM_TURN_L) ? TURN_LEFT : TURN_RIGHT, &res);
            s_turns90++;
        }
        Debug_KVF("ang10", res.achieved_tenths);
        Debug_KVF("grid10", res.grid_error_tenths);
        Debug_KVF("nudges", res.nudges_used);
        Debug_KVF("conv", res.converged);
        // A wrong-way pivot is the expensive failure here: the robot is now
        // facing somewhere the stored route knows nothing about, and run 2's
        // very next signature check is what catches it.
        if (res.wrong_way) Debug_P("WRONG WAY ");
        Debug_NL();
        enter(WM_RECOVER);
        break;
    }

    case WM_RECOVER:
        // Drive out of the turn cell on the gyro alone while the sonar filters
        // refill. The pivot happened with the axle at the cell centre, so the
        // side sonars leave the cell SIDE_EDGE_MS in -- note when, in case they
        // leave it straight into an opening, and ignore them until then.
        if (!s_recover_begun) {
            Drive_Begin();
            s_recover_begun = 1;
            s_leg_edge_t   = millis() + SIDE_EDGE_MS;
            s_side_guard_t = s_leg_edge_t + OPENING_DETECT_LAG_MS;
        }
        Drive_Tick(rate);
        if (in_state_for(RECOVER_MS)) {
            clear_debounce();
            s_leg_started = millis();
            s_segments++;
            enter(WM_DRIVING);
        }
        break;

    case WM_DONE:
        Motors_Stop();
        if (s_finished) break;
        s_finished = 1;
        s_segments++;

        Debug_P("\r\n*** EXIT REACHED ***\r\n");
        report_run();

        if (s_exploring) {
            uint8_t stray = 0;
            uint8_t before = WallMem_Count();

            Debug_P("explore log:\r\n");
            WallMem_Dump();
            WallMem_Reduce(&stray);
            Debug_P("collapsed ");
            Debug_Int((int32_t)before);
            Debug_P(" -> ");
            Debug_Int((int32_t)WallMem_Count());
            Debug_P(" records, stray U-turns=");
            Debug_Int((int32_t)stray);
            Debug_P("\r\n");
            // The failure explanation below is ~184 bytes on its own; on top of
            // this line it would overflow the 192-byte debug ring and lose the
            // part that says what went wrong. Stopped here, so blocking is free.
            Debug_Flush();

            s_collapse_failed = (uint8_t)((stray || WallMem_Overflowed()) ? 1u : 0u);
            if (s_collapse_failed) {
                // A U-turn with nothing to fold into means the walk did not
                // come back out the way it went in: a loop in the maze, or a
                // junction read wrongly. Either way the string is not a route.
                // Erase rather than save, so the next power-up explores again
                // instead of confidently driving a string that is not a path.
                Debug_P("*** NOT SAVING: the log did not collapse to a route.\r\n");
                Debug_P("    Either the maze has a loop (this algorithm needs"
                        " one with none),\r\n");
                Debug_P("    or a junction was misread. Run the explore leg"
                        " again.\r\n");
                WallMem_Erase();
            } else {
                Debug_P("route:\r\n");
                WallMem_Dump();
                if (WallMem_Save()) {
                    Debug_P("saved to EEPROM.\r\n");
                } else {
                    Debug_P("*** EEPROM WRITE FAILED -- the route is still in"
                            " RAM, so the button will\r\n");
                    Debug_P("    still run it, but a power cycle now loses"
                            " it.\r\n");
                }
            }
        } else {
            Debug_P("replayed ");
            Debug_Int((int32_t)WallMem_PlaybackIndex());
            Debug_P(" of ");
            Debug_Int((int32_t)WallMem_Count());
            Debug_P(" stored decisions.\r\n");
            Debug_Flush();
            // Run 2 is the end of the demo. Slow blink says finished, which is
            // not the same light as ready -- press again to re-run it.
            Panel_SetLed(LED_BLINK_SLOW);
            enter(WM_DONE_IDLE);
            break;
        }
        Debug_Flush();
        arm();          // LED on, waiting for the button and the second run
        break;

    case WM_DONE_IDLE:
        Motors_Stop();
        if (Panel_ButtonHeld()) {
            Debug_P("\r\nbutton held -- discarding the saved route\r\n");
            WallMem_Erase();
            WallMem_Reset();
            s_collapse_failed = 0;
            arm();
            break;
        }
        if (Panel_ButtonPressed()) arm();      // short press: run it again
        break;

    case WM_FAULT:
    default:
        Motors_Stop();
        Panel_SetLed(LED_BLINK_FAST);
        break;
    }
}

uint8_t Solver_State(void) { return (uint8_t)s_state; }
