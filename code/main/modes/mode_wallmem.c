#include "config.h"
#include <avr/io.h>
#include "mode.h"
#include "telemetry.h"
#include "timer.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "wallmem.h"
#include "debug.h"

// ============================================================================
//  Build mode: WALL FOLLOWER WITH MEMORY.
//
//  RUN 1 (explore)  left-hand rule, L > F > R > U. One byte logged per
//                   decision point. On reaching the exit the log is collapsed
//                   -- every dead-end excursion folds into the single turn it
//                   was equivalent to -- and the result is written to EEPROM.
//
//  RUN 2 (speed)    power the robot up again in the same start cell facing the
//                   same way. The saved route is loaded and replayed. Every
//                   junction is checked against the signature that was stored
//                   for it before its turn is acted on.
//
//  The two runs are deliberately separate power-ups. That is not a limitation
//  worked around, it is the point: the route lives in EEPROM, so the brown-outs
//  this chassis suffers cannot lose it, and the operator gets a clean reset
//  between the exploring run and the fast one.
//
//  To explore again: set WALLMEM_FORCE_EXPLORE to 1 in config.h and rebuild.
// ============================================================================

typedef enum {
    WM_STARTUP = 0,
    WM_DRIVING,        // centring forward, watching for a junction
    WM_APPROACH,       // junction seen; drive on so the AXLE reaches it
    WM_CONFIRM_EXIT,   // all three open -- exit, or a 4-way seen early?
    WM_STOPPING,
    WM_RECAL,
    WM_DECIDE,         // execute the pivot
    WM_RECOVER,        // gyro-only while the sonar filters refill
    WM_DONE,
    WM_FAULT
} wm_state_t;

static wm_state_t s_state = WM_STARTUP;
static uint32_t   s_entered = 0;
static uint32_t   s_leg_started = 0;
static uint32_t   s_run_started = 0;

static uint8_t    s_exploring = 1;      // 0 = replaying a saved route
static uint8_t    s_degraded  = 0;      // replay failed; finishing on left-hand
static uint8_t    s_pending_turn = WM_TURN_F;

// Debounce, same shape as the maze solver's: a single bad ping must never
// trigger a turn, and a spurious U-turn is the expensive one.
static uint8_t s_open_l = 0, s_open_r = 0, s_block_f = 0, s_deadend = 0;
static uint8_t s_recover_begun = 0;

// Run statistics -- this is what the demo is measured with. "segments" counts
// junction-to-junction runs, not 40 cm cells: the robot has no encoders, so it
// genuinely does not know how many cells it crossed, and a number that looked
// like a cell count would be a number nobody could trust.
static uint16_t s_segments = 0, s_turns90 = 0, s_turns180 = 0;
static uint8_t  s_finished = 0;

static void enter(wm_state_t st) {
    s_state = st;
    s_entered = millis();
    s_recover_begun = 0;
}
static uint8_t in_state_for(uint32_t ms) { return ((millis() - s_entered) >= ms) ? 1 : 0; }

static void clear_debounce(void) { s_open_l = s_open_r = s_block_f = s_deadend = 0; }

static void update_debounce(void) {
    if (Sonar_IsOpen(SONAR_LEFT))  { if (s_open_l  < 255) s_open_l++;  } else s_open_l  = 0;
    if (Sonar_IsOpen(SONAR_RIGHT)) { if (s_open_r  < 255) s_open_r++;  } else s_open_r  = 0;
    if (Sonar_FrontBlocked())      { if (s_block_f < 255) s_block_f++; } else s_block_f = 0;

    if (Sonar_FrontBlocked() && !Sonar_IsOpen(SONAR_LEFT) && !Sonar_IsOpen(SONAR_RIGHT)) {
        if (s_deadend < 255) s_deadend++;
    } else {
        s_deadend = 0;
    }
}

// The three bits everything downstream is decided from. Note that FORWARD is
// reported as OPEN, i.e. the complement of "front blocked" -- wallmem.h stores
// openness, not obstruction, so that the signature is directly comparable with
// what the classifier saw in the other run.
static void read_openness(uint8_t *f, uint8_t *l, uint8_t *r) {
    *l = (uint8_t)((s_open_l  >= OPENING_CONFIRM) ? 1u : 0u);
    *r = (uint8_t)((s_open_r  >= OPENING_CONFIRM) ? 1u : 0u);
    *f = (uint8_t)((s_block_f >= OPENING_CONFIRM) ? 0u : 1u);
}

// Rotate AWAY from the nearer wall. The front corners sweep
// PIVOT_FRONT_RADIUS_CM into the side being turned towards while the rear
// corners only reach ~10.6 cm the other way, so the turning side needs about
// 6 cm more room. See turn.h.
static turn_dir_t pick_180_dir(void) {
    uint16_t l_cm = Sonar_Median(SONAR_LEFT);
    uint16_t r_cm = Sonar_Median(SONAR_RIGHT);
    uint8_t  l_ok = Sonar_IsValid(SONAR_LEFT);
    uint8_t  r_ok = Sonar_IsValid(SONAR_RIGHT);

    if (l_ok && r_ok) {
        if (l_cm > r_cm && (uint16_t)(l_cm - r_cm) >= DEADEND_DECIDE_MARGIN_CM) return TURN_LEFT;
        if (r_cm > l_cm && (uint16_t)(r_cm - l_cm) >= DEADEND_DECIDE_MARGIN_CM) return TURN_RIGHT;
    } else if (l_ok) {
        return TURN_RIGHT;      // only the left wall is visible -- turn away
    } else if (r_ok) {
        return TURN_LEFT;
    }
    return DEADEND_TIE_DIR;
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

static void report_run(void) {
    Debug_P("\r\n---- run summary ----\r\n");
    Debug_KVF("mode", (int32_t)(s_exploring ? 1 : 2));
    Debug_KVF("segments", (int32_t)s_segments);
    Debug_KVF("turn90", (int32_t)s_turns90);
    Debug_KVF("turn180", (int32_t)s_turns180);
    Debug_KVF("ms", (int32_t)(millis() - s_run_started));
    Debug_KVF("degraded", (int32_t)s_degraded);
    Debug_NL();
    Debug_Flush();
}

// ===========================================================================
//  Entry points
// ===========================================================================
void Mode_PreGyro(void) { }

void Mode_Header(void) {
    Debug_P("\r\n=== MODE wallmem: wall follower with memory ===\r\n");

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

void Mode_Begin(void) {
    clear_debounce();
    s_segments = 0;
    s_turns90  = 0;
    s_turns180 = 0;
    s_finished = 0;
    s_degraded = 0;
    WallMem_RewindPlayback();
    s_leg_started = millis();
    s_run_started = millis();
    enter(WM_STARTUP);
}

void Mode_Tick(const tick_ctx_t *t) {
    int16_t rate = t->rate;
    uint8_t f, l, r;

    switch (s_state) {

    case WM_STARTUP:
        if (in_state_for(STARTUP_DELAY_MS)) {
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
            f = 0; l = 0; r = 0;
        } else if (s_deadend >= DEADEND_CONFIRM) {
            // A confirmed dead end is a decision point like any other: the
            // left-hand rule answers U, and the replay answers whatever was
            // stored -- which after collapsing is never U, so run 2 never
            // enters a dead end at all.
            f = 0; l = 0; r = 0;
        } else if (f && l && r) {
            // Everything open. That is the exit -- or a T junction whose front
            // wall has not closed in yet, which looks identical from here.
            Debug_P("all-open, confirming\r\n");
            enter(WM_CONFIRM_EXIT);
            break;
        } else if (!WallMem_IsDecision(f, l, r)) {
            break;      // plain corridor: drive on, record nothing, consume nothing
        }

        s_pending_turn = decide_turn(f, l, r);
        if (s_state == WM_FAULT) break;          // decide_turn halted us

        Debug_P("junction ");
        Debug_StrP(WallMem_TypeName(f, l, r));
        Debug_P(" -> ");
        Debug_StrP(WallMem_TurnName(s_pending_turn));
        Debug_P("\r\n");

        if (s_pending_turn == WM_TURN_F) {
            // Straight on. Suppress re-triggering on this same opening until
            // it has passed out of view, exactly as the maze solver does.
            s_open_l = s_open_r = 0;
            break;
        }
        if (s_pending_turn == WM_TURN_U) {
            Drive_Stop();
            enter(WM_STOPPING);
            break;
        }
        // The sonar sits ahead of the axle, so keep driving until the AXLE --
        // not the nose -- is level with the opening.
        enter(WM_APPROACH);
        break;

    case WM_APPROACH:
        Drive_Tick(rate);
        if (Sonar_IsValid(SONAR_FRONT) && Sonar_Latest(SONAR_FRONT) < FRONT_STOP_CM) {
            Debug_P("front wall close, stopping short\r\n");
            Drive_Stop();
            enter(WM_STOPPING);
            break;
        }
        if (in_state_for(APPROACH_TIME_MS)) {
            Drive_Stop();
            enter(WM_STOPPING);
        }
        break;

    case WM_CONFIRM_EXIT:
        // The distinction between "the exit" and "a junction whose front wall
        // has not closed in yet" is made HERE, and it is made by driving: keep
        // going, and see whether a wall turns up. A junction produces one
        // within EXIT_FALSE_WINDOW_CM; the exit never does.
        //
        // Bailing on the RAW front reading rather than the debounced one is
        // deliberate. It returns to WM_DRIVING at the same front distance at
        // which the junction would have been classified anyway, so the detour
        // costs one control tick and no extra travel -- the approach timing
        // downstream is unaffected.
        Drive_Tick(rate);
        update_debounce();
        if (Sonar_FrontBlocked()) {
            // How far into the window the wall appeared. This is the margin
            // nobody can compute from a datasheet: if it ever creeps close to
            // EXIT_CONFIRM_CM on real cardboard, raise EXIT_CONFIRM_MARGIN_CM
            // before it costs a run.
            Debug_P("not the exit -- front wall at ");
            Debug_Int((int32_t)(((millis() - s_entered) * TRAVEL_SPEED_CMS) / 1000UL));
            Debug_P(" cm of ");
            Debug_Int((int32_t)EXIT_CONFIRM_CM);
            Debug_P(" cm\r\n");
            enter(WM_DRIVING);
            break;
        }
        if (in_state_for(EXIT_CONFIRM_MS)) {
            read_openness(&f, &l, &r);
            if (f && l && r) {
                Drive_Stop();
                enter(WM_DONE);
            } else {
                enter(WM_DRIVING);
            }
        }
        break;

    case WM_STOPPING:
        Motors_Stop();
        if (in_state_for(GYRO_SETTLE_MS)) enter(WM_RECAL);
        break;

    case WM_RECAL:
        // Refresh the gyro bias while genuinely stationary; thermal drift over
        // a long explore run otherwise creeps into every later turn.
        if (!Gyro_CalibrateQuick()) Debug_P("recal SKIPPED\r\n");
        Heading_Reset();
        Sonar_Flush();
        enter(WM_DECIDE);
        break;

    case WM_DECIDE: {
        turn_result_t res;
        if (s_pending_turn == WM_TURN_U) {
            Turn_180(pick_180_dir(), &res);
            s_turns180++;
        } else {
            Turn_90((s_pending_turn == WM_TURN_L) ? TURN_LEFT : TURN_RIGHT, &res);
            s_turns90++;
        }
        Debug_KVF("ang10", res.achieved_tenths);
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
        if (!s_recover_begun) { Drive_Begin(); s_recover_begun = 1; }
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

            if (stray || WallMem_Overflowed()) {
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
                    Debug_P("POWER-CYCLE, put the robot back in the START cell"
                            " facing in, and run 2 will replay this.\r\n");
                } else {
                    Debug_P("*** EEPROM WRITE FAILED -- run 2 would explore"
                            " again.\r\n");
                }
            }
        } else {
            Debug_P("replayed ");
            Debug_Int((int32_t)WallMem_PlaybackIndex());
            Debug_P(" of ");
            Debug_Int((int32_t)WallMem_Count());
            Debug_P(" stored decisions.\r\n");
        }
        Debug_Flush();
        break;

    case WM_FAULT:
    default:
        Motors_Stop();
        break;
    }
}

void Mode_Telemetry(const tick_ctx_t *t) { Telemetry_Tick(t); }
