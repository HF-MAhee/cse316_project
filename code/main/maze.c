#include "config.h"
#include <stdlib.h>
#include "maze.h"
#include "drive.h"
#include "turn.h"
#include "sonar.h"
#include "motors.h"
#include "heading.h"
#include "timer.h"
#include "debug.h"

static maze_state_t s_state = ST_STARTUP;
static uint32_t     s_state_entered = 0;
static uint32_t     s_leg_started   = 0;
static junction_t   s_pending       = J_CORRIDOR;
static turn_dir_t   s_pending_dir   = TURN_RIGHT;
static uint8_t      s_do_180        = 0;

// Debounce counters
static uint8_t s_open_l = 0, s_open_r = 0, s_block_f = 0, s_deadend = 0;

static uint8_t s_recover_begun = 0;

static void enter(maze_state_t st) {
    s_state = st;
    s_state_entered = millis();
    s_recover_begun = 0;   // any state change re-arms the recovery kickstart
}

static uint8_t in_state_for(uint32_t ms) {
    return ((millis() - s_state_entered) >= ms) ? 1 : 0;
}

// ---------------------------------------------------------------------------
//  Randomness
// ---------------------------------------------------------------------------
// No entropy source on an ATmega32, but the gyro's own noise floor is a real
// one: the low bits of the bias measurement differ every power-up.
void Maze_Init(void) {
    srand((unsigned int)(Gyro_GetOffset() ^ (int32_t)millis()));
    s_open_l = s_open_r = s_block_f = s_deadend = 0;
    s_leg_started = millis();
    enter(ST_STARTUP);
}

maze_state_t Maze_State(void) { return s_state; }

const char *Maze_StateName(void) {
    switch (s_state) {
        case ST_STARTUP:       return "STARTUP";
        case ST_DRIVING:       return "DRIVING";
        case ST_APPROACHING:   return "APPROACH";
        case ST_CONFIRM_EXIT:  return "CONFIRM_EXIT";
        case ST_STOPPING:      return "STOPPING";
        case ST_RECALIBRATING: return "RECAL";
        case ST_DECIDING:      return "DECIDING";
        case ST_TURNING:       return "TURNING";
        case ST_RECOVERING:    return "RECOVER";
        case ST_FINISHED:      return "FINISHED";
        default:               return "FAULT";
    }
}

// ---------------------------------------------------------------------------
//  Junction classification
// ---------------------------------------------------------------------------
// Debounced so a single bad ping cannot trigger a turn. Side openings need
// OPENING_CONFIRM agreeing samples; a dead end needs more, because a spurious
// U-turn is expensive to recover from.
static void update_debounce(void) {
    if (Sonar_IsOpen(SONAR_LEFT))  { if (s_open_l < 255) s_open_l++; } else s_open_l = 0;
    if (Sonar_IsOpen(SONAR_RIGHT)) { if (s_open_r < 255) s_open_r++; } else s_open_r = 0;
    if (Sonar_FrontBlocked())      { if (s_block_f < 255) s_block_f++; } else s_block_f = 0;

    if (Sonar_FrontBlocked() && !Sonar_IsOpen(SONAR_LEFT) && !Sonar_IsOpen(SONAR_RIGHT)) {
        if (s_deadend < 255) s_deadend++;
    } else {
        s_deadend = 0;
    }
}

static junction_t classify(void) {
    uint8_t left_open  = (s_open_l  >= OPENING_CONFIRM);
    uint8_t right_open = (s_open_r  >= OPENING_CONFIRM);
    uint8_t front_wall = (s_block_f >= OPENING_CONFIRM);

    if (s_deadend >= DEADEND_CONFIRM) return J_DEAD_END;

    if (!front_wall && left_open && right_open) return J_ALL_OPEN;      // exit?
    if (!front_wall && left_open  && !right_open) return J_FWD_OR_LEFT;
    if (!front_wall && !left_open && right_open)  return J_FWD_OR_RIGHT;
    if ( front_wall && left_open  && right_open)  return J_LEFT_OR_RIGHT;
    if ( front_wall && left_open  && !right_open) return J_FORCED_LEFT;
    if ( front_wall && !left_open && right_open)  return J_FORCED_RIGHT;

    return J_CORRIDOR;
}

// For the three "either/or" junctions, pick at random. A proper solving
// algorithm replaces only this function later.
static void choose(junction_t j, uint8_t *turn_now, turn_dir_t *dir) {
    *turn_now = 1;
    switch (j) {
        case J_FWD_OR_LEFT:
            if (rand() & 1) { *dir = TURN_LEFT; }
            else            { *turn_now = 0; }          // carry straight on
            break;
        case J_FWD_OR_RIGHT:
            if (rand() & 1) { *dir = TURN_RIGHT; }
            else            { *turn_now = 0; }
            break;
        case J_LEFT_OR_RIGHT:
            *dir = (rand() & 1) ? TURN_LEFT : TURN_RIGHT;
            break;
        case J_FORCED_LEFT:  *dir = TURN_LEFT;  break;
        case J_FORCED_RIGHT: *dir = TURN_RIGHT; break;
        default:             *turn_now = 0;     break;
    }
}

// ---------------------------------------------------------------------------
//  State machine
// ---------------------------------------------------------------------------
void Maze_Tick(int16_t gyro_rate) {
    junction_t j;

    switch (s_state) {

    case ST_STARTUP:
        if (in_state_for(STARTUP_DELAY_MS)) {
            Debug_Str("GO\r\n");
            Drive_Begin();
            s_leg_started = millis();
            enter(ST_DRIVING);
        }
        break;

    case ST_DRIVING:
        Drive_Tick(gyro_rate);
        update_debounce();
        j = classify();

        if (j == J_DEAD_END) {
            Debug_Str("DEAD END\r\n");
            s_do_180 = 1;
            Drive_Stop();
            enter(ST_STOPPING);
            break;
        }

        if (j == J_ALL_OPEN) {
            // Might be the exit -- but at a left-or-right T the two side
            // openings can appear a moment before the front wall closes in,
            // which looks identical. Drive on a short way and re-check.
            Debug_Str("all-open, confirming\r\n");
            enter(ST_CONFIRM_EXIT);
            break;
        }

        if (j != J_CORRIDOR) {
            uint8_t turn_now;
            choose(j, &turn_now, &s_pending_dir);
            if (turn_now) {
                s_pending = j;
                s_do_180  = 0;
                // Front-mounted sonar: the sensor is level with the opening
                // but the robot pivots about the axle, further back. Keep
                // driving so the axle -- not the nose -- reaches the opening.
                enter(ST_APPROACHING);
            } else {
                // Chose to continue straight; suppress re-triggering on this
                // same opening until it has passed out of view.
                s_open_l = s_open_r = 0;
            }
            break;
        }

        // Safety net: a wall the sonar never saw (angled or soft surfaces
        // reflect the pulse away and read as clear).
        if ((millis() - s_leg_started) > MAX_LEG_MS) {
            Debug_Str("leg timeout\r\n");
            s_do_180 = 0;
            s_pending_dir = TURN_RIGHT;
            Drive_Stop();
            enter(ST_STOPPING);
        }
        break;

    case ST_APPROACHING:
        // Keep centring while covering the offset distance.
        Drive_Tick(gyro_rate);
        // If the front wall closes in first, stop short of it instead.
        if (Sonar_IsValid(SONAR_FRONT) && Sonar_Latest(SONAR_FRONT) < FRONT_STOP_CM) {
            Debug_Str("front obstacle detected, stopping short\r\n");
            Drive_Stop();
            enter(ST_STOPPING);
            break;
        }
        if (in_state_for(APPROACH_TIME_MS)) {
            Drive_Stop();
            enter(ST_STOPPING);
        }
        break;

    case ST_CONFIRM_EXIT:
        Drive_Tick(gyro_rate);
        update_debounce();
        if (Sonar_FrontBlocked()) {
            // A wall appeared: this was a junction, not the exit. Re-evaluate
            // on the next DRIVING tick.
            Debug_Str("not exit, front wall\r\n");
            enter(ST_DRIVING);
            break;
        }
        if (in_state_for(EXIT_CONFIRM_MS)) {
            if (classify() == J_ALL_OPEN) {
                Debug_Str("MAZE COMPLETE\r\n");
                Drive_Stop();
                enter(ST_FINISHED);
            } else {
                enter(ST_DRIVING);
            }
        }
        break;

    case ST_STOPPING:
        Motors_Stop();
        // Let the chassis stop rocking before the gyro is sampled for bias.
        if (in_state_for(GYRO_SETTLE_MS)) enter(ST_RECALIBRATING);
        break;

    case ST_RECALIBRATING: {
        // Refresh the gyro bias while genuinely stationary. Thermal drift
        // over a long run would otherwise creep into every subsequent turn.
        uint8_t ok = Gyro_CalibrateQuick();
        Debug_Str(ok ? "recal ok " : "recal SKIPPED ");
        Debug_KV("off", Gyro_GetOffset());
        Debug_NL();
        Heading_Reset();
        Sonar_Flush();
        enter(ST_DECIDING);
        break;
    }

    case ST_DECIDING: {
        turn_result_t r;
        if (s_do_180) {
            Debug_Str("180\r\n");
            Turn_180(&r);
        } else {
            Debug_Str((s_pending_dir == TURN_RIGHT) ? "turn R\r\n" : "turn L\r\n");
            Turn_90(s_pending_dir, &r);
        }
        Debug_KV("ang10", r.achieved_tenths);
        Debug_KV("nudge", r.nudges_used);
        Debug_KV("to", r.timed_out);
        // A wrong-way pivot here is the expensive one: the maze solver thinks
        // it went right when it went left, and every position estimate after
        // this point is wrong.
        if (r.wrong_way) Debug_Str("WRONG WAY ");
        Debug_NL();
        enter(ST_RECOVERING);
        break;
    }

    case ST_TURNING:
        // Turns are executed synchronously inside ST_DECIDING; this state is
        // kept for clarity and future non-blocking rework.
        enter(ST_RECOVERING);
        break;

    case ST_RECOVERING:
        // The sonar filters were flushed after the pivot. Drive gyro-only
        // until they refill, so a half-empty filter cannot steer the robot.
        if (!s_recover_begun) {
            Drive_Begin();          // fresh kickstart: we are starting from rest
            s_recover_begun = 1;
        }
        Drive_Tick(gyro_rate);
        if (in_state_for(RECOVER_MS)) {
            s_open_l = s_open_r = s_block_f = s_deadend = 0;
            s_leg_started = millis();
            enter(ST_DRIVING);
        }
        break;

    case ST_FINISHED:
        Motors_Stop();
        break;

    case ST_FAULT:
    default:
        Motors_Stop();
        break;
    }
}
