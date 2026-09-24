#ifndef TURN_H
#define TURN_H
#include <stdint.h>

typedef enum { TURN_LEFT = 0, TURN_RIGHT = 1 } turn_dir_t;

typedef struct {
    int32_t achieved_tenths;   // final measured rotation, tenths of a degree
    uint8_t nudges_used;
    uint8_t timed_out;
    uint8_t recal_ok;          // did the post-turn gyro recalibration succeed

    // Signed residual error measured right after the kick/sweep/brake/settle
    // phases, BEFORE any closed-loop nudging -- i.e. what the fixed
    // TURN_STOP_MARGIN_DEG early-stop actually produced this run, tenths of a
    // degree. Positive = undershot the target (coast didn't carry it far
    // enough, needs more rotation the same direction). Negative = overshot
    // (coast carried it past target, needs a reverse nudge). This is what to
    // watch to tell whether the "stop early, let it coast the rest" guess is
    // running consistently high or low.
    int32_t initial_error_tenths;

    // Largest |yaw rate| seen during the turn, raw LSB. At +/-500 dps the
    // gyro clips at 32767; a clipped sample integrates LOW, so the heading
    // under-reads and the robot physically overshoots while the code believes
    // it hit the target. Anything above ~25000 (~380 deg/s) means the kick or
    // nudge PWM is too high for this range.
    int32_t peak_rate;

    // 1 when the chassis rotated the OPPOSITE way to the one commanded.
    // execute_single() measures |heading|, so a wrong-way turn otherwise
    // completes and reports success -- easy to spot by eye on the bench, silent
    // navigation garbage in a maze run. Assumes positive gyro Z = turning LEFT
    // (so a TURN_RIGHT should accumulate NEGATIVE); if this reads 1 on every
    // turn while the robot visibly turns the right way, that convention is
    // inverted on your board, not the motors.
    uint8_t wrong_way;

    // How long after the motors cut the chassis was STILL rotating faster
    // than TURN_STILL_LSB, measured during the phase-4 settle, in ms.
    // If this lands at or near TURN_SETTLE_MS the settle is ending before the
    // chassis has stopped, so the closed-loop correction below it is reading a
    // heading that is still moving -- it nudges, coasts further, overshoots,
    // reverses, and burns its whole nudge budget oscillating. Well under
    // TURN_SETTLE_MS means the settle is long enough.
    int32_t coast_ms;

    // Signed error STILL REMAINING when the correction loop gave up, tenths of
    // a degree, and whether it got inside TURN_DEADBAND_DEG at all.
    //
    // Without these, a turn that exhausts TURN_MAX_NUDGES while still out of
    // deadband reports achieved_tenths and timed_out=0 -- indistinguishable
    // from success. A measured run used all 5 nudges on 3 of 6 turns starting
    // ~40 degrees out, so the budget was one bad turn from running dry.
    int32_t final_error_tenths;
    uint8_t converged;

    // Where the chassis ended up relative to the maze grid, tenths of a
    // degree, + = left of it. This is the number that says whether the robot
    // will drive off square -- achieved_tenths only says how far it rotated.
    int32_t grid_error_tenths;
} turn_result_t;

// Blocking closed-loop pivots onto the next maze-grid heading (see
// Heading_GridStep). Both recalibrate the gyro and flush the sonar afterwards:
// every filtered reading was taken pointing somewhere else.
void Turn_90(turn_dir_t dir, turn_result_t *res);

// Reverse direction. `dir` is the side the chassis ROTATES TOWARDS, and it
// matters physically, not just cosmetically: an in-place pivot is asymmetric.
// The FRONT corners swing ~17 cm from the axle into the side being
// turned towards, while the rear corners only reach about 10.6 cm out the
// other side. So a 180 needs roughly 6 cm MORE free space on the side it
// rotates into. In a dead end, pick the direction AWAY from the nearer wall.
void Turn_180(turn_dir_t dir, turn_result_t *res);

// Facing a wall: creep until the axle is CORRIDOR_HALF_CM from it, so a pivot
// here happens in the middle of the cell. Does nothing if no wall is within
// UTURN_CENTRE_MAX_CM ahead.
void Turn_CentreOnWall(void);

#endif
