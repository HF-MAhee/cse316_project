#ifndef TURN_H
#define TURN_H
#include <stdint.h>

typedef enum { TURN_LEFT = 0, TURN_RIGHT = 1 } turn_dir_t;

typedef struct {
    int32_t achieved_tenths;   // final measured rotation, tenths of a degree
    uint8_t nudges_used;
    uint8_t timed_out;
    uint8_t recal_ok;          // did the post-turn gyro recalibration succeed

    // Signed error after the FIRST arrival and settle, before any further
    // control pass, tenths of a degree. Positive = short of the target,
    // negative = past it. Near zero means the speed profile is doing its job;
    // a consistent sign means TURN_CTL_DECEL_DPS2 / TURN_CTL_MIN_DPS want
    // tuning (see config.h). nudges_used counts the extra control passes.
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
    // a degree, and whether it got inside TURN_DEADBAND_TENTHS at all.
    //
    // Without these, a turn that runs out of control passes while still out of
    // deadband reports achieved_tenths and timed_out=0 -- indistinguishable
    // from success. A measured run used all 5 nudges on 3 of 6 turns starting
    // ~40 degrees out, so the budget was one bad turn from running dry.
    int32_t final_error_tenths;
    uint8_t converged;

    // The same leftover error, but expressed in the NEXT leg's frame and in
    // raw LSB*ms: how far the chassis sits from the heading it should now
    // hold, positive = rotated LEFT of it. Turn_Execute() zeroes the heading
    // accumulator when it finishes, so without carrying this forward every
    // turn's residual is simply discarded and accumulates into the path.
    // Hand it straight to Drive_StraightHold() -- the direction sign is
    // already folded in here so call sites cannot get it backwards.
    int32_t residual_raw;
} turn_result_t;

// Blocking closed-loop pivot. Sonar is meaningless while rotating, so the
// caller should flush the sonar history afterwards.
void Turn_Execute(uint16_t degrees, turn_dir_t dir, turn_result_t *res);

void Turn_90(turn_dir_t dir, turn_result_t *res);

// Reverse direction. `dir` is the side the chassis ROTATES TOWARDS, and it
// matters physically, not just cosmetically: an in-place pivot is asymmetric.
// The FRONT corners swing PIVOT_RADIUS_CM from the axle into the side being
// turned towards, while the rear corners only reach about 10.6 cm out the
// other side. So a 180 needs roughly 6 cm MORE free space on the side it
// rotates into. In a dead end, pick the direction AWAY from the nearer wall.
// side_cm: what the sonar on the side it rotates towards read before the
// turn (SONAR_NO_ECHO if unknown). Used to tell "too close to echo" from
// "open" at the halfway point -- see uturn_clearance() in turn.c.
void Turn_180(turn_dir_t dir, uint16_t side_cm, turn_result_t *res);

#endif
