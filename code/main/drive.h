#ifndef DRIVE_H
#define DRIVE_H
#include <stdint.h>

typedef enum {
    CENTER_BOTH_WALLS = 0,   // differential centring (width-independent)
    CENTER_LEFT_ONLY  = 1,   // hold half-corridor from the left wall
    CENTER_RIGHT_ONLY = 2,   // hold half-corridor from the right wall
    CENTER_GYRO_ONLY  = 3    // no usable walls: fall back to heading hold
} center_mode_t;

// Full internal state of the last Drive_Tick(). Exposed so telemetry can show
// WHY a correction was chosen, not just its value -- the previous logs could
// not distinguish "wall term computed and small" from "wall term never ran".
typedef enum {
    BRANCH_NORMAL       = 0,
    BRANCH_ROCKING      = 1,   // wall term applied at reduced gain
    BRANCH_EMERG_L      = 2,   // hard steer right, left wall too close
    BRANCH_EMERG_R      = 3,   // hard steer left, right wall too close
    BRANCH_EMERG_RECOVER = 4   // steering alone didn't break wall contact --
                                // backing off and pivoting clear instead
} drive_branch_t;

typedef struct {
    int16_t error_cm;    // wall error the controller actually saw
    int16_t wall_term;   // proportional contribution
    int16_t gyro_term;   // derivative contribution
    int16_t corr;        // final, after clamping
    uint8_t pwm_l;       // what was actually written to the motors
    uint8_t pwm_r;
    uint8_t branch;      // drive_branch_t
    uint8_t l_ok;        // sonar validity that drove mode selection
    uint8_t r_ok;
} drive_debug_t;

const drive_debug_t *Drive_Debug(void);

void          Drive_Begin(void);            // kickstart + reset controller
void          Drive_Tick(int16_t gyro_rate);// one control step
void          Drive_Stop(void);

// BLOCKING straight drive for a fixed duration, held on the gyro alone (no
// sonar). This is the original firmware's straight-line autocorrect: P on
// accumulated heading error plus D on rate, so the chassis returns to the
// heading it started on instead of merely resisting rotation. Includes the
// breakaway kick, and integrates throughout so no rotation goes uncounted.
//
// start_offset_raw is where the chassis ALREADY sits relative to the heading
// it should hold, in raw LSB*ms, positive = rotated LEFT of it. Feed a turn's
// turn_result_t.residual_raw in here and the leg steers that leftover out,
// instead of every turn's error accumulating into the next leg.
//
// Returns the leg's NET heading change in raw LSB*ms (+ = net rotation left),
// so a caller running several legs and turns can account for the total
// rotation over the whole path -- a closed square must come to -360 degrees.
int32_t       Drive_StraightHold(uint8_t pwm, uint32_t ms, int32_t start_offset_raw);

// Same controller, but it services the sonar and stops as soon as the FRONT
// sensor reports a debounced obstacle, giving up after max_ms. Sets
// *blocked_out to 1 if it stopped on an obstacle, 0 if it timed out -- the
// caller must check, because "drove the whole way with nothing ahead" and
// "found a wall" need very different follow-up. Returns the net heading change
// like Drive_StraightHold().
int32_t       Drive_StraightUntilBlocked(uint8_t pwm, uint32_t max_ms,
                                         int32_t start_offset_raw,
                                         uint8_t *blocked_out);
center_mode_t Drive_Mode(void);
int16_t       Drive_LastCorrection(void);
#endif
