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
center_mode_t Drive_Mode(void);
int16_t       Drive_LastCorrection(void);
#endif
