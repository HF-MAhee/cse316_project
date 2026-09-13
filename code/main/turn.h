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
} turn_result_t;

// Blocking closed-loop pivot. Sonar is meaningless while rotating, so the
// caller should flush the sonar history afterwards.
void Turn_Execute(uint16_t degrees, turn_dir_t dir, turn_result_t *res);

void Turn_90(turn_dir_t dir, turn_result_t *res);
void Turn_180(turn_result_t *res);
#endif
