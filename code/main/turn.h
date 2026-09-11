#ifndef TURN_H
#define TURN_H
#include <stdint.h>

typedef enum { TURN_LEFT = 0, TURN_RIGHT = 1 } turn_dir_t;

typedef struct {
    int32_t achieved_tenths;   // measured rotation, tenths of a degree
    uint8_t nudges_used;
    uint8_t timed_out;
    uint8_t recal_ok;          // did the post-turn gyro recalibration succeed
} turn_result_t;

// Blocking closed-loop pivot. Sonar is meaningless while rotating, so the
// caller should flush the sonar history afterwards.
void Turn_Execute(uint16_t degrees, turn_dir_t dir, turn_result_t *res);

void Turn_90(turn_dir_t dir, turn_result_t *res);
void Turn_180(turn_result_t *res);
#endif
