#ifndef MOTORS_H
#define MOTORS_H
#include <stdint.h>

typedef enum { DIR_FWD = 0, DIR_REV = 1, DIR_STOP = 2 } motor_dir_t;

void Motors_Init(void);

// Direction + speed for each side. Speeds are clamped into
// [MOTOR_MIN_PWM, MOTOR_MAX_PWM]; a speed of 0 means a true stop.
void Motors_SetLeft(motor_dir_t dir, uint8_t pwm);
void Motors_SetRight(motor_dir_t dir, uint8_t pwm);

// Both wheels forward at independent speeds (the straight-line case).
void Motors_Forward(uint8_t left_pwm, uint8_t right_pwm);

// In-place pivot. clockwise!=0 spins the chassis right (assuming
// left-forward + right-reverse == clockwise on your wiring; if the robot
// turns the wrong way, swap the direction at the CALL SITE, not here).
void Motors_Pivot(uint8_t clockwise, uint8_t pwm);

void Motors_Stop(void);
#endif
