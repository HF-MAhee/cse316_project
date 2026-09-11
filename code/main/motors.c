#include "config.h"
#include <avr/io.h>
#include "motors.h"

void Motors_Init(void) {
    MOTOR_DIR_DDR |= (1 << LEFT_IN1_BIT)  | (1 << LEFT_IN2_BIT)
                   | (1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT);
    PWM_DDR       |= (1 << LEFT_PWM_BIT)  | (1 << RIGHT_PWM_BIT);

    // Timer1: 8-bit phase-correct PWM, prescaler 64.
    // 16 MHz / (64 * 2 * 255) ~= 490 Hz. OCR1A/OCR1B take 0..255.
    //   OCR1B -> PD4 -> OC1B -> LEFT
    //   OCR1A -> PD5 -> OC1A -> RIGHT
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << CS11) | (1 << CS10);

    OCR1A = 0;
    OCR1B = 0;
}

// The chassis will not move below MOTOR_MIN_PWM, so anything non-zero is
// pushed up to that floor. Returning 0 unchanged keeps a real stop possible.
static uint8_t clamp_pwm(uint8_t pwm) {
    if (pwm == 0) return 0;
    if (pwm < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
    if (pwm > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
    return pwm;
}

void Motors_SetLeft(motor_dir_t dir, uint8_t pwm) {
    switch (dir) {
        case DIR_FWD:
            MOTOR_DIR_PORT |=  (1 << LEFT_IN1_BIT);
            MOTOR_DIR_PORT &= ~(1 << LEFT_IN2_BIT);
            break;
        case DIR_REV:
            MOTOR_DIR_PORT &= ~(1 << LEFT_IN1_BIT);
            MOTOR_DIR_PORT |=  (1 << LEFT_IN2_BIT);
            break;
        default:
            MOTOR_DIR_PORT &= ~((1 << LEFT_IN1_BIT) | (1 << LEFT_IN2_BIT));
            pwm = 0;
            break;
    }
    OCR1B = clamp_pwm(pwm);
}

void Motors_SetRight(motor_dir_t dir, uint8_t pwm) {
    switch (dir) {
        case DIR_FWD:
            MOTOR_DIR_PORT |=  (1 << RIGHT_IN3_BIT);
            MOTOR_DIR_PORT &= ~(1 << RIGHT_IN4_BIT);
            break;
        case DIR_REV:
            MOTOR_DIR_PORT &= ~(1 << RIGHT_IN3_BIT);
            MOTOR_DIR_PORT |=  (1 << RIGHT_IN4_BIT);
            break;
        default:
            MOTOR_DIR_PORT &= ~((1 << RIGHT_IN3_BIT) | (1 << RIGHT_IN4_BIT));
            pwm = 0;
            break;
    }
    OCR1A = clamp_pwm(pwm);
}

void Motors_Forward(uint8_t left_pwm, uint8_t right_pwm) {
    Motors_SetLeft(DIR_FWD, left_pwm);
    Motors_SetRight(DIR_FWD, right_pwm);
}

void Motors_Pivot(uint8_t clockwise, uint8_t pwm) {
    if (clockwise) {
        Motors_SetLeft(DIR_FWD, pwm);
        Motors_SetRight(DIR_REV, pwm);
    } else {
        Motors_SetLeft(DIR_REV, pwm);
        Motors_SetRight(DIR_FWD, pwm);
    }
}

void Motors_Stop(void) {
    Motors_SetLeft(DIR_STOP, 0);
    Motors_SetRight(DIR_STOP, 0);
}
