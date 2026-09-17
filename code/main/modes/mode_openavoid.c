#include "config.h"
#include <avr/io.h>
#include "mode.h"
#include "telemetry.h"
#include "resetlog.h"
#include "timer.h"
#include "mpu6050.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "maze.h"
#include "debug.h"
#include "power.h"
#include "gyrodiag.h"

// Build mode 9 -- obstacle avoidance with no corridor at all: forward on gyro
// heading hold, stop at an obstacle, check clearance, pivot, check again.
void Mode_PreGyro(void) { }
// The original printed the STANDARD CSV header here, even though this mode
// never reaches the control loop and so never emits a CSV row. Preserved
// verbatim: changing it is a fix, not a refactor. Swap to
// Telemetry_NoneHeader() if you would rather the header stopped promising
// rows that never arrive.
void Mode_Header(void) { Telemetry_Header(); }

void Mode_Begin(void) {
    // Runs to completion here and never returns -- this mode owns the
    // whole run, so there is nothing for the control loop to do.
    {
        turn_result_t r;
        uint8_t  blocked = 0, i;
        uint16_t front_cm, side_cm;

        Debug_P("# OBSTACLE AVOID -- open space, no corridor needed.\r\n");
        Debug_P("#   forward on gyro heading hold -> stop at obstacle ->\r\n");
        Debug_P("#   check clearance -> right 90 -> check again -> one leg\r\n");
        Debug_Flush();
        Debug_P("# needs ");
        Debug_Int((int32_t)AVOID_TURN_CLEARANCE_CM);
        Debug_P(" cm clear to the RIGHT of the obstacle, and stops ");
        Debug_Int((int32_t)AVOID_PIVOT_CLEARANCE_CM);
        Debug_P("+ cm short of it\r\n");
        Debug_Flush();

        Timer_WaitMs(STARTUP_DELAY_MS);

        // Prime the filters. Medians need HIST samples and the front vote
        // window needs history before either reading means anything -- without
        // this the clearance checks below would run on empty history.
        for (i = 0; i < 12; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }

        // ---- leg 1: forward until something is ahead ----------------------
        Debug_P("[1] driving forward, watching the front sensor\r\n");
        Debug_Flush();
        Drive_StraightUntilBlocked(SQUARE_TEST_PWM, AVOID_APPROACH_MAX_MS,
                                   0, &blocked);
        front_cm = Sonar_Median(SONAR_FRONT);

        if (!blocked) {
            Debug_P("    nothing found before the time limit -- stopping.\r\n");
            Debug_P("OBSTACLE AVOID ABORTED\r\n");
            Debug_Flush();
            Motors_Stop();
            for (;;) { }
        }
        Debug_P("    OBSTACLE found, stopped with ");
        Debug_Int((int32_t)front_cm);
        Debug_P(" cm showing on the front sensor\r\n");
        Debug_Flush();

        // ---- clearance gate: is there room to turn INTO? ------------------
        // Checked before the pivot so a blocked right side costs nothing but a
        // message. The reading is from the RIGHT sensor, which is pointing
        // where the robot is about to travel.
        Timer_WaitMs(GYRO_SETTLE_MS);
        for (i = 0; i < 9; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }
        side_cm  = Sonar_Median(SONAR_RIGHT);
        front_cm = Sonar_Median(SONAR_FRONT);

        // Room to pivot without clipping the obstacle. Re-measured AFTER the
        // stop on purpose: drive.c has no active brake, and a previous run
        // coasted 17cm past its stop point, so the distance that triggered the
        // stop is not the distance the robot actually ended up at.
        Debug_P("[2a] room to pivot: ");
        Debug_Int((int32_t)front_cm);
        Debug_P(" cm after coasting (need ");
        Debug_Int((int32_t)AVOID_PIVOT_CLEARANCE_CM);
        Debug_P(")\r\n");
        if (Sonar_IsTooClose(SONAR_FRONT) ||
            (Sonar_IsValid(SONAR_FRONT) && front_cm < AVOID_PIVOT_CLEARANCE_CM)) {
            Debug_P("    TOO CLOSE to pivot -- the front corners would clip it\r\n");
            Debug_P("    on the way round. Back the robot off and re-run.\r\n");
            Debug_P("OBSTACLE AVOID ABORTED\r\n");
            Debug_Flush();
            Motors_Stop();
            for (;;) { }
        }
        Debug_Flush();

        Debug_P("[2b] clearance to the right: ");
        if (Sonar_Latest(SONAR_RIGHT) == SONAR_NO_ECHO) {
            Debug_P("beyond sensor range -- open, good\r\n");
        } else {
            Debug_Int((int32_t)side_cm);
            Debug_P(" cm (need ");
            Debug_Int((int32_t)AVOID_TURN_CLEARANCE_CM);
            Debug_P(")\r\n");
            if (!Sonar_IsValid(SONAR_RIGHT) ||
                side_cm < AVOID_TURN_CLEARANCE_CM) {
                Debug_P("    NOT ENOUGH ROOM to the right. Refusing to turn --\r\n");
                Debug_P("    the second leg would drive into it. Move the\r\n");
                Debug_P("    obstacle further from the side wall and re-run.\r\n");
                Debug_P("OBSTACLE AVOID ABORTED\r\n");
                Debug_Flush();
                Motors_Stop();
                for (;;) { }
            }
        }
        Debug_Flush();

        // ---- pivot --------------------------------------------------------
        Debug_P("[3] turning right 90\r\n");
        Debug_Flush();
        Turn_90(TURN_RIGHT, &r);
        Debug_P("    ");
        Debug_KVF("ang10", r.achieved_tenths);
        Debug_KVF("fin10", r.final_error_tenths);
        Debug_KVF("conv", r.converged);
        Debug_KVF("wrong", r.wrong_way);
        Debug_NL();
        Debug_Flush();

        // ---- final gate: the path actually ahead now ----------------------
        // Turn_Execute() flushed the sonar, so this has to re-prime. This is
        // the authoritative check: the front sensor is now pointing down the
        // path the robot is about to drive, rather than inferring it from a
        // side reading taken before the pivot.
        for (i = 0; i < 12; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }
        front_cm = Sonar_Median(SONAR_FRONT);
        Debug_P("[4] path ahead after the turn: ");
        if (Sonar_Latest(SONAR_FRONT) == SONAR_NO_ECHO) {
            Debug_P("clear beyond sensor range\r\n");
        } else {
            Debug_Int((int32_t)front_cm);
            Debug_P(" cm\r\n");
            if (Sonar_IsValid(SONAR_FRONT) && front_cm < AVOID_TURN_CLEARANCE_CM) {
                Debug_P("    TOO CLOSE to drive the full leg. Stopping here\r\n");
                Debug_P("    rather than running into it.\r\n");
                Debug_P("OBSTACLE AVOID ABORTED\r\n");
                Debug_Flush();
                Motors_Stop();
                for (;;) { }
            }
        }
        Debug_Flush();

        // ---- leg 2 --------------------------------------------------------
        Debug_P("[5] driving the final leg\r\n");
        Debug_Flush();
        Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS, r.residual_raw);

        Debug_P("OBSTACLE AVOID DONE\r\n");
        Debug_Flush();
        Motors_Stop();
        for (;;) { }
    }
}
void Mode_Tick(const tick_ctx_t *t) { (void)t; }

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
