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

// Build mode 5 -- four timed legs and four 90 degree turns, no corridor needed.
// Tests drive and turn capability in open space.
void Mode_PreGyro(void) { }
void Mode_Header(void) { Telemetry_NoneHeader(); }

void Mode_Begin(void) {
    // Runs to completion here and never returns -- this mode owns the
    // whole run, so there is nothing for the control loop to do.
    {
        uint8_t leg;
        int32_t carry = 0;      // previous turn's leftover, in this leg's frame
        int32_t total_raw = 0;  // rotation over the WHOLE path, legs and turns

        // Self-describing header: every tunable that shapes this run, so the
        // log can be analysed later without having to guess the build.
        //
        // Debug_Flush() after EVERY line, not once at the end. tx_push() drops
        // when the ring buffer is full, so the loss happens during the pushes
        // -- a flush afterwards is far too late. A measured run lost 213 bytes
        // of this header with a single trailing flush.
        Debug_P("# SQUARE TEST  sides=");   Debug_Int(SQUARE_SIDES);
        Debug_P(" pwm=");                   Debug_Int(SQUARE_TEST_PWM);
        Debug_P(" legms=");                 Debug_Int(SQUARE_LEG_MS);
        Debug_NL();                           Debug_Flush();
        Debug_P("# hold: tick=");           Debug_Int(HOLD_TICK_MS);
        Debug_P(" kp=");                    Debug_Int(HOLD_KP_NUM);
        Debug_P("/");                       Debug_Int(HOLD_KP_DEN);
        Debug_P(" kd=");                    Debug_Int(HOLD_KD_NUM);
        Debug_P("/");                       Debug_Int(HOLD_KD_DEN);
        Debug_NL();                           Debug_Flush();
        Debug_P("# lim: maxcorr=");         Debug_Int(WALL_MAX_CORRECTION);
        Debug_P(" pwmfloor=");              Debug_Int(MOTOR_MIN_PWM);
        Debug_P(" pwmceil=");               Debug_Int(MOTOR_MAX_PWM);
        Debug_NL();                           Debug_Flush();
        Debug_P("# turn: margin=");         Debug_Int(TURN_STOP_MARGIN_DEG);
        Debug_P(" deadband=");              Debug_Int(TURN_DEADBAND_DEG);
        Debug_P(" settle=");                Debug_Int(TURN_SETTLE_MS);
        Debug_P(" pwm=");                   Debug_Int(TURN_PWM);
        Debug_NL();                           Debug_Flush();
        Debug_P("# gyro: lsbms_per_deg=");  Debug_Int(GYRO_LSB_MS_PER_DEGREE);
        Debug_NL();                           Debug_Flush();
        Debug_P("# L,ms,err10,rate,corr,pwmL,pwmR  <- heading-hold sample\r\n");
        Debug_Flush();
        Debug_P("# S,ms,rate,hdg10                 <- turn sample\r\n");
        Debug_Flush();

#if SQUARE_TRACE_TURNS
        Turn_SampleTrace(1);
#endif
        Timer_WaitMs(STARTUP_DELAY_MS);

        for (leg = 0; leg < SQUARE_SIDES; leg++) {
            turn_result_t r;
            int32_t net_raw;

            Debug_P("=== leg "); Debug_Int(leg + 1); Debug_P(" forward ===\r\n");
            // Gyro heading hold, NOT open-loop. Two motors at equal PWM never
            // track straight (gearbox, tyre and friction mismatch), which is
            // why the original firmware had a straight-line autocorrect at
            // all. `carry` feeds the previous turn's residual in so the leg
            // steers it out rather than baking it into the path.
            net_raw    = Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS, carry);
            total_raw += net_raw;
            Timer_WaitMs(SQUARE_TURN_SETTLE_MS);

            Debug_P("=== leg "); Debug_Int(leg + 1); Debug_P(" turn ===\r\n");
            Turn_90(TURN_RIGHT, &r);
            // A right turn rotates negative, so subtract it from the running
            // total: four clean corners plus four straight legs must land on
            // -3600 tenths.
            total_raw -= (int32_t)r.achieved_tenths * GYRO_LSB_MS_PER_DEGREE / 10;

            Debug_P("  TURN ");
            Debug_KVF("ang10",  r.achieved_tenths);
            Debug_KVF("err10",  r.initial_error_tenths);
            Debug_KVF("fin10",  r.final_error_tenths);
            Debug_KVF("conv",   r.converged);
            Debug_KVF("nudges", r.nudges_used);
            Debug_KVF("peak",   r.peak_rate);
            Debug_KVF("coastms", r.coast_ms);
            Debug_KVF("wrong",  r.wrong_way);
            Debug_KVF("recal",  r.recal_ok);
            Debug_KVF("to",     r.timed_out);
            Debug_NL();
            Debug_P("  CUM ");
            Debug_KVF("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
            Debug_NL();

            // Hand this turn's leftover to the next leg instead of discarding
            // it -- Turn_Execute() zeroes the accumulator, so without this the
            // residual from every corner accumulates into the square.
            carry = r.residual_raw;
        }

        // No end-of-run table: the live HOLD/TURN/CUM lines above already carry
        // every number, and buffering them into arrays to replay at the end
        // pushed ~250 bytes into a 192-byte ring buffer in one burst, which
        // came back garbled and untrustworthy.
        //
        // This is the one number worth restating: whether the square closed in
        // HEADING. -3600 tenths is a perfect four-corner circuit. Position can
        // still be off even at -3600, since heading hold does not correct
        // sideways displacement and battery sag shortens the later legs.
        Debug_NL();
        Debug_Flush();
        Debug_P("SQUARE TOTAL ");
        Debug_KVF("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
        Debug_KVF("ideal10", -3600L);
        Debug_KVF("drop", Debug_Dropped());
        Debug_NL();
        Debug_Flush();
        Debug_P("SQUARE TEST DONE\r\n");
        Debug_Flush();
        Motors_Stop();
        for (;;) { }
    }
}
void Mode_Tick(const tick_ctx_t *t) { (void)t; }

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
