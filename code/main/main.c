#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "i2c.h"
#include "mpu6050.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "debug.h"
#include "power.h"
#include "resetlog.h"
#include "panel.h"
#include "solver.h"
#include "telemetry.h"

// ============================================================================
//  THE RUNNER.
//
//  Brings the hardware up in a safe order, reports what the last reset did,
//  calibrates the gyro, then runs a fixed-rate control tick forever. All maze
//  behaviour lives in solver.c; this file only sequences it.
// ============================================================================

int main(void) {
    uint32_t next_tick, run_start;
    static uint16_t overruns = 0;

    // MOTORS OFF FIRST -- before the UART, the timer, anything.
    //
    // A reset does NOT stop the motors. It makes every port pin a high-Z input,
    // so the L298N's direction inputs float and its last commanded state can
    // persist: the chassis keeps driving, or keeps pivoting, until firmware
    // takes the pins back. That is the "kept rotating after the turn" symptom
    // seen in the early dead-end logs -- the MCU browned out mid-pivot and the
    // motors simply carried on.
    //
    // Nothing here depends on any other subsystem, so it costs nothing to make
    // it the first thing that happens.
    Motors_Init();
    Motors_Stop();

    // Straight after the motors: the LED is a status light and a floating pin
    // is not a status. This also arms the button's pull-up before anything can
    // read it.
    Panel_Init();

    Debug_Init();
    Timer_Init();          // before sei() so millis() is live immediately
    I2C_Init();
    Sonar_Init();
    Power_Init();
    sei();

    Debug_P("\r\n=== AGV maze solver ===\r\n");
    ResetLog_Report();

    // Idle rail reading, taken before anything draws current. This is the
    // baseline every later sag figure is measured against, so it is worth a
    // line of its own.
    Debug_KVF("VCC idle mV", (int32_t)Power_VccMv());
    Debug_P(" (bandgap-derived: trust the CHANGE, not the absolute)\r\n");
    Debug_Flush();

    MPU6050_Init();
    Debug_P("calibrating gyro, hold still...\r\n");
    if (!Gyro_CalibrateFull()) {
        // A rejected calibration means the bias was measured while the chassis
        // was moving, so the heading zero -- and therefore every turn and every
        // straight-line correction in the run -- is built on a wrong number.
        Debug_P("*** GYRO CALIBRATION NOT VALIDATED: the chassis would not"
                " hold still.\r\n");
        Debug_P("    The offsets below were averaged DURING MOTION, so the"
                " heading\r\n");
        Debug_P("    zero is wrong and every turn this run inherits the"
                " error.\r\n");
        Debug_P("    Let the robot settle and restart before trusting"
                " anything.\r\n");
        Debug_Flush();
    }
    Debug_KVF("offZ", Gyro_GetOffset());
    Debug_KVF("offX", Gyro_GetOffsetX());
    Debug_KVF("offY", Gyro_GetOffsetY());
    Debug_NL();

    Solver_Init();

    next_tick = millis();
    run_start = millis();

    Solver_Begin();

    for (;;) {
        gyro_xyz_t g;
        tick_ctx_t ctx;
        uint32_t tick_start;

        // ---- fixed control tick -----------------------------------------
        if ((int32_t)(millis() - next_tick) < 0) continue;
        next_tick += CONTROL_TICK_MS;
        tick_start = millis();
        // If we fell badly behind (a long sonar timeout, say), re-base rather
        // than firing a burst of catch-up ticks with no spacing.
        if ((int32_t)(millis() - next_tick) > (int32_t)(CONTROL_TICK_MS * 3)) {
            next_tick = millis() + CONTROL_TICK_MS;
        }

        // ---- sensors -----------------------------------------------------
        MPU6050_ReadAll(&g);
        Motion_Update(&g);                     // pitch/roll -> rocking flag
        Heading_Add(g.z, CONTROL_TICK_MS);

        Sonar_Task();                          // exactly one ping per tick

        // Supply rail, every tick. The dip that resets the MCU lasts a few
        // milliseconds, so anything slower simply never observes it.
        Power_Task();

        // Button debounce and LED blink, at the same fixed rate everything
        // else runs at -- BUTTON_DEBOUNCE_TICKS is counted in these ticks.
        Panel_Task();

        ctx.rate      = Gyro_Rate(g.z);
        ctx.gx        = g.x;
        ctx.gy        = g.y;
        ctx.now       = tick_start;
        ctx.run_start = run_start;
        ctx.overruns  = overruns;

        // ---- behaviour ---------------------------------------------------
        Solver_Tick(&ctx);

        // Measure how long the tick's real work took. If this exceeds the
        // budget the control loop is no longer running at a fixed rate, which
        // silently invalidates the gyro integration and the PD tuning.
        if ((millis() - tick_start) > TICK_OVERRUN_WARN_MS) {
            if (overruns < 0xFFFF) overruns++;
        }
        ctx.overruns = overruns;

        // Reporting, after the deadline measurement, so turning the trace up
        // cannot itself manufacture overruns.
        Telemetry_Tick(&ctx);

    }
}
