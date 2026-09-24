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
#include "mode.h"

// ============================================================================
//  THE RUNNER.
//
//  This file used to hold every build mode behind a #if chain, and had grown
//  past 1400 lines: adding a mode meant editing four separate conditionals in
//  three functions, and a mode's helpers sat hundreds of lines from the code
//  that called them. Modes now live one-per-file under modes/, and the Makefile
//  compiles exactly one of them:
//
//      make MODE=maze             build the full solver
//      make MODE=deadend flash    build the dead-end run and flash it
//      make list-modes            show every mode with a one-line description
//
//  What is left here is only the part that is the same for every mode: bring
//  the hardware up in a safe order, report what the last reset did, calibrate,
//  then run a fixed-rate control tick forever. Everything mode-specific reaches
//  it through the five entry points in mode.h.
// ============================================================================

int main(void) {
    uint32_t next_tick, run_start;
    static uint16_t overruns = 0;

    // MOTORS OFF FIRST -- before the UART, the timer, anything.
    //
    // A reset does NOT stop the motors. It makes every port pin a high-Z input,
    // so the L298N's direction inputs float and its last commanded state can
    // persist: the chassis keeps driving, or keeps pivoting, until firmware
    // takes the pins back. That is the "kept rotating after the turn" and "kept
    // rotating 360" symptom in the Mode 10 logs -- the MCU browned out mid-
    // pivot and the motors simply carried on.
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

    // The MODE the image was built with, straight from the Makefile. This used
    // to say "AGV maze solver" whichever mode was flashed, which is worse than
    // useless: flashing the wrong image is the single easiest mistake to make
    // with this project, and the one line of output that could have caught it
    // was claiming to be something else. Now the first line names the truth.
    Debug_P("\r\n=== AGV firmware -- MODE=" BUILD_MODE_NAME " ===\r\n");
    ResetLog_Report();

    // Idle rail reading, taken before anything draws current. This is the
    // baseline every later sag figure is measured against, so it is worth a
    // line of its own.
    Debug_KVF("VCC idle mV", (int32_t)Power_VccMv());
    Debug_P(" (bandgap-derived: trust the CHANGE, not the absolute)\r\n");
    Debug_Flush();

    // Before MPU6050_Init() AND before the unsafe-restart gate below, both on
    // purpose.
    //
    // Before MPU6050_Init() because a mode that diagnoses a dead I2C bus has to
    // run before anything that would hang on it.
    //
    // Before the gate because every mode that uses this hook is a DIAGNOSTIC
    // that never drives -- MODE=gyrodiag and MODE=panel both take over here and
    // never return. The gate exists to stop the chassis driving from an unknown
    // pose, which is not something a bench test of the LED and button can do;
    // blocking them would mean a brown-out during a previous run makes the
    // button test look broken, when the button is fine. Motors_Stop() has
    // already run, so nothing can move either way.
    //
    // Every driving mode leaves this empty, returns immediately, and still
    // meets the gate below untouched.
    Mode_PreGyro();

#if HALT_ON_UNSAFE_RESTART
    if (ResetLog_UnsafeRestart()) {
        // Refuse to drive. See HALT_ON_UNSAFE_RESTART in config.h -- carrying
        // on from an unknown pose is what turned one brown-out into a whole run
        // of undefined behaviour.
        Motors_Stop();
        Debug_P("*** HALTED: will not restart a run that was interrupted"
                " mid-motion.\r\n");
        Debug_P("    The robot is not where the firmware would assume, so"
                " driving\r\n");
        Debug_P("    on would be guesswork. Fix the supply (this was a"
                " brown-out),\r\n");
        Debug_P("    then CYCLE THE POWER for a few seconds -- that clears"
                " SRAM and\r\n");
        Debug_P("    gives a clean cold start. Set HALT_ON_UNSAFE_RESTART to 0"
                "\r\n");
        Debug_P("    to override, but expect undefined behaviour if you do."
                "\r\n");
        Debug_Flush();
        // Blink it out too. A halted robot and a flat battery look identical
        // from across the room, and the serial cable is usually not attached
        // at the moment this fires.
        Panel_SetLed(LED_BLINK_FAST);
        for (;;) { Motors_Stop(); Panel_Task(); }
    }
#endif


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

    Mode_Header();

    next_tick = millis();
    run_start = millis();

    // A mode that owns its whole run does its work in here and never comes
    // back; one that runs on the control loop returns immediately.
    Mode_Begin();

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
        Mode_Tick(&ctx);

        // Measure how long the tick's real work took. If this exceeds the
        // budget the control loop is no longer running at a fixed rate, which
        // silently invalidates the gyro integration and the PD tuning.
        if ((millis() - tick_start) > TICK_OVERRUN_WARN_MS) {
            if (overruns < 0xFFFF) overruns++;
        }
        ctx.overruns = overruns;

        // Reporting, after the deadline measurement -- same position this had
        // when it was a telemetry() call in this function.
        Mode_Telemetry(&ctx);

        // ---- global safety ----------------------------------------------
        if ((millis() - run_start) > MAX_RUN_MS) {
            Motors_Stop();
            Debug_P("RUN LIMIT\r\n");
            for (;;) { }
        }
    }
}
