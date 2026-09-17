#include "config.h"
#include <avr/io.h>
#include "resetlog.h"
#include "debug.h"
#include "power.h"

// ---------------------------------------------------------------------------
//  Reset forensics
// ---------------------------------------------------------------------------
// Variables in .noinit are NOT zeroed by the C startup code, so they keep their
// values across a RESET -- but they are lost if VCC actually falls far enough
// for SRAM to forget. That makes them a direct physical test of WHICH kind of
// fault happened, which the MCUCSR flags alone cannot tell you:
//
//   magic intact  -> SRAM held its charge -> VCC never collapsed.
//                    The reset came from the brown-out detector or the RESET
//                    pin. Suspect a supply DIP or electrical noise.
//   magic lost    -> SRAM was wiped -> VCC really did fall to near zero.
//                    That is a BROKEN CONNECTION, not a dip.
#define BOOT_MAGIC 0xB007
static uint16_t s_boot_magic  __attribute__((section(".noinit")));
static uint16_t s_boot_count  __attribute__((section(".noinit")));
static uint8_t  s_prev_flags  __attribute__((section(".noinit")));

// Was the robot MOVING when the last reset hit?
//
// This is the difference between "restarted cleanly" and the reported symptom
// that everything after an unexpected reset was undefined. A reset does not
// return the robot to the start line -- it leaves it at an unknown position and
// heading, maybe still coasting -- so re-running the mode from scratch drives
// blind from a pose the firmware has no idea about. Worse, the startup gyro
// calibration then runs on a chassis that may still be rotating, which poisons
// the heading zero for the entire next run.
//
// Kept in .noinit so it survives the reset. Only trustworthy when the boot
// magic also survived; a real power cycle wipes both and reads as a cold start.
#define RUN_IDLE   0x00
#define RUN_MOVING 0x5A            // distinctive, so uninitialised RAM is
                                   // unlikely to imitate it
static uint8_t  s_run_state   __attribute__((section(".noinit")));

// Set once report_reset_cause() has decided the previous boot died mid-motion.
static uint8_t  s_unsafe_restart = 0;

void ResetLog_Report(void) {
    uint8_t f = MCUCSR;
    uint8_t ram_survived;
    MCUCSR = 0;                     // must clear, or flags accumulate forever

    ram_survived = (s_boot_magic == BOOT_MAGIC) ? 1 : 0;
    if (ram_survived) {
        s_boot_count++;
        // The run-state flag is only meaningful when SRAM held, because that is
        // the only case where the previous boot actually wrote it.
        if (s_run_state == RUN_MOVING) s_unsafe_restart = 1;
    } else {
        s_boot_magic = BOOT_MAGIC;
        s_boot_count = 1;
        s_prev_flags = 0;
        s_run_state  = RUN_IDLE;     // cold start: nothing was in progress
    }

    Debug_P("RESET:");
    if (f & (1 << PORF))  Debug_P(" power-on");
    if (f & (1 << EXTRF)) Debug_P(" EXTERNAL(reset-pin)");
    if (f & (1 << BORF))  Debug_P(" BROWNOUT");
    if (f & (1 << WDRF))  Debug_P(" watchdog");
    if (f == 0)           Debug_P(" (none/unknown)");
    Debug_KVF("  raw", f);
    Debug_KVF("boot#", (int32_t)s_boot_count);
    Debug_NL();

    if (s_boot_count == 1) {
        Debug_P("  cold start (SRAM was empty) -- baseline, nothing to read"
                " into this one\r\n");
    } else if (ram_survived) {
        Debug_P("  *** UNEXPECTED RESET #");
        Debug_Int((int32_t)s_boot_count);
        Debug_P(" ***\r\n");
        Debug_P("  SRAM SURVIVED, so VCC did NOT collapse. This was the\r\n");
        Debug_P("  brown-out detector or the RESET pin, not a broken wire.\r\n");
        if (f & (1 << EXTRF)) {
            Debug_P("  EXTERNAL flag -> the RESET PIN was pulled low. On a\r\n");
            Debug_P("  bare build that usually means no 10k pull-up + 100nF on\r\n");
            Debug_P("  pin 9, or a dangling ISP cable picking up noise.\r\n");
        } else if (f & (1 << BORF)) {
            Debug_P("  BROWNOUT flag -> the rail dipped below the BOD\r\n");
            Debug_P("  threshold. Decoupling and bulk capacitance.\r\n");
        }
        Debug_KVF("  previous boot's flags", (int32_t)s_prev_flags);
        Debug_NL();
        if (s_unsafe_restart) {
            Debug_P("  *** AND THE ROBOT WAS MOVING WHEN IT HAPPENED ***\r\n");
            Debug_P("  So its position and heading are now unknown, and it may\r\n");
            Debug_P("  still have been coasting through the gyro calibration.\r\n");
            Debug_P("  There is no safe way to carry on from here.\r\n");
        }
        Debug_NL();
    } else {
        Debug_P("  *** SRAM WAS WIPED -> VCC actually fell to near zero ***\r\n");
        Debug_P("  That is an INTERMITTENT POWER CONNECTION, not a dip:\r\n");
        Debug_P("  battery holder contacts, a VCC/GND jumper, or the buck\r\n");
        Debug_P("  converter dropping out. Note the boot# restarting at 1\r\n");
        Debug_P("  every time is itself the evidence.\r\n");
    }
    // ---- what the rail was doing when the MCU died -----------------------
    // This is the measurement that separates the two candidate faults, and they
    // need opposite fixes. See the commentary in power.h.
    if (Power_CrashValid() && Power_CrashMinMv() > 0) {
        uint16_t mn = Power_CrashMinMv();
        Debug_P("  BEFORE THE RESET: lowest rail seen was ");
        Debug_Int((int32_t)mn);
        Debug_P(" mV, during ");
        switch (Power_CrashActivity()) {
            case ACT_IDLE:        Debug_P("idle (motors off)");    break;
            case ACT_DRIVE_KICK:  Debug_P("the DRIVE KICK");       break;
            case ACT_DRIVING:     Debug_P("normal driving");       break;
            case ACT_DRIVE_BRAKE: Debug_P("the DRIVE BRAKE");      break;
            case ACT_TURN_KICK:   Debug_P("the PIVOT KICK");       break;
            case ACT_TURN_SWEEP:  Debug_P("the pivot sweep");      break;
            case ACT_TURN_BRAKE:  Debug_P("the pivot brake");      break;
            case ACT_TURN_NUDGE:  Debug_P("a pivot nudge");        break;
            case ACT_REVERSING:   Debug_P("the reverse");          break;
            default:              Debug_P("an unknown phase");     break;
        }
        Debug_NL();
        Debug_Flush();

        // The interpretation, spelled out, because the two cases look identical
        // in the MCUCSR flags and are fixed by completely different work.
        if (mn < POWER_MIN_SAFE_MV) {
            Debug_P("  -> The rail was ALREADY SAGGING before it died, so this"
                    " is a\r\n");
            Debug_P("     current-delivery problem: capacitance, wire"
                    " resistance,\r\n");
            Debug_P("     connector resistance, shared ground return.\r\n");
        } else {
            Debug_P("  -> The rail was STILL HEALTHY at the last sample, then"
                    " gone.\r\n");
            Debug_P("     That is an ABRUPT COLLAPSE, not a sag: a regulator"
                    " shutting\r\n");
            Debug_P("     off (over-current hiccup / thermal / two regulators"
                    "\r\n");
            Debug_P("     fighting) or a connection momentarily opening. Adding"
                    "\r\n");
            Debug_P("     capacitors will NOT fix this one.\r\n");
        }
        Debug_Flush();
    }

    Debug_Flush();
    s_prev_flags = f;
}
uint8_t ResetLog_UnsafeRestart(void) { return s_unsafe_restart; }
void    ResetLog_MarkMoving(void)    { s_run_state = RUN_MOVING; }
void    ResetLog_MarkIdle(void)      { s_run_state = RUN_IDLE; }
