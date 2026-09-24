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

void ResetLog_Report(void) {
    uint8_t f = MCUCSR;
    uint8_t ram_survived;
    MCUCSR = 0;                     // must clear, or flags accumulate forever

    ram_survived = (s_boot_magic == BOOT_MAGIC) ? 1 : 0;
    if (ram_survived) {
        s_boot_count++;
    } else {
        s_boot_magic = BOOT_MAGIC;
        s_boot_count = 1;
        s_prev_flags = 0;
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
    // The debug TX ring is 192 bytes and DROPS on overflow. This report is
    // several times that on a warm reset, so it is flushed in pieces -- the
    // unflushed version arrived as "fi rpc tRr4un" in usart_20260924_second.
    // Nothing is moving yet, so blocking here costs nothing.
    Debug_Flush();

    if (s_boot_count == 1) {
        Debug_P("  cold start (SRAM was empty) -- baseline, nothing to read"
                " into this one\r\n");
    } else if (ram_survived) {
        Debug_P("  *** UNEXPECTED RESET #");
        Debug_Int((int32_t)s_boot_count);
        Debug_P(" ***\r\n");
        Debug_P("  SRAM SURVIVED, so VCC did NOT collapse. This was the\r\n");
        Debug_P("  brown-out detector or the RESET pin, not a broken wire.\r\n");
        Debug_Flush();
        if (f & (1 << EXTRF)) {
            Debug_P("  EXTERNAL flag -> the RESET PIN was pulled low. On a\r\n");
            Debug_P("  bare build that usually means no 10k pull-up + 100nF on\r\n");
            Debug_P("  pin 9, or a dangling ISP cable picking up noise.\r\n");
            Debug_Flush();
        } else if (f & (1 << BORF)) {
            Debug_P("  BROWNOUT flag -> the rail dipped below the BOD\r\n");
            Debug_P("  threshold. Decoupling and bulk capacitance.\r\n");
        }
        Debug_KVF("  previous boot's flags", (int32_t)s_prev_flags);
        Debug_NL();
        Debug_NL();
        Debug_Flush();
    } else {
        Debug_P("  *** SRAM WAS WIPED -> VCC actually fell to near zero ***\r\n");
        Debug_P("  That is an INTERMITTENT POWER CONNECTION, not a dip:\r\n");
        Debug_Flush();
        Debug_P("  battery holder contacts, a VCC/GND jumper, or the buck\r\n");
        Debug_P("  converter dropping out. Note the boot# restarting at 1\r\n");
        Debug_P("  every time is itself the evidence.\r\n");
        Debug_Flush();
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
            Debug_Flush();
            Debug_P("     current-delivery problem: capacitance, wire"
                    " resistance,\r\n");
            Debug_P("     connector resistance, shared ground return.\r\n");
            Debug_Flush();
        } else {
            Debug_P("  -> The rail was STILL HEALTHY at the last sample, then"
                    " gone.\r\n");
            Debug_Flush();
            Debug_P("     That is an ABRUPT COLLAPSE, not a sag: a regulator"
                    " shutting\r\n");
            Debug_P("     off (over-current hiccup / thermal / two regulators"
                    "\r\n");
            Debug_Flush();
            Debug_P("     fighting) or a connection momentarily opening. Adding"
                    "\r\n");
            Debug_P("     capacitors will NOT fix this one.\r\n");
        }
        Debug_Flush();
    }

    Debug_Flush();
    s_prev_flags = f;
}
