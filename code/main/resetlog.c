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
//   magic intact  -> SRAM held its charge -> VCC never collapsed. The MCUCSR
//                    flag then says which: watchdog (a firmware hang -- not
//                    power), RESET pin, a brief power dropout (power-on flag),
//                    or a brown-out dip.
//   magic lost    -> SRAM was wiped -> VCC really did fall to near zero: a
//                    normal switch-on, or a broken connection.
#define BOOT_MAGIC 0xB007
static uint16_t s_boot_magic  __attribute__((section(".noinit")));
static uint16_t s_boot_count  __attribute__((section(".noinit")));
static uint8_t  s_prev_flags  __attribute__((section(".noinit")));

static void print_activity(uint8_t a) {
    switch (a) {
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
}

// What the rail was doing just before the reset, and whether that points at
// the supply. Only meaningful when SRAM survived (Power_CrashValid()).
static void report_rail(uint8_t watchdog) {
    uint16_t mn;
    if (!(Power_CrashValid() && Power_CrashMinMv() > 0)) return;
    mn = Power_CrashMinMv();
    Debug_P("  BEFORE THE RESET: lowest rail seen was ");
    Debug_Int((int32_t)mn);
    Debug_P(" mV, during ");
    print_activity(Power_CrashActivity());
    Debug_NL();
    Debug_Flush();

    if (mn < POWER_MIN_SAFE_MV) {
        Debug_P("  -> The rail was SAGGING before the reset: a current-delivery\r\n");
        Debug_P("     problem -- capacitance, wire or connector resistance, or\r\n");
        Debug_P("     motor current sharing the logic ground.\r\n");
    } else if (watchdog) {
        // The watchdog does not care about the supply. A healthy rail here
        // rules the supply out, which is the whole point of printing it.
        Debug_P("  -> The rail was fine, so this was NOT a power problem.\r\n");
    } else {
        Debug_P("  -> The rail was healthy at the last sample, then gone: an\r\n");
        Debug_P("     abrupt collapse, not a sag -- a connection opening, or a\r\n");
        Debug_P("     regulator cutting out. Capacitors will NOT fix this.\r\n");
    }
    Debug_Flush();
}

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
    Debug_Flush();

    // Explained by the FIRST matching cause, most specific first, so each
    // reset gets one verdict instead of several that contradict each other.
    if (!ram_survived) {
        // SRAM wiped: the supply really went away. On a normal switch-on the
        // brown-out flag comes with power-on (the rail ramps up through the
        // BOD threshold) -- that is expected, not a fault.
        if (f & (1 << PORF)) {
            Debug_P("  cold start -- normal power-on, nothing to read into it\r\n");
        } else {
            Debug_P("  *** SRAM WAS WIPED without a power-on flag: the supply fell\r\n");
            Debug_P("  to near zero -- an intermittent power connection.\r\n");
        }
        Debug_Flush();
    } else {
        Debug_P("  *** UNEXPECTED RESET #");
        Debug_Int((int32_t)s_boot_count);
        Debug_P(" ***\r\n");
        Debug_Flush();
        if (f & (1 << WDRF)) {
            Debug_P("  WATCHDOG -> the firmware stopped feeding it for 250 ms: a\r\n");
            Debug_P("  hang, not a supply fault. Last activity: ");
            print_activity(Power_CrashActivity());
            Debug_NL();
            Debug_Flush();
            report_rail(1);
        } else if (f & (1 << EXTRF)) {
            Debug_P("  RESET PIN pulled low: no 10k pull-up + 100nF on pin 9, or\r\n");
            Debug_P("  a dangling ISP cable picking up noise.\r\n");
            Debug_Flush();
        } else if (f & (1 << PORF)) {
            Debug_P("  POWER DROPPED OUT briefly (power-on flag, but SRAM kept its\r\n");
            Debug_P("  contents): the supply blinked off and back -- a switch,\r\n");
            Debug_P("  battery holder or jumper contact opening for a moment.\r\n");
            Debug_Flush();
            report_rail(0);
        } else if (f & (1 << BORF)) {
            Debug_P("  BROWNOUT: the rail dipped below the BOD threshold.\r\n");
            Debug_Flush();
            report_rail(0);
        }
        Debug_KVF("  previous boot's flags", (int32_t)s_prev_flags);
        Debug_NL();
        Debug_NL();
        Debug_Flush();
    }

    s_prev_flags = f;
}
