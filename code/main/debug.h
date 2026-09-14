#ifndef DEBUG_H
#define DEBUG_H
#include <stdint.h>
#include <avr/pgmspace.h>

// ============================================================================
//  Non-blocking telemetry.
//
//  CRITICAL: the old implementation spun on UDRE for every byte, so a long
//  telemetry line stalled the control loop. Measuring the system must not
//  perturb it. This version queues into a ring buffer drained by the UDRE
//  interrupt, and DROPS output when the buffer is full rather than waiting.
//  A dropped debug line is harmless; a missed control tick is not.
// ============================================================================

void Debug_Init(void);
void Debug_Str(const char *s);
void Debug_Int(int32_t v);
void Debug_NL(void);
void Debug_KV(const char *key, int32_t v);

// Compact CSV field: writes the integer followed by a comma.
void Debug_CSV(int32_t v);

// Bytes lost to buffer-full. If this climbs, telemetry is over budget:
// raise the baud rate or print fewer fields, don't just ignore it.
uint16_t Debug_Dropped(void);

// Spin until the queue drains. Only for shutdown / fatal paths.
void Debug_Flush(void);

// ============================================================================
//  FLASH-RESIDENT STRINGS -- use these for every literal.
//
//  avr-gcc puts string literals in .data, which the startup code copies into
//  SRAM. The ATmega32 has 2048 bytes of SRAM TOTAL, and this project's debug
//  text had grown to ~6300 bytes: .data alone was three times the whole of RAM,
//  so the build either failed to link or left no room for the stack and the MCU
//  died before printing a character. Flash is 32K and barely a third used, so
//  the text belongs there.
//
//  Debug_P("...") and Debug_KVF("key", v) are the drop-in replacements for
//  Debug_Str("...") and Debug_KV("key", v). Keep plain Debug_Str() only for
//  strings that genuinely live in RAM (a formatted buffer, say).
// ============================================================================
void Debug_StrP(const char *flash_str);
void Debug_KVP(const char *flash_key, int32_t v);
#define Debug_P(s)      Debug_StrP(PSTR(s))
#define Debug_KVF(k, v) Debug_KVP(PSTR(k), (v))
#endif
