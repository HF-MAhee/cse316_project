#ifndef DEBUG_H
#define DEBUG_H
#include <stdint.h>

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
#endif
