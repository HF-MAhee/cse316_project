#ifndef TELEMETRY_H
#define TELEMETRY_H
#include "mode.h"

// The standard per-tick control-loop trace, shared by every mode that drives a
// corridor. Modes with their own format (the sonar-cone test, the dead-end run)
// print their own instead and do not call these.
void Telemetry_Header(void);
void Telemetry_Tick(const tick_ctx_t *t);

// For modes that produce event lines only -- says so once, so a silent log is
// not mistaken for a dead link.
void Telemetry_NoneHeader(void);

#endif
