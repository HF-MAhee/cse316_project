#ifndef TELEMETRY_H
#define TELEMETRY_H
#include "solver.h"

// The per-tick control-loop trace: one CSV line every TELEMETRY_INTERVAL_MS.
// The first column is the solver state -- see Solver_State() for the codes.
void Telemetry_Header(void);
void Telemetry_Tick(const tick_ctx_t *t);

#endif
