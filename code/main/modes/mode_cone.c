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

// Build mode 4 -- sonar cone characterisation. Motors stay off; the chassis is
// rotated BY HAND so the beam edges can be found, which is why this mode prints
// RAW readings rather than the median-filtered ones every other mode steers on.
void Mode_PreGyro(void) { }
void Mode_Begin(void) { }
static void cone_telemetry(void) {
    static uint32_t last = 0;
    if ((millis() - last) < SONAR_TEST_INTERVAL_MS) return;
    last = millis();

    Debug_CSV(Heading_DegreesTenths());
    Debug_CSV(Sonar_Latest(SONAR_LEFT));
    Debug_CSV(Sonar_Latest(SONAR_FRONT));
    Debug_CSV(Sonar_Latest(SONAR_RIGHT));
    Debug_CSV(Sonar_IsOpen(SONAR_LEFT));
    Debug_CSV(Sonar_IsOpen(SONAR_RIGHT));
    Debug_CSV(Sonar_IsTooClose(SONAR_LEFT));
    Debug_CSV(Sonar_IsTooClose(SONAR_RIGHT));
    Debug_CSV(Sonar_FrontBlocked());
    Debug_Int(Sonar_FrontVotes());
    Debug_NL();
}

void Mode_Header(void) {
    Debug_P("# hdg10,L,F,R,Lopen,Ropen,Ltoo,Rtoo,Fblocked,Fvotes");
    Debug_NL();
}

void Mode_Tick(const tick_ctx_t *t) {
    int16_t  rate      = t->rate;
    uint32_t run_start = t->run_start;
    (void)rate; (void)run_start;
        // Mode 4: motors permanently off. Sonar/heading above still run every
        // tick, so rotating the chassis by hand is exactly what the cone test
        // needs -- only the telemetry format differs (see below).
        Motors_Stop();
        (void)rate;
}

void Mode_Telemetry(const tick_ctx_t *t) {
    (void)t;
    cone_telemetry();
}
