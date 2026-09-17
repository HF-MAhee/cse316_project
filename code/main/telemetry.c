#include "config.h"
#include "telemetry.h"
#include "mpu6050.h"
#include "heading.h"
#include "sonar.h"
#include "drive.h"
#include "maze.h"
#include "debug.h"
#include "timer.h"

void Telemetry_Header(void) {
    Debug_P("# st,L,F,R,lok,rok,near,fv,md,br,err,wt,gt,corr,pwmL,pwmR,"
            "rock,rate,gx,gy,ovr,drop");
    Debug_NL();
}

void Telemetry_NoneHeader(void) {
    Debug_P("# no periodic CSV in this mode -- event and summary lines only");
    Debug_NL();
}

void Telemetry_Tick(const tick_ctx_t *t) {
    int16_t  gyro_rate = t->rate;
    int16_t  gx = t->gx, gy = t->gy;
    uint16_t overruns = t->overruns;
    static uint32_t last = 0;
    const drive_debug_t *d;
    if ((millis() - last) < TELEMETRY_INTERVAL_MS) return;
    last = millis();

#if DEBUG_LEVEL >= 1
    d = Drive_Debug();
    Debug_CSV((int32_t)Maze_State());
    Debug_CSV(Sonar_Median(SONAR_LEFT));
    Debug_CSV(Sonar_Median(SONAR_FRONT));
    Debug_CSV(Sonar_Median(SONAR_RIGHT));
    Debug_CSV(Sonar_IsValid(SONAR_LEFT));
    Debug_CSV(Sonar_IsValid(SONAR_RIGHT));
    // one field for both too-close flags: 0 none, 1 left, 2 right, 3 both
    Debug_CSV((Sonar_IsTooClose(SONAR_LEFT) ? 1 : 0) |
              (Sonar_IsTooClose(SONAR_RIGHT) ? 2 : 0));
    Debug_CSV(Sonar_FrontVotes());
    Debug_CSV((int32_t)Drive_Mode());
    Debug_CSV(d->branch);
    Debug_CSV(d->error_cm);
    Debug_CSV(d->wall_term);
    Debug_CSV(d->gyro_term);
    Debug_CSV(d->corr);
    Debug_CSV(d->pwm_l);
    Debug_CSV(d->pwm_r);
    Debug_CSV(Motion_IsSuspect());
    Debug_CSV(gyro_rate);
    Debug_CSV(gx);
    Debug_CSV(gy);
    Debug_CSV(overruns);
    Debug_Int(Debug_Dropped());
    Debug_NL();
#else
    (void)gyro_rate; (void)gx; (void)gy; (void)overruns; (void)d;
#endif
}
