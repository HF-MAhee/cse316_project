// ============================================================================
//  Closed-loop simulator for the two-run maze solver.
//
//  Runs the REAL firmware -- main.c, solver.c, wallmem.c, drive.c, turn.c,
//  heading.c, sonar.c, telemetry.c, panel.c -- against a physical model of the
//  demo maze in README.md. Only the lowest layer is replaced: motors, gyro,
//  clock, debug UART, supply monitor. The sonar driver is NOT replaced: its
//  trigger/echo pins are driven from the model, so sonar.c's own timing,
//  filtering and voting are what get tested.
//
//  The model is calibrated against the real robot's logs (usart_20260924_*):
//    - pivots: sweep cut at 55 deg lands at ~70-80, a 40 ms nudge is worth
//      a few degrees; brake pulses mostly vanish into motor dead time
//    - straight: ~30 cm/s cruise, rolls ~5-7 cm after the brake
//    - driving off from rest, one wheel bites late: 25-70 deg/s of yaw
//    - HC-SR04 beams: a flat wall echoes only near its normal, but the END of
//      a wall echoes from 20-30 deg off-axis (why the left sonar read ~18 cm
//      beside the D0 pocket with C1 wide open); 1-4% dropouts, some garbage
//    - gyro bias, noise and a sensitivity error (SIM_SCALE_PCT, default 3%)
//    - the axle wanders a little during a pivot; the start pose is +/-2 cm, 3 deg
//  Every one of those is randomised per seed, so a sweep over seeds is a sweep
//  over plausible robots.
//
//  PASS means: run 1 makes exactly the designed decisions, collapses to the
//  5-record route and saves it; run 2 replays it without a mismatch and exits;
//  no wall contact (chassis outline vs wall segments); no pivot ends more than
//  8 degrees off the true maze grid.
//
//  Usage:  sim <seed> [logfile] [q]      exit status 0 = PASS
//          sim probe <x> <y> <hdg_deg>   print the three sonar readings there
//  See test/sim/run.sh (make sim).
// ============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>

#include <avr/io.h>
#include "config.h"
#include "motors.h"
#include "mpu6050.h"
#include "timer.h"
#include "debug.h"
#include "power.h"
#include "solver.h"
#include "wallmem.h"

extern uint8_t  Panel_TestLevel;
extern uint8_t  Panel_TestLed;
extern uint32_t Panel_TestMs;
int fw_main(void);
int32_t Heading_GridErrorTenths(void);
void WallMem_HostEraseEeprom(void);

#define DEG (M_PI / 180.0)

static FILE *slog;                  // firmware debug output + [sim] notes

// ---------------------------------------------------------------------------
//  Maze: the demo maze's 11 cut pieces (README.md), cm, origin bottom-left.
// ---------------------------------------------------------------------------
typedef struct { double x1, y1, x2, y2; } seg_t;
static const seg_t WALLS[] = {
    {   0,   0, 160,   0 }, {   0,  80,  80,  80 }, { 120,  80, 200,  80 },
    {  40, 120, 160, 120 }, {   0,   0,   0,  80 }, {  40,   0,  40,  40 },
    {  40,  80,  40, 120 }, {  80,  40,  80,  80 }, { 120,   0, 120,  40 },
    { 160,   0, 160,  40 }, { 200,   0, 200, 120 },
};
#define NWALLS ((int)(sizeof(WALLS) / sizeof(WALLS[0])))

// Robot geometry, relative to the axle centre (x forward, y left).
#define R_FRONT   15.0      // SONAR_TO_AXLE_CM: the front sonar is at the nose
#define R_REAR     7.0      // ROBOT_LENGTH 22 - 15
#define R_HALFW    8.0      // ROBOT_WIDTH 16 / 2
#define TRACK     13.0      // wheel track

typedef struct { double x, y, a; } sensor_t;           // a = axis angle, rad
static const sensor_t SENS[3] = {
    // Side faces sit inboard of the chassis edge: centred in a 40 cm corridor
    // they read ~15 cm, as the real robot does (config SIDE_CENTRED_CM).
    { 12.0,  5.0,      M_PI / 2 },   // left
    { R_FRONT, 0.0,    0.0      },   // front
    { 12.0, -5.0,     -M_PI / 2 },   // right
};

// ---------------------------------------------------------------------------
//  Randomised robot
// ---------------------------------------------------------------------------
static struct {
    double gain_l, gain_r;      // motor strength
    double tau_drive;           // s, motor/chassis response when driven
    double fric_w;              // rad/s^2 kinetic friction on yaw
    double tau_w;               // s, yaw response (chassis inertia)
    double kpiv;                // pivot: unloaded yaw rate per unit wheel command
    double deadtime_ms;         // every start from rest: no torque for this long
    double ramp_ms;             // ...then torque builds up over this long (winding inductance)
    double fric_v;              // cm/s^2 rolling friction
    double accel_max;           // cm/s^2 the motors can deliver
    double backlash_ms_max;     // per-reversal drag time, uniform 0..max
    double driveoff_ms_max;     // one wheel late to bite when driving off from rest
    double gyro_scale;          // sensitivity error
    double gyro_bias;           // raw LSB
    double gyro_noise;          // raw LSB, 1 sigma
    double sonar_noise;         // cm, 1 sigma
    double p_drop, p_spur;      // per-ping dropout / garbage probability
    double surf_half, edge_half;// rad: flat-wall and wall-end beam half-angles
    double pivot_slip;          // cm of axle drift per radian of pivot
    double slip_dir;            // direction of that drift, robot frame
    double start_dx, start_da;  // placement error at the start cell
} P;

// splitmix64, not rand(): glibc's rand() is an additive lagged-Fibonacci
// generator, and with a fixed number of draws per sonar cycle its lag
// correlations showed up as PERIODIC dropouts and garbage readings -- every
// third front ping, say -- which no real sensor produces.
static uint64_t rng_state = 1;
static uint64_t rng_next(void) {
    uint64_t z = (rng_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
static double urand(void) { return (double)(rng_next() >> 11) * (1.0 / 9007199254740992.0); }
static double urange(double a, double b) { return a + (b - a) * urand(); }
static double grand(void) {
    double u1 = urand() + 1e-12, u2 = urand();
    return sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2);
}

static void randomise(int seed) {
    rng_state = (uint64_t)seed * 0x2545F4914F6CDD1Dull + 1u;
    P.gain_l          = urange(0.94, 1.06);
    P.gain_r          = urange(0.94, 1.06);
    P.tau_drive       = urange(0.05, 0.09);
    P.fric_w          = urange(40, 60);
    P.tau_w           = urange(0.12, 0.18);
    P.kpiv            = (urange(5.0, 7.0) + P.fric_w * P.tau_w) / 60.0;
    P.deadtime_ms     = urange(3, 10);
    P.ramp_ms         = urange(10, 30);
    P.fric_v          = urange(70, 110);
    P.accel_max       = P.fric_v + urange(40, 80);    // must beat friction to move at all
    P.backlash_ms_max = urange(5, 25);
    P.driveoff_ms_max = urange(60, 220);
    {   // gyro sensitivity error: +/-3% is the MPU-6050 datasheet worst case
        double r = getenv("SIM_SCALE_PCT") ? atof(getenv("SIM_SCALE_PCT")) / 100.0 : 0.03;
        P.gyro_scale = urange(1.0 - r, 1.0 + r);
    }
    if (getenv("SIM_SCALE")) P.gyro_scale = atof(getenv("SIM_SCALE"));
    P.gyro_bias       = urange(-120, -40);
    P.gyro_noise      = 12;
    P.sonar_noise     = 0.3;
    P.p_drop          = urange(0.01, 0.04);
    P.p_spur          = urange(0.003, 0.012);
    P.surf_half       = urange(12, 18) * DEG;
    P.edge_half       = urange(22, 32) * DEG;
    P.pivot_slip      = urange(0.0, 1.2);
    P.slip_dir        = urange(-M_PI, M_PI);
    P.start_dx        = urange(-2.0, 2.0);
    P.start_da        = urange(-3.0, 3.0) * DEG;
}

// ---------------------------------------------------------------------------
//  World state
// ---------------------------------------------------------------------------
static double   rx, ry, ra;         // axle centre (cm) and heading (rad, CCW, 0 = +x)
static double   v, w;               // forward speed cm/s, yaw rate rad/s
static int      cmd_dir[2] = { DIR_STOP, DIR_STOP };   // 0 = left, 1 = right
static int      cmd_pwm[2] = { 0, 0 };
static double   slack_until[2];     // sim seconds: wheel drags until then
static double   engage_at[2];       // sim seconds: when the current push began
static int      last_drive_dir[2] = { DIR_FWD, DIR_FWD };
static int64_t  now_us = 0;
static double   phys_acc = 0;

static void start_pose(void) {
    rx = 180.0 + P.start_dx;
    ry = 15.0;
    ra = M_PI / 2 + P.start_da;
    v = w = 0;
}

static double wheel_target(int side, int *driven) {
    double s;
    if (cmd_dir[side] == DIR_STOP || cmd_pwm[side] == 0) { *driven = 0; return 0; }
    *driven = 1;
    if (now_us * 1e-6 < slack_until[side]) { *driven = 0; return 0; }
    s = (cmd_pwm[side] - 25.0);          // PWM 60 -> ~30 cm/s after friction
    if (s < 0) s = 0;
    s *= (side == 0) ? P.gain_l : P.gain_r;
    return (cmd_dir[side] == DIR_REV) ? -s : s;
}

static double apply(double x, double a, double f, double dt) {
    double nx;
    if (x == 0.0) {
        if (fabs(a) <= f) return 0.0;                   // cannot break away
        return (a - (a > 0 ? f : -f)) * dt;
    }
    nx = x + (a - (x > 0 ? f : -f)) * dt;
    if ((nx > 0) != (x > 0) && fabs(a) <= f) return 0.0; // friction stops it
    return nx;
}

static void physics_step(double dt) {
    int dl, dr;
    double vl = wheel_target(0, &dl), vr = wheel_target(1, &dr);
    double cur_l = v - w * TRACK / 2, cur_r = v + w * TRACK / 2;
    double av, aw, aw_scale = 1.0;

    if (!dl && !dr) {
        av = 0; aw = 0;                                // coasting
    } else {
        double vt, wt;
        if (!dl) vl = cur_l * 0.3;                     // undriven wheel drags
        if (!dr) vr = cur_r * 0.3;
        vt = (vl + vr) / 2;
        wt = (vr - vl) / TRACK;
        if (vl * vr < 0) wt = (vr - vl) / 2 * P.kpiv;
        av = (vt - v) / P.tau_drive;
        {   // torque builds up over ramp_ms after a start (winding inductance):
            // short pulses are weak but not dead, as the real robot's nudges are
            double since = now_us * 1e-6 - (engage_at[0] > engage_at[1] ? engage_at[0] : engage_at[1]);
            double k = since / (P.ramp_ms * 1e-3);
            if (k < 0) k = 0;
            if (k > 1) k = 1;
            av *= k;
            aw_scale = k;
        }
        // The motors can only push so hard: from cruise the real robot still
        // rolls ~5 cm after an 80 ms reverse-brake (F=12 -> 6-7 in the logs).
        if (av >  P.accel_max) av =  P.accel_max;
        if (av < -P.accel_max) av = -P.accel_max;
        aw = (wt - w) / P.tau_w * aw_scale;
    }
    // Rolling, the tyres barely resist yaw; pivoting on the spot they scrub.
    v = apply(v, av, P.fric_v, dt);
    w = apply(w, aw, P.fric_w * exp(-fabs(v) / 5.0), dt);

    // Pivot slip: the axle wanders while the tyres scrub.
    if (fabs(v) < 5 && fabs(w) > 0.3) {
        double d = P.pivot_slip * fabs(w) * dt;
        rx += d * cos(ra + P.slip_dir);
        ry += d * sin(ra + P.slip_dir);
    }
    rx += v * cos(ra) * dt;
    ry += v * sin(ra) * dt;
    ra += w * dt;
}

static void track_world(void);
// The clock advances by exactly what was spent; physics integrates in fixed
// 200 us steps behind it. (Stepping the clock itself in 200 us chunks
// quantised every echo to 3.4 cm -- far coarser than a real HC-SR04.)
static int64_t phys_us = 0;
static void advance_us(double us) {
    phys_acc += us;
    if (phys_acc >= 1.0) {
        int64_t whole = (int64_t)phys_acc;
        now_us += whole;
        phys_acc -= (double)whole;
    }
    while (now_us - phys_us >= 200) {
        physics_step(200e-6);
        phys_us += 200;
        track_world();
    }
}

// ---------------------------------------------------------------------------
//  Geometry helpers
// ---------------------------------------------------------------------------
static int seg_intersect(double ax, double ay, double bx, double by,
                         double cx, double cy, double dx, double dy) {
    double d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    double d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
    double d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
    double d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}
static int occluded(double px, double py, double qx, double qy, int skip) {
    int i;
    for (i = 0; i < NWALLS; i++) {
        if (i == skip) continue;
        if (seg_intersect(px, py, qx, qy, WALLS[i].x1, WALLS[i].y1, WALLS[i].x2, WALLS[i].y2))
            return 1;
    }
    return 0;
}
static double ang_diff(double a, double b) {        // a - b, wrapped to (-pi, pi]
    double d = fmod(a - b, 2 * M_PI);
    if (d >  M_PI) d -= 2 * M_PI;
    if (d <= -M_PI) d += 2 * M_PI;
    return d;
}
static double pt_seg_dist(double px, double py, const seg_t *s) {
    double dx = s->x2 - s->x1, dy = s->y2 - s->y1;
    double t = ((px - s->x1) * dx + (py - s->y1) * dy) / (dx * dx + dy * dy);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return hypot(px - (s->x1 + t * dx), py - (s->y1 + t * dy));
}

// True distance a sonar would report, or -1 for no echo.
static double sonar_measure(int id) {
    const sensor_t *s = &SENS[id];
    double px = rx + s->x * cos(ra) - s->y * sin(ra);
    double py = ry + s->x * sin(ra) + s->y * cos(ra);
    double ax = ra + s->a;
    double best = 1e9;
    int i;
    for (i = 0; i < NWALLS; i++) {
        const seg_t *g = &WALLS[i];
        double dx = g->x2 - g->x1, dy = g->y2 - g->y1;
        double t = ((px - g->x1) * dx + (py - g->y1) * dy) / (dx * dx + dy * dy);
        // Flat surface: echoes only along its normal, if the normal is in the beam.
        if (t >= 0 && t <= 1) {
            double fx = g->x1 + t * dx, fy = g->y1 + t * dy;
            double d = hypot(fx - px, fy - py);
            if (d > 0.5 && fabs(ang_diff(atan2(fy - py, fx - px), ax)) <= P.surf_half &&
                !occluded(px, py, fx, fy, i) && d < best) best = d;
        }
        // Wall ends and corners diffract: visible well off-axis.
        {
            double ex[2] = { g->x1, g->x2 }, ey[2] = { g->y1, g->y2 };
            int k;
            for (k = 0; k < 2; k++) {
                double d = hypot(ex[k] - px, ey[k] - py);
                if (d > 0.5 && d < 55 &&
                    fabs(ang_diff(atan2(ey[k] - py, ex[k] - px), ax)) <= P.edge_half &&
                    !occluded(px, py, ex[k] - 0.01 * (ex[k] - px), ey[k] - 0.01 * (ey[k] - py), i) &&
                    d < best) best = d;
            }
        }
    }
    if (urand() < P.p_drop) return -1;
    if (urand() < P.p_spur) return urange(5, 65);
    if (best > 1e8) return -1;
    best += grand() * P.sonar_noise;
    return best < 1 ? 1 : best;
}

// ---------------------------------------------------------------------------
//  Pins: sonar trigger/echo, driven by sonar.c's own code
// ---------------------------------------------------------------------------
uint8_t PORTA, DDRA, PORTB, DDRB, PINB, PORTC, DDRC, PORTD, DDRD, PIND;
static int64_t echo_start[3], echo_end[3];
static uint8_t echo_bits(void) {
    uint8_t v2 = 0;
    const uint8_t eb[3] = { LEFT_ECHO_BIT, FRONT_ECHO_BIT, RIGHT_ECHO_BIT };
    int i;
    for (i = 0; i < 3; i++)
        if (now_us >= echo_start[i] && now_us < echo_end[i]) v2 |= (uint8_t)(1u << eb[i]);
    return v2;
}
uint8_t sim_pina(void) { advance_us(1); return echo_bits(); }

void sim_delay_us(double us) {
    const uint8_t tb[3] = { LEFT_TRIG_BIT, FRONT_TRIG_BIT, RIGHT_TRIG_BIT };
    int i;
    for (i = 0; i < 3; i++) {
        if (PORTA & (1u << tb[i])) {
            // trigger pulse in progress: fire this sensor
            double d = sonar_measure(i);
            if (getenv("SIM_PINGS") && slog && now_us > atof(getenv("SIM_PINGS")) * 1e6 &&
                now_us < (atof(getenv("SIM_PINGS")) + (getenv("SIM_PINGS_LEN") ? atof(getenv("SIM_PINGS_LEN")) : 1.0)) * 1e6)
                fprintf(slog, "  [ping] t=%.3f id=%d d=%.1f pose (%.1f,%.1f,%.1f)\n",
                        now_us / 1e6, i, d, rx, ry, ra / DEG);
            echo_start[i] = now_us + (int64_t)us + 450;
            echo_end[i]   = (d < 0) ? echo_start[i] - 1           // never rises
                                    : echo_start[i] + (int64_t)(d * 58.0);
        }
    }
    advance_us(us);
}

// ---------------------------------------------------------------------------
//  Motors, gyro, clock
// ---------------------------------------------------------------------------
static uint8_t clamp_pwm(uint8_t pwm) {
    if (pwm == 0) return 0;
    if (pwm < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
    if (pwm > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
    return pwm;
}
static void set_wheel(int side, motor_dir_t dir, uint8_t pwm) {
    if (dir != DIR_STOP && pwm && (cmd_dir[side] == DIR_STOP || cmd_pwm[side] == 0 ||
                                   (int)dir != cmd_dir[side])) {
        // electrical + mechanical dead time on every start or reversal
        double t = now_us * 1e-6 + P.deadtime_ms * 1e-3;
        if (t > slack_until[side]) slack_until[side] = t;
        engage_at[side] = t;
    }
    if (dir != DIR_STOP && pwm) {
        if ((int)dir != last_drive_dir[side]) {
            // gearbox backlash: the wheel drags until the slack is taken up
            double t = now_us * 1e-6 + urange(0, P.backlash_ms_max) * 1e-3;
            if (t > slack_until[side]) slack_until[side] = t;
            last_drive_dir[side] = dir;
        }
    }
    cmd_dir[side] = dir;
    cmd_pwm[side] = (dir == DIR_STOP) ? 0 : clamp_pwm(pwm);
}
void Motors_Init(void) {}
void Motors_SetLeft(motor_dir_t dir, uint8_t pwm)  { set_wheel(0, dir, pwm); }
void Motors_SetRight(motor_dir_t dir, uint8_t pwm) { set_wheel(1, dir, pwm); }
void Motors_Forward(uint8_t l, uint8_t r) {
    if (cmd_dir[0] == DIR_STOP && cmd_dir[1] == DIR_STOP) {
        // driving off from rest: one wheel (at random) bites late -- the hard
        // yaw seen at the start of every leg in the real logs
        int late = (urand() < 0.5) ? 0 : 1;
        double t = now_us * 1e-6 + urange(0, P.driveoff_ms_max) * 1e-3;
        set_wheel(0, DIR_FWD, l); set_wheel(1, DIR_FWD, r);
        if (t > slack_until[late]) slack_until[late] = t;
        return;
    }
    set_wheel(0, DIR_FWD, l); set_wheel(1, DIR_FWD, r);
}
void Motors_Pivot(uint8_t cw, uint8_t pwm) {
    if (cw) { set_wheel(0, DIR_FWD, pwm); set_wheel(1, DIR_REV, pwm); }
    else    { set_wheel(0, DIR_REV, pwm); set_wheel(1, DIR_FWD, pwm); }
}
void Motors_Stop(void) { set_wheel(0, DIR_STOP, 0); set_wheel(1, DIR_STOP, 0); }

void MPU6050_Init(void) {}
static int16_t sat16(double x) { return (int16_t)(x > 32767 ? 32767 : x < -32768 ? -32768 : x); }
void MPU6050_ReadAll(gyro_xyz_t *g) {
    advance_us(400);                               // one I2C burst read
    g->z = sat16(w / DEG * 65.5 * P.gyro_scale + P.gyro_bias + grand() * P.gyro_noise);
    g->x = sat16(-250 + grand() * 10);
    g->y = sat16(-25 + grand() * 10);
}
int16_t MPU6050_ReadZ(void) { gyro_xyz_t g; MPU6050_ReadAll(&g); return g.z; }

void I2C_Init(void) {}
void ResetLog_Report(void) {}

void     Timer_Init(void) {}
uint32_t millis(void) { advance_us(1); return (uint32_t)(now_us / 1000); }
uint32_t micros(void) { advance_us(1); return (uint32_t)now_us; }
void     Timer_WaitMs(uint32_t ms) { advance_us(ms * 1000.0); }
uint8_t  Timer_Elapsed(uint32_t start, uint32_t ms) { return (millis() - start) >= ms; }

void     Power_SetActivity(uint8_t a) { (void)a; }
uint8_t  Power_CrashValid(void) { return 0; }
uint16_t Power_CrashMinMv(void) { return 0; }
uint8_t  Power_CrashActivity(void) { return 0; }
void     Power_Init(void) {}
uint16_t Power_VccMv(void) { return 4900; }
void     Power_Task(void) {}
uint16_t Power_MinMv(void) { return 4900; }
uint16_t Power_LastMv(void) { return 4900; }
void     Power_ResetMin(void) {}
uint8_t  Power_SagSeen(void) { return 0; }

// ---------------------------------------------------------------------------
//  Debug output: to the log file, and parsed line by line for the verdict
// ---------------------------------------------------------------------------
static char  line[512];
static int   line_n = 0;
static int   run_no = 0;                     // 1 or 2 while a run is going
static char  turns[3][128];                  // decisions taken per run
static int   n_mismatch = 0, n_exit = 0, n_fault_msgs = 0, n_legtimeout = 0;
static int   n_wrongway = 0, n_collapsed_ok = 0;

static void on_line(const char *s) {
    const char *j = strstr(s, "junction ");
    if (j && run_no >= 1 && run_no <= 2) {
        const char *arrow = strstr(s, "-> ");
        if (arrow) {
            char c = arrow[3];
            size_t n = strlen(turns[run_no]);
            char t = (c == 'L') ? 'L' : (c == 'R') ? 'R' : (c == 'F') ? 'F' : 'U';
            if (n < sizeof(turns[0]) - 1) { turns[run_no][n] = t; turns[run_no][n + 1] = 0; }
        }
    }
    if (strstr(s, "REPLAY MISMATCH") || strstr(s, "ROUTE EXHAUSTED")) n_mismatch++;
    if (strstr(s, "EXIT REACHED")) n_exit++;
    if (strstr(s, "RUN LIMIT") || strstr(s, "NOT SAVING") || strstr(s, "LOG FULL")) n_fault_msgs++;
    if (strstr(s, "leg timeout")) n_legtimeout++;
    if (strstr(s, "WRONG WAY")) n_wrongway++;
    if (strncmp(s, "collapsed ", 10) == 0 && strstr(s, "-> 5 records")) n_collapsed_ok++;
}
static void out_c(char c) {
    if (slog) fputc(c, slog);
    if (c == '\r') return;
    if (c == '\n') { line[line_n] = 0; on_line(line); line_n = 0; return; }
    if (line_n < (int)sizeof(line) - 1) line[line_n++] = c;
}
static void out_s(const char *s) { while (*s) out_c(*s++); }
void Debug_Init(void) {}
void Debug_Str(const char *s) { out_s(s); }
void Debug_StrP(const char *s) { out_s(s); }
void Debug_Int(int32_t x) { char b[16]; snprintf(b, sizeof b, "%ld", (long)x); out_s(b); }
void Debug_NL(void) { out_s("\r\n"); }
void Debug_KV(const char *k, int32_t x) { out_s(k); out_c('='); Debug_Int(x); out_c(' '); }
void Debug_KVP(const char *k, int32_t x) { Debug_KV(k, x); }
void Debug_CSV(int32_t x) { Debug_Int(x); out_c(','); }
uint16_t Debug_Dropped(void) { return 0; }
void Debug_Flush(void) {}

// ---------------------------------------------------------------------------
//  The operator, and the scorekeeping
// ---------------------------------------------------------------------------
static jmp_buf done_jmp;
static int     result = -1;             // 0 pass, else failure code
static const char *fail_why = "";
static int     phase = 0;               // 0 wait run1, 1 run1, 2 run2
static int64_t armed_since = -1, press_until = -1;
static int     last_state = -1;
static double  min_clear = 1e9, min_clear_turn = 1e9;
static int     contacts = 0, in_contact = 0;
static double  worst_turn_err = 0, sum_turn_err = 0; static int n_turns = 0;
static double  worst_leg_err = 0;
static int64_t leg_start_us = 0;
static int     run_moving = 0;         // the current run has left ARMED

static double grid_err_deg(void) {
    double a = fmod(ra / DEG, 90.0);
    if (a < 0) a += 90.0;
    if (a > 45) a -= 90.0;
    return a;
}

static double robot_clearance(void) {
    double c = cos(ra), s = sin(ra);
    double cx[4], cy[4], best = 1e9;
    const double lx[4] = { R_FRONT, R_FRONT, -R_REAR, -R_REAR };
    const double ly[4] = { R_HALFW, -R_HALFW, -R_HALFW, R_HALFW };
    int i, k;
    for (k = 0; k < 4; k++) { cx[k] = rx + lx[k] * c - ly[k] * s; cy[k] = ry + lx[k] * s + ly[k] * c; }
    for (i = 0; i < NWALLS; i++) {
        const seg_t *g = &WALLS[i];
        for (k = 0; k < 4; k++) {
            int n = (k + 1) & 3;
            double d;
            if (seg_intersect(cx[k], cy[k], cx[n], cy[n], g->x1, g->y1, g->x2, g->y2)) return 0;
            d = pt_seg_dist(cx[k], cy[k], g);
            if (d < best) best = d;
            {   // wall endpoints against the robot's edge
                seg_t e = { cx[k], cy[k], cx[n], cy[n] };
                d = pt_seg_dist(g->x1, g->y1, &e); if (d < best) best = d;
                d = pt_seg_dist(g->x2, g->y2, &e); if (d < best) best = d;
            }
        }
    }
    return best;
}

static void finish(int code, const char *why) {
    result = code; fail_why = why;
    longjmp(done_jmp, 1);
}

static int64_t last_track_ms = -1;
static void track_world(void) {
    int64_t ms = now_us / 1000;
    int st;
    if (ms == last_track_ms) return;
    last_track_ms = ms;
    Panel_TestMs = (uint32_t)ms;
    if (ms < 100) return;                     // firmware still booting
    st = Solver_State();

    // --- clearance ----------------------------------------------------------
    if (phase >= 1 && st != 0 && ms % 5 == 0) {
        double c = robot_clearance();
        if (c < min_clear) min_clear = c;
        if (st == 7 && c < min_clear_turn) min_clear_turn = c;
        if (c <= 0.0) {
            if (!in_contact) {
                contacts++;
                if (slog) fprintf(slog, "  [sim] WALL CONTACT st=%d axle (%.1f, %.1f) hdg %.1f\n",
                                  st, rx, ry, ra / DEG);
            }
            in_contact = 1;
        }
        else in_contact = 0;
    }

    // --- turn and leg accuracy (true heading vs the maze grid) --------------
    if (last_state == 7 && st != 7) {                  // a pivot just ended
        double e = fabs(grid_err_deg());
        if (e > worst_turn_err) worst_turn_err = e;
        sum_turn_err += e; n_turns++;
        if (slog) fprintf(slog, "  [sim] pivot done: true grid error %.2f deg, axle (%.1f, %.1f) hdg %.1f  grid misalign %.2f\n",
                          grid_err_deg(), rx, ry, fmod(ra / DEG + 720.0, 360.0),
                          grid_err_deg() - Heading_GridErrorTenths() / 10.0);
    }
    if (st == 2 && last_state != 2) leg_start_us = now_us;
    if (slog && phase >= 1 && ms % 250 == 0 && st >= 2 && st <= 8)
        fprintf(slog, "  [sim] t=%.2f st=%d axle (%.1f, %.1f) hdg %.1f v=%.1f misalign %.1f\n",
                ms / 1000.0, st, rx, ry, fmod(ra / DEG + 720.0, 360.0), v,
                grid_err_deg() - Heading_GridErrorTenths() / 10.0);
    if (st == 2 && now_us - leg_start_us > 700000) {   // settled into the leg
        double e = fabs(grid_err_deg());
        if (e > worst_leg_err) worst_leg_err = e;
    }
    last_state = st;

    // --- the operator ---------------------------------------------------------
    if (press_until >= 0 && now_us >= press_until) { Panel_TestLevel = 1; press_until = -1; }
    if (st == 0) {
        if (armed_since < 0) armed_since = now_us;
    } else { armed_since = -1; run_moving = 1; }

    if (phase == 0 && st == 0 && now_us - armed_since > 1500000) {
        Panel_TestLevel = 0; press_until = now_us + 150000;
        phase = 1; run_no = 1; run_moving = 0;
    } else if (phase == 1 && run_moving && st == 0 && armed_since >= 0 && now_us - armed_since > 300000
               && press_until < 0) {
        if (!Panel_TestLed) finish(2, "run 1 ended without a saved route");
        if (n_exit < 1) finish(3, "run 1 re-armed without reaching the exit");
        // carry the robot back to the start cell and press again
        start_pose();
        armed_since = now_us;
        phase = 2; run_no = 2; run_moving = 0;
        Panel_TestLevel = 0; press_until = now_us + 150000;
    } else if (phase == 2 && st == 10) {
        finish(0, "");
    }
    if (st == 11) finish(4, "FAULT state");
    if (phase >= 1 && (rx < -30 || rx > 230 || ry < -30 || ry > 190)) finish(5, "left the maze area");
    if (ms > 400000) finish(6, "timeout");
}

int main(int argc, char **argv) {
    int seed;
    if (argc > 4 && strcmp(argv[1], "probe") == 0) {       // sim probe x y hdg
        int k;
        randomise(getenv("SIM_SEED") ? atoi(getenv("SIM_SEED")) : 1);
        P.p_drop = P.p_spur = 0; P.sonar_noise = 0;
        rx = atof(argv[2]); ry = atof(argv[3]); ra = atof(argv[4]) * DEG;
        for (k = 0; k < 3; k++) printf("%s=%.1f ", k == 0 ? "L" : k == 1 ? "F" : "R", sonar_measure(k));
        printf("\n");
        return 0;
    }
    seed = (argc > 1) ? atoi(argv[1]) : 1;
    int quiet = (argc > 3);
    const char *design = "LLULLRRLLURRLLFLUFL";
    int ok;
    randomise(seed);
    start_pose();
    slog = (argc > 2) ? fopen(argv[2], "w") : NULL;
    WallMem_HostEraseEeprom();

    if (setjmp(done_jmp) == 0) {
        fw_main();
    }
    if (slog) fclose(slog);

    // Pass/fail: the run has to have been CORRECT, not merely finished.
    ok = (result == 0);
    if (ok && strcmp(turns[1], design) != 0) { ok = 0; fail_why = "run-1 decisions differ from the design"; }
    if (ok && n_collapsed_ok != 1) { ok = 0; fail_why = "run-1 log did not collapse to 5"; }
    if (ok && strcmp(turns[2], "LFRRL") != 0)
        { ok = 0; fail_why = "run-2 decisions are not the optimal route"; }
    if (ok && n_mismatch) { ok = 0; fail_why = "replay mismatch"; }
    if (ok && contacts) { ok = 0; fail_why = "robot touched a wall"; }
    if (ok && n_fault_msgs) { ok = 0; fail_why = "fault message in the log"; }
    if (ok && n_wrongway) { ok = 0; fail_why = "a pivot went the wrong way"; }
    if (ok && worst_turn_err > 8.0) { ok = 0; fail_why = "a pivot ended >8 deg off the grid"; }

    if (!quiet || !ok) {
        printf("seed %4d  %-4s run1=%-22s run2=%-8s turns=%2d maxturnerr=%4.1f avg=%4.2f "
               "maxlegerr=%4.1f clear=%4.1f/%4.1f contacts=%d legTO=%d %s\n",
               seed, ok ? "PASS" : "FAIL", turns[1], turns[2], n_turns, worst_turn_err,
               n_turns ? sum_turn_err / n_turns : 0.0, worst_leg_err, min_clear,
               min_clear_turn, contacts, n_legtimeout, ok ? "" : fail_why);
    }
    return ok ? 0 : 1;
}
