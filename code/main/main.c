#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "i2c.h"
#include "mpu6050.h"
#include "heading.h"
#include "motors.h"
#include "sonar.h"
#include "drive.h"
#include "turn.h"
#include "maze.h"
#include "debug.h"
#include "gyrodiag.h"

// ---------------------------------------------------------------------------
//  Build mode. Work through these in order -- each proves one phase on
//  hardware before the next depends on it.
//    0 = sonar telemetry only (no motion)
//    1 = wall centring in a straight corridor
//    2 = single turn accuracy test
//    3 = full maze solver
//    4 = sonar cone characterization -- motors OFF, rotate the chassis BY
//        HAND and watch heading vs L/F/R to find the angle where a beam
//        picks up the wrong wall
//    5 = open-loop square drive test -- no sonar, no corridor: 4x (2s
//        forward + 90 degree turn) to prove raw drive/turn capability
//    6 = turn debugging -- repeats one pivot with a measuring pause after
//        each, streaming the full yaw-rate profile. Nothing but the turn.
//    7 = single obstacle-avoidance cycle -- wall-centred drive straight until
//        the front is blocked, stop, turn right 90, then drive forward
//        open-loop for one fixed leg. One cycle, then halts (not a loop).
//    8 = gyro / I2C connection diagnostic -- motors off, plain-language
//        verdicts, then a live monitor to catch an intermittent link
//    9 = open-space obstacle avoidance -- forward on gyro heading hold (no
//        corridor) until the front sonar sees something, stop, check there is
//        room, pivot right 90, check again, then one more leg
//   10 = dead-end turnaround -- wall-centred drive down a corridor until the
//        front is blocked, then reverse out of the coast, measure both walls,
//        and pivot 180 AWAY from whichever wall is nearer
// ---------------------------------------------------------------------------
#define BUILD_MODE 3

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

static void report_reset_cause(void) {
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
    } else {
        Debug_P("  *** SRAM WAS WIPED -> VCC actually fell to near zero ***\r\n");
        Debug_P("  That is an INTERMITTENT POWER CONNECTION, not a dip:\r\n");
        Debug_P("  battery holder contacts, a VCC/GND jumper, or the buck\r\n");
        Debug_P("  converter dropping out. Note the boot# restarting at 1\r\n");
        Debug_P("  every time is itself the evidence.\r\n");
    }
    Debug_Flush();
    s_prev_flags = f;
}

static void telemetry_header(void) {
#if BUILD_MODE == 4
    Debug_P("# hdg10,L,F,R,Lopen,Ropen,Ltoo,Rtoo,Fblocked,Fvotes");
#elif BUILD_MODE == 2 || BUILD_MODE == 5 || BUILD_MODE == 6
    Debug_P("# no periodic CSV in this mode -- event and summary lines only");
#elif BUILD_MODE == 10
    // The per-tick T stream below carries strictly more than the standard CSV
    // and already runs at 36% of the byte budget. Printing both would drop
    // bytes out of the middle of whichever line lost the race.
    Debug_P("# no periodic CSV in this mode -- see the T trace below");
#else
    Debug_P("# st,L,F,R,lok,rok,near,fv,md,br,err,wt,gt,corr,pwmL,pwmR,rock,rate,gx,gy,ovr,drop");
#endif
    Debug_NL();
}

#if BUILD_MODE == 4
// Mode 4 telemetry: heading (tenths of a degree, accumulated since boot) and
// RAW (not median-filtered) L/F/R so a single bad ping from a beam edge is
// visible rather than smoothed away -- the whole point here is finding
// exactly where the cone edges are, not steering on clean data.
static void telemetry_sonar_test(void) {
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
#endif

#if BUILD_MODE != 4
static void telemetry(int16_t gyro_rate, int16_t gx, int16_t gy, uint16_t overruns) {
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
#endif // BUILD_MODE != 4

#if BUILD_MODE == 10
// ---------------------------------------------------------------------------
//  Mode 10 helpers: dead-end turnaround
// ---------------------------------------------------------------------------
// Approach phases. The two-stage cruise/creep split is what makes the stop
// distance independent of the (unmeasured, speed-dependent) coast.
enum { DEP_WAIT = 0, DEP_CRUISE = 1, DEP_CREEP = 2 };

// Per-tick approach trace. Deliberately CSV rather than prose: this is the
// stream that has to be read numerically afterwards to tell a late stop from a
// phantom obstacle from a centring fault. The prose goes in the event lines.
//
// Field meanings are in the legend printed by deadend_trace_header().
static void deadend_trace(uint32_t t0, uint8_t phase, int16_t rate) {
#if DEADEND_TRACE
    static uint8_t decimate = 0;
    const drive_debug_t *d = Drive_Debug();

    if (++decimate < DEADEND_TRACE_EVERY) return;
    decimate = 0;

    Debug_P("T,");
    Debug_CSV((int32_t)(millis() - t0));
    Debug_CSV(phase);
    Debug_CSV((int32_t)Sonar_Median(SONAR_LEFT));
    Debug_CSV((int32_t)Sonar_Median(SONAR_FRONT));
    Debug_CSV((int32_t)Sonar_Median(SONAR_RIGHT));
    Debug_CSV((int32_t)Sonar_Latest(SONAR_FRONT));
    Debug_CSV(d->l_ok);
    Debug_CSV(d->r_ok);
    Debug_CSV(Sonar_IsTooClose(SONAR_FRONT));
    Debug_CSV(Sonar_IsTooClose(SONAR_LEFT) || Sonar_IsTooClose(SONAR_RIGHT));
    Debug_CSV(Sonar_FrontVotesBelow(DEADEND_STOP_CM));
    Debug_CSV(Sonar_FrontVotesBelow(DEADEND_SLOW_CM));
    Debug_CSV((int32_t)Sonar_FrontGatedCount());
    Debug_CSV(Drive_Mode());
    Debug_CSV(d->branch);
    Debug_CSV(d->error_cm);
    Debug_CSV(d->wall_term);
    Debug_CSV(d->gyro_term);
    Debug_CSV(d->corr);
    Debug_CSV(d->pwm_l);
    Debug_CSV(d->pwm_r);
    Debug_CSV(rate);
    Debug_CSV((Heading_Raw() * 10L) / GYRO_LSB_MS_PER_DEGREE);
    Debug_Int(Debug_Dropped());
    Debug_NL();
#else
    (void)t0; (void)phase; (void)rate;
#endif
}

static void deadend_trace_header(void) {
#if DEADEND_TRACE
    Debug_P("# ---- MODE 10 approach trace ----------------------------\r\n");
    Debug_Flush();
    Debug_P("# T,ms,ph,L,F,R,Fraw,lok,rok,tcF,tcS,vStop,vSlow,gate,md,br,"
            "err,wt,gt,corr,pL,pR,rate,hdg10,drop\r\n");
    Debug_Flush();
    Debug_P("#   ph   1=cruise 2=creep\r\n");
    Debug_P("#   L,F,R   median cm;  Fraw  unfiltered front (999=no echo)\r\n");
    Debug_P("#   lok,rok side reading trusted by the centring controller\r\n");
    Debug_P("#   tcF,tcS front / either side closer than the sensor can"
            " measure\r\n");
    Debug_Flush();
    Debug_P("#   vStop   front votes under DEADEND_STOP_CM (need ");
    Debug_Int((int32_t)FRONT_VOTE_THRESHOLD);
    Debug_P(" of ");
    Debug_Int((int32_t)FRONT_VOTE_WINDOW);
    Debug_P(")\r\n");
    Debug_P("#   vSlow   same, under DEADEND_SLOW_CM -- triggers the creep\r\n");
    Debug_Flush();
    Debug_P("#   gate    front pings DISCARDED by the jump filter, cumulative."
            "\r\n");
    Debug_P("#           Each one costs 60 ms of detection delay, so a rising"
            "\r\n");
    Debug_P("#           gate during the approach is why a stop came late.\r\n");
    Debug_Flush();
    Debug_P("#   md      0=both walls 1=left only 2=right only 3=gyro only\r\n");
    Debug_P("#   br      0=normal 1=rocking 2=emerg-L 3=emerg-R 4=recovering"
            "\r\n");
    Debug_P("#   err     R-L median difference, so TWICE the off-centre offset"
            "\r\n");
    Debug_P("#   wt,gt   wall (P) and gyro (D) contributions; corr = sum,"
            " clamped\r\n");
    Debug_Flush();
    Debug_P("#   pL,pR   PWM actually written. When pR-pL stops equalling"
            "\r\n");
    Debug_P("#           2*corr the correction is being eaten by the PWM"
            " limits\r\n");
    Debug_P("#   rate    raw yaw LSB (65.5 per deg/s); hdg10 heading, 0.1 deg"
            "\r\n");
    Debug_P("#   drop    telemetry bytes lost. If this climbs, raise"
            " DEADEND_TRACE_EVERY\r\n");
    Debug_Flush();
#endif
}

// Everything that shapes this run, so a log can be analysed later without
// having to guess which build produced it. Modelled on Mode 5's header, which
// proved its worth diagnosing the heading-hold D gain.
static void deadend_config_header(void) {
    Debug_P("# ---- MODE 10 dead-end turnaround -----------------------\r\n");
    Debug_P("#   centre the corridor -> front obstacle -> 180 AWAY from the"
            " nearer wall\r\n");
    Debug_Flush();
    Debug_KVF("# corridor", CORRIDOR_WIDTH_CM);
    Debug_KVF("robot_w", ROBOT_WIDTH_CM);
    Debug_KVF("side_gap", CORRIDOR_SIDE_GAP_CM);
    Debug_KVF("emerg", WALL_EMERGENCY_CM);
    Debug_NL();
    Debug_Flush();
    Debug_KVF("# base", DRIVE_BASE_PWM);
    Debug_KVF("creep", DEADEND_CREEP_PWM);
    Debug_KVF("slow_at", DEADEND_SLOW_CM);
    Debug_KVF("stop_at", DEADEND_STOP_CM);
    Debug_NL();
    Debug_Flush();
    Debug_KVF("# brake_pwm", DRIVE_BRAKE_PWM);
    Debug_KVF("brake_ms", DRIVE_BRAKE_MS);
    Debug_KVF("rev_trig", DEADEND_BACKUP_TRIGGER_CM);
    Debug_KVF("rev_target", DEADEND_BACKUP_TARGET_CM);
    Debug_KVF("rev_max_ms", DEADEND_BACKUP_MAX_MS);
    Debug_NL();
    Debug_Flush();
    Debug_KVF("# pivot_side_need", PIVOT_SIDE_NEED_CM);
    Debug_KVF("pivot_front_need", PIVOT_FRONT_NEED_CM);
    Debug_KVF("decide_margin", DEADEND_DECIDE_MARGIN_CM);
    Debug_NL();
    Debug_Flush();
    Debug_KVF("# deadband", WALL_DEADBAND_CM);
    Debug_KVF("kp", WALL_KP_NUM);
    Debug_KVF("/", WALL_KP_DEN);
    Debug_KVF("kd", WALL_KD_NUM);
    Debug_KVF("/", WALL_KD_DEN);
    Debug_KVF("maxcorr", WALL_MAX_CORRECTION);
    Debug_NL();
    Debug_Flush();
    deadend_trace_header();
}
// Give the filters a full set of fresh samples. Every reading in Mode 10 is
// taken after either a coast or a reverse, so the history in the medians was
// gathered somewhere the robot no longer is.
static void deadend_reprime(uint8_t pings) {
    uint8_t i;
    for (i = 0; i < pings; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }
}

// One side wall distance, collapsed to a single number the clearance test can
// compare. The three sonar outcomes mean very different things and the naive
// "just read the median" loses the two that matter most:
//   too close  -> a wall nearer than the sensor can measure. Worst case, so 0.
//   no echo    -> nothing within range. Best case, so the full range.
//   otherwise  -> the median, if it is trustworthy; an untrusted reading is
//                 treated as 0 rather than optimistically believed.
static uint16_t deadend_side_cm(sonar_id_t id) {
    if (Sonar_IsTooClose(id))                 return 0;
    if (Sonar_Latest(id) == SONAR_NO_ECHO)    return SONAR_MAX_RANGE_CM;
    if (!Sonar_IsValid(id))                   return 0;
    return Sonar_Median(id);
}

// "open" reads better than the range ceiling when nothing echoed back, and it
// keeps a genuine 70 cm reading distinguishable from "nothing out there".
static void deadend_print_side(uint16_t cm) {
    if (cm >= SONAR_MAX_RANGE_CM) Debug_P("open");
    else                          Debug_Int((int32_t)cm);
}

// The whole dead-end sequence, run once, blocking, after the approach has
// stopped. Never returns -- Mode 10 is a single-shot test.
static void deadend_sequence(void) {
    turn_result_t r;
    uint16_t l_cm, r_cm, f_cm;
    turn_dir_t dir;
    uint16_t chosen_cm;
    uint8_t  too_close, backed_up;

    Debug_P("MODE10: front blocked -- treating as a DEAD END\r\n");
    Debug_Flush();

    // ---- where did it ACTUALLY stop? ------------------------------------
    // The stop was triggered at DEADEND_STOP_CM, then the creep speed and the
    // active brake between them absorbed the momentum. Whatever difference
    // remains between the two figures below is the RESIDUAL COAST -- the one
    // number that says whether DRIVE_BRAKE_MS and DEADEND_CREEP_PWM are doing
    // their job. It should now be small; if it is not, the brake pulse is too
    // short or the creep too fast.
    Timer_WaitMs(GYRO_SETTLE_MS);
    Sonar_Flush();
    deadend_reprime(12);
    f_cm      = Sonar_Median(SONAR_FRONT);
    too_close = Sonar_IsTooClose(SONAR_FRONT);
    Debug_P("[1] stopped with ");
    if (too_close) Debug_P("a wall TOO CLOSE to measure");
    else {
        Debug_Int((int32_t)f_cm);
        Debug_P(" cm");
    }
    Debug_P(" ahead; asked to stop at ");
    Debug_Int((int32_t)DEADEND_STOP_CM);
    Debug_P(" cm, total stopping distance = ");
    if (too_close) Debug_P("MORE than the whole margin -- it hit the wall");
    else           Debug_Int((int32_t)DEADEND_STOP_CM - (int32_t)f_cm);
    Debug_P("\r\n");
    // That total is NOT all coast, and reading it as coast sends you tuning
    // the wrong constant. It is detection lag plus brake coast. The lag is the
    // front range that goes by between the first ping under the threshold and
    // the second vote landing -- 2 refreshes at 60 ms, so it scales with speed
    // -- and is visible directly in the T trace as the Fraw steps just before
    // STOPPING. Measured at DEADEND_STOP_CM=12: ~6 cm lag, ~2-3 cm coast.
    Debug_P("    (= detection lag + brake coast. Read the last Fraw values in\r\n");
    Debug_P("     the T trace above: the drop from the threshold to the final\r\n");
    Debug_P("     one is the LAG; only the rest is the brake.)\r\n");
    Debug_Flush();

    // ---- back out of the coast, ONLY if it needs to ----------------------
    // The direction rule below solves the LATERAL clearance problem only. The
    // front corners still swing PIVOT_FRONT_RADIUS_CM forward of the axle, so
    // if the coast left the nose against the wall no choice of direction
    // helps -- but if it stopped with room to spare, reversing is wasted
    // travel that only moves the chassis somewhere else in the corridor.
    // A too-close front always reverses: the sensor cannot say how bad it is.
    backed_up = 0;
    if (too_close || f_cm < DEADEND_BACKUP_TRIGGER_CM) {
        uint32_t t0 = millis();
        uint8_t  reached = 0;

        Debug_P("[2] too close to pivot -- reversing until the front reads ");
        Debug_Int((int32_t)DEADEND_BACKUP_TARGET_CM);
        Debug_P(" cm\r\n");
        Debug_Flush();

        // CLOSED LOOP, watching the sensor as it goes. The old fixed 400 ms
        // pulse moved the chassis a measured 20-22 cm for a pivot that needs
        // about 6 -- most of a corridor width thrown away, and a reverse
        // collision waiting to happen in a real maze.
        Motors_SetLeft(DIR_REV, DEADEND_BACKUP_PWM);
        Motors_SetRight(DIR_REV, DEADEND_BACKUP_PWM);
        while ((millis() - t0) < DEADEND_BACKUP_MAX_MS) {
            Sonar_Task();
            if (!Sonar_IsTooClose(SONAR_FRONT) &&
                Sonar_Latest(SONAR_FRONT) != SONAR_NO_ECHO &&
                Sonar_Latest(SONAR_FRONT) >= DEADEND_BACKUP_TARGET_CM) {
                reached = 1;
                break;
            }
            Timer_WaitMs(CONTROL_TICK_MS);
        }
        Motors_Stop();
        Timer_WaitMs(GYRO_SETTLE_MS);
        backed_up = 1;

        Debug_P("    reversed ");
        Debug_Int((int32_t)(millis() - t0));
        Debug_P(" ms, ");
        if (reached) Debug_P("target reached\r\n");
        else         Debug_P("HIT THE TIME LIMIT -- sensor never reported the"
                             " target. Check for a wall behind.\r\n");
        Debug_Flush();
    } else {
        Debug_P("[2] enough room ahead already (");
        Debug_Int((int32_t)f_cm);
        Debug_P(" cm, want ");
        Debug_Int((int32_t)DEADEND_BACKUP_TRIGGER_CM);
        Debug_P("+) -- no reverse needed\r\n");
        Debug_Flush();
    }

    // ---- measure both walls from where the pivot will happen -------------
    // Re-measured even when no reverse ran: the [1] readings were taken
    // before this point either way, and the side sensors have not been read
    // since the chassis stopped moving.
    Sonar_Flush();
    deadend_reprime(12);
    l_cm = deadend_side_cm(SONAR_LEFT);
    r_cm = deadend_side_cm(SONAR_RIGHT);
    f_cm = Sonar_Median(SONAR_FRONT);

    Debug_P("[3] walls: L=");
    deadend_print_side(l_cm);
    Debug_P("  R=");
    deadend_print_side(r_cm);
    Debug_P("  F=");
    Debug_Int((int32_t)f_cm);
    if (backed_up) Debug_P(" cm (after the reverse)\r\n");
    else           Debug_P(" cm\r\n");

    // The front corner grazes the wall at a reading of PIVOT_FRONT_NEED_CM.
    // Warn rather than abort: the pivot may still complete, and knowing it ran
    // this tight is what tells you whether to raise DEADEND_BACKUP_TRIGGER_CM
    // or DEADEND_BACKUP_MS for the next run.
    if (Sonar_IsTooClose(SONAR_FRONT) ||
        (Sonar_IsValid(SONAR_FRONT) && f_cm <= PIVOT_FRONT_NEED_CM)) {
        Debug_P("    WARNING: a front corner will graze the wall at this\r\n");
        Debug_P("    distance. Raise DEADEND_BACKUP_TRIGGER_CM or\r\n");
        Debug_P("    DEADEND_BACKUP_MS, or stop further out.\r\n");
    }
    Debug_Flush();

    // ---- choose the rotation direction ----------------------------------
    // Rotate AWAY from the nearer wall. See turn.h: the front corners sweep
    // PIVOT_FRONT_RADIUS_CM into the side being turned towards while the rear
    // corners only reach ~10.6 cm out the other side, so the turning side
    // needs roughly 6 cm more room. Hugging the left wall and rotating left
    // drags the wide front corner into it; rotating right puts only the
    // narrow rear corner there.
    if (l_cm > r_cm && (uint16_t)(l_cm - r_cm) >= DEADEND_DECIDE_MARGIN_CM) {
        dir = TURN_LEFT;                  // more room on the left -> go left
        chosen_cm = l_cm;
        Debug_P("[4] nearer the RIGHT wall -> rotating LEFT, away from it\r\n");
    } else if (r_cm > l_cm && (uint16_t)(r_cm - l_cm) >= DEADEND_DECIDE_MARGIN_CM) {
        dir = TURN_RIGHT;
        chosen_cm = r_cm;
        Debug_P("[4] nearer the LEFT wall -> rotating RIGHT, away from it\r\n");
    } else {
        // Within sonar noise of centred. "Nearer wall" is a coin flip here,
        // so pick the fixed default rather than chase the noise.
        dir = DEADEND_TIE_DIR;
        chosen_cm = (dir == TURN_LEFT) ? l_cm : r_cm;
        Debug_P("[4] centred within ");
        Debug_Int((int32_t)DEADEND_DECIDE_MARGIN_CM);
        Debug_P(" cm -- no near wall, using the default direction: ");
        if (dir == TURN_LEFT) Debug_P("LEFT\r\n");
        else                  Debug_P("RIGHT\r\n");
    }
    Debug_Flush();

    // ---- will the chosen side actually take it? --------------------------
    if (chosen_cm < PIVOT_SIDE_NEED_CM) {
        uint16_t other_cm = (dir == TURN_LEFT) ? r_cm : l_cm;
        if (other_cm >= PIVOT_SIDE_NEED_CM) {
            // Only reachable from the tie branch, which may have defaulted
            // into the tighter of two near-equal sides.
            dir = (dir == TURN_LEFT) ? TURN_RIGHT : TURN_LEFT;
            chosen_cm = other_cm;
            Debug_P("    that side is too tight -- switching to the other one\r\n");
        } else {
            Debug_P("    NEITHER SIDE HAS ROOM. Need ");
            Debug_Int((int32_t)PIVOT_SIDE_NEED_CM);
            Debug_P(" cm clear on the turning side; L=");
            deadend_print_side(l_cm);
            Debug_P(" R=");
            deadend_print_side(r_cm);
            Debug_P("\r\n");
            Debug_P("    Refusing the pivot -- it would grind a front corner\r\n");
            Debug_P("    along the wall. This corridor is too narrow for an\r\n");
            Debug_P("    in-place 180 at this chassis size.\r\n");
            Debug_P("MODE10 ABORTED\r\n");
            Debug_Flush();
            Motors_Stop();
            for (;;) { }
        }
        Debug_Flush();
    }

    // ---- turn around -----------------------------------------------------
    Debug_P("[5] 180 degrees, rotating ");
    if (dir == TURN_LEFT) Debug_P("LEFT");
    else                  Debug_P("RIGHT");
    Debug_P(" (");
    Debug_Int((int32_t)chosen_cm);
    Debug_P(" cm clear that side, need ");
    Debug_Int((int32_t)PIVOT_SIDE_NEED_CM);
    Debug_P(")\r\n");
    Debug_Flush();

    Turn_180(dir, &r);

    Debug_P("    ");
    Debug_KVF("ang10", r.achieved_tenths);
    Debug_KVF("fin10", r.final_error_tenths);
    Debug_KVF("conv", r.converged);
    Debug_KVF("nudge", r.nudges_used);
    Debug_KVF("wrong", r.wrong_way);
    Debug_NL();
    Debug_Flush();

    // ---- what is ahead now? ---------------------------------------------
    // Turn_Execute() flushed the sonar, so this needs its own priming. The
    // way out should be open; anything short means the 180 fell well short
    // of 180 and the robot is looking at a side wall.
    deadend_reprime(12);
    f_cm = Sonar_Median(SONAR_FRONT);
    Debug_P("[6] way out ahead: ");
    if (Sonar_Latest(SONAR_FRONT) == SONAR_NO_ECHO) {
        Debug_P("clear beyond sensor range -- the turn worked\r\n");
    } else {
        Debug_Int((int32_t)f_cm);
        Debug_P(" cm\r\n");
        if (Sonar_IsValid(SONAR_FRONT) && f_cm < FRONT_BLOCKED_CM) {
            Debug_P("    STILL BLOCKED. Either the 180 fell short and this is\r\n");
            Debug_P("    a side wall, or the chassis is wedged in the corner.\r\n");
        }
    }
    Debug_P("MODE10 DONE\r\n");
    Debug_Flush();
    Motors_Stop();
    for (;;) { }
}
#endif // BUILD_MODE == 10

int main(void) {
    uint32_t next_tick;
    uint32_t run_start;

    // MOTORS OFF FIRST -- before the UART, the timer, anything.
    //
    // A reset does NOT stop the motors. It makes every port pin a high-Z input,
    // so the L298N's direction inputs float and its last commanded state can
    // persist: the chassis keeps driving, or keeps pivoting, until firmware
    // takes the pins back. That is the "kept rotating after the turn" and "kept
    // rotating 360" symptom in the Mode 10 logs -- the MCU browned out mid-
    // pivot and the motors simply carried on.
    //
    // This used to run after Debug_Init/Timer_Init/I2C_Init, which is
    // milliseconds of unguided motion per reset, and much worse in a repeated
    // brown-out loop where the code may never reach the old position at all.
    // Nothing here depends on any other subsystem, so it costs nothing to make
    // it the first thing that happens.
    Motors_Init();
    Motors_Stop();

    Debug_Init();
    Timer_Init();          // before sei() so millis() is live immediately
    I2C_Init();
    Sonar_Init();
    sei();

    Debug_P("\r\n=== AGV maze solver ===\r\n");
    report_reset_cause();

#if BUILD_MODE == 8
    // Deliberately BEFORE MPU6050_Init() and Gyro_CalibrateFull(). Calibration
    // is 500 reads on the unprotected I2C path, so on a dead bus the firmware
    // hangs there and never reaches a diagnostic placed later. This runs on a
    // cold bus, does its own init, and never returns. The reset cause above is
    // printed first because BROWNOUT is the prime suspect for a flaky gyro.
    GyroDiag_Run();
#endif

    MPU6050_Init();
    Debug_P("calibrating gyro, hold still...\r\n");
    Gyro_CalibrateFull();
    Debug_KVF("offZ", Gyro_GetOffset());
    Debug_KVF("offX", Gyro_GetOffsetX());
    Debug_KVF("offY", Gyro_GetOffsetY());
    Debug_NL();

    Maze_Init();
    telemetry_header();
#if BUILD_MODE == 10
    deadend_config_header();
#endif
    next_tick = millis();
    run_start = millis();

#if BUILD_MODE == 2
    {
        turn_result_t r;
        Timer_WaitMs(STARTUP_DELAY_MS);
        Turn_90(TURN_RIGHT, &r);
        Debug_P("turn test ");
        Debug_KVF("ang10", r.achieved_tenths);
        // err10 > 0: the fixed early-stop + coast undershot this run, needed
        // more rotation. err10 < 0: it overshot, needed a reverse nudge.
        // Consistently one sign across repeated runs -> retune
        // TURN_STOP_MARGIN_DEG in that direction.
        Debug_KVF("err10", r.initial_error_tenths);
        Debug_KVF("nudges", r.nudges_used);
        // peak near 32767 = the gyro clipped at +/-500 dps, so the heading
        // under-read and the chassis physically overshot. wrong=1 = it rotated
        // the opposite way to the one commanded.
        Debug_KVF("peak", r.peak_rate);
        Debug_KVF("wrong", r.wrong_way);
        Debug_NL();
        Motors_Stop();
        for (;;) { }
    }
#elif BUILD_MODE == 5
    {
        uint8_t leg;
        int32_t carry = 0;      // previous turn's leftover, in this leg's frame
        int32_t total_raw = 0;  // rotation over the WHOLE path, legs and turns

        // Self-describing header: every tunable that shapes this run, so the
        // log can be analysed later without having to guess the build.
        //
        // Debug_Flush() after EVERY line, not once at the end. tx_push() drops
        // when the ring buffer is full, so the loss happens during the pushes
        // -- a flush afterwards is far too late. A measured run lost 213 bytes
        // of this header with a single trailing flush.
        Debug_P("# SQUARE TEST  sides=");   Debug_Int(SQUARE_SIDES);
        Debug_P(" pwm=");                   Debug_Int(SQUARE_TEST_PWM);
        Debug_P(" legms=");                 Debug_Int(SQUARE_LEG_MS);
        Debug_NL();                           Debug_Flush();
        Debug_P("# hold: tick=");           Debug_Int(HOLD_TICK_MS);
        Debug_P(" kp=");                    Debug_Int(HOLD_KP_NUM);
        Debug_P("/");                       Debug_Int(HOLD_KP_DEN);
        Debug_P(" kd=");                    Debug_Int(HOLD_KD_NUM);
        Debug_P("/");                       Debug_Int(HOLD_KD_DEN);
        Debug_NL();                           Debug_Flush();
        Debug_P("# lim: maxcorr=");         Debug_Int(WALL_MAX_CORRECTION);
        Debug_P(" pwmfloor=");              Debug_Int(MOTOR_MIN_PWM);
        Debug_P(" pwmceil=");               Debug_Int(MOTOR_MAX_PWM);
        Debug_NL();                           Debug_Flush();
        Debug_P("# turn: margin=");         Debug_Int(TURN_STOP_MARGIN_DEG);
        Debug_P(" deadband=");              Debug_Int(TURN_DEADBAND_DEG);
        Debug_P(" settle=");                Debug_Int(TURN_SETTLE_MS);
        Debug_P(" pwm=");                   Debug_Int(TURN_PWM);
        Debug_NL();                           Debug_Flush();
        Debug_P("# gyro: lsbms_per_deg=");  Debug_Int(GYRO_LSB_MS_PER_DEGREE);
        Debug_NL();                           Debug_Flush();
        Debug_P("# L,ms,err10,rate,corr,pwmL,pwmR  <- heading-hold sample\r\n");
        Debug_Flush();
        Debug_P("# S,ms,rate,hdg10                 <- turn sample\r\n");
        Debug_Flush();

#if SQUARE_TRACE_TURNS
        Turn_SampleTrace(1);
#endif
        Timer_WaitMs(STARTUP_DELAY_MS);

        for (leg = 0; leg < SQUARE_SIDES; leg++) {
            turn_result_t r;
            int32_t net_raw;

            Debug_P("=== leg "); Debug_Int(leg + 1); Debug_P(" forward ===\r\n");
            // Gyro heading hold, NOT open-loop. Two motors at equal PWM never
            // track straight (gearbox, tyre and friction mismatch), which is
            // why the original firmware had a straight-line autocorrect at
            // all. `carry` feeds the previous turn's residual in so the leg
            // steers it out rather than baking it into the path.
            net_raw    = Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS, carry);
            total_raw += net_raw;
            Timer_WaitMs(SQUARE_TURN_SETTLE_MS);

            Debug_P("=== leg "); Debug_Int(leg + 1); Debug_P(" turn ===\r\n");
            Turn_90(TURN_RIGHT, &r);
            // A right turn rotates negative, so subtract it from the running
            // total: four clean corners plus four straight legs must land on
            // -3600 tenths.
            total_raw -= (int32_t)r.achieved_tenths * GYRO_LSB_MS_PER_DEGREE / 10;

            Debug_P("  TURN ");
            Debug_KVF("ang10",  r.achieved_tenths);
            Debug_KVF("err10",  r.initial_error_tenths);
            Debug_KVF("fin10",  r.final_error_tenths);
            Debug_KVF("conv",   r.converged);
            Debug_KVF("nudges", r.nudges_used);
            Debug_KVF("peak",   r.peak_rate);
            Debug_KVF("coastms", r.coast_ms);
            Debug_KVF("wrong",  r.wrong_way);
            Debug_KVF("recal",  r.recal_ok);
            Debug_KVF("to",     r.timed_out);
            Debug_NL();
            Debug_P("  CUM ");
            Debug_KVF("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
            Debug_NL();

            // Hand this turn's leftover to the next leg instead of discarding
            // it -- Turn_Execute() zeroes the accumulator, so without this the
            // residual from every corner accumulates into the square.
            carry = r.residual_raw;
        }

        // No end-of-run table: the live HOLD/TURN/CUM lines above already carry
        // every number, and buffering them into arrays to replay at the end
        // pushed ~250 bytes into a 192-byte ring buffer in one burst, which
        // came back garbled and untrustworthy.
        //
        // This is the one number worth restating: whether the square closed in
        // HEADING. -3600 tenths is a perfect four-corner circuit. Position can
        // still be off even at -3600, since heading hold does not correct
        // sideways displacement and battery sag shortens the later legs.
        Debug_NL();
        Debug_Flush();
        Debug_P("SQUARE TOTAL ");
        Debug_KVF("total10", (total_raw * 10L) / GYRO_LSB_MS_PER_DEGREE);
        Debug_KVF("ideal10", -3600L);
        Debug_KVF("drop", Debug_Dropped());
        Debug_NL();
        Debug_Flush();
        Debug_P("SQUARE TEST DONE\r\n");
        Debug_Flush();
        Motors_Stop();
        for (;;) { }
    }
#elif BUILD_MODE == 6
    {
        uint8_t n;

        Turn_SampleTrace(1);

        Debug_P("# TURN DEBUG  angle=");   Debug_Int(TURNDBG_ANGLE);
        Debug_P(" repeats=");              Debug_Int(TURNDBG_REPEATS);
        Debug_P(" alt=");                  Debug_Int(TURNDBG_ALTERNATE);
        Debug_P(" decim=");                Debug_Int(TURNDBG_SAMPLE_EVERY);
        Debug_NL();
        Debug_P("# S,ms,rate,hdg10  <- per-sample stream (rate is raw LSB, "
                  "65.5 per deg/sec)\r\n");
        Debug_P("# T <phase> hdg10=..    <- phase boundary\r\n");
        Debug_P("# SUM ..                <- per-turn summary\r\n");
        Debug_P("# tape a reference line on the floor, protractor each turn "
                  "during the pause\r\n");
        // These header lines are ~350 bytes pushed back to back, which is more
        // than DEBUG_TX_BUF (192) can hold while the UART drains at 38400 --
        // a measured run lost 165 bytes of them and garbled the legend. Safe
        // to spin here: nothing is moving yet.
        Debug_Flush();
        Timer_WaitMs(STARTUP_DELAY_MS);

        for (n = 0; n < TURNDBG_REPEATS; n++) {
            turn_result_t r;
            turn_dir_t dir = (TURNDBG_ALTERNATE && (n & 1)) ? TURN_LEFT : TURN_RIGHT;

            Debug_P("=== turn ");
            Debug_Int(n + 1);
            if (dir == TURN_RIGHT) Debug_P(" RIGHT ===\r\n");
            else                   Debug_P(" LEFT ===\r\n");

            Turn_Execute(TURNDBG_ANGLE, dir, &r);

            Debug_P("SUM ");
            Debug_KVF("n",       n + 1);
            Debug_KVF("dirR",    (dir == TURN_RIGHT) ? 1 : 0);
            Debug_KVF("ang10",   r.achieved_tenths);
            Debug_KVF("err10",   r.initial_error_tenths);
            Debug_KVF("nudges",  r.nudges_used);
            Debug_KVF("peak",    r.peak_rate);
            Debug_KVF("coastms", r.coast_ms);
            // fin10 is what was still left when the nudge loop stopped, and
            // conv=0 means it ran out of nudges before reaching the deadband.
            Debug_KVF("fin10",   r.final_error_tenths);
            Debug_KVF("conv",    r.converged);
            Debug_KVF("wrong",   r.wrong_way);
            Debug_KVF("recal",   r.recal_ok);
            Debug_KVF("to",      r.timed_out);
            // Cumulative since boot, so compare it turn to turn: any increase
            // means the per-sample stream lost bytes during that turn and its
            // S lines cannot be trusted -- raise TURNDBG_SAMPLE_EVERY.
            Debug_KVF("drop",    Debug_Dropped());
            Debug_NL();

            Debug_P("measure the angle now\r\n");
            Timer_WaitMs(TURNDBG_PAUSE_MS);
        }

        Debug_P("TURN DEBUG DONE\r\n");
        Motors_Stop();
        for (;;) { }
    }
#elif BUILD_MODE == 9
    {
        turn_result_t r;
        uint8_t  blocked = 0, i;
        uint16_t front_cm, side_cm;

        Debug_P("# OBSTACLE AVOID -- open space, no corridor needed.\r\n");
        Debug_P("#   forward on gyro heading hold -> stop at obstacle ->\r\n");
        Debug_P("#   check clearance -> right 90 -> check again -> one leg\r\n");
        Debug_Flush();
        Debug_P("# needs ");
        Debug_Int((int32_t)AVOID_TURN_CLEARANCE_CM);
        Debug_P(" cm clear to the RIGHT of the obstacle, and stops ");
        Debug_Int((int32_t)AVOID_PIVOT_CLEARANCE_CM);
        Debug_P("+ cm short of it\r\n");
        Debug_Flush();

        Timer_WaitMs(STARTUP_DELAY_MS);

        // Prime the filters. Medians need HIST samples and the front vote
        // window needs history before either reading means anything -- without
        // this the clearance checks below would run on empty history.
        for (i = 0; i < 12; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }

        // ---- leg 1: forward until something is ahead ----------------------
        Debug_P("[1] driving forward, watching the front sensor\r\n");
        Debug_Flush();
        Drive_StraightUntilBlocked(SQUARE_TEST_PWM, AVOID_APPROACH_MAX_MS,
                                   0, &blocked);
        front_cm = Sonar_Median(SONAR_FRONT);

        if (!blocked) {
            Debug_P("    nothing found before the time limit -- stopping.\r\n");
            Debug_P("OBSTACLE AVOID ABORTED\r\n");
            Debug_Flush();
            Motors_Stop();
            for (;;) { }
        }
        Debug_P("    OBSTACLE found, stopped with ");
        Debug_Int((int32_t)front_cm);
        Debug_P(" cm showing on the front sensor\r\n");
        Debug_Flush();

        // ---- clearance gate: is there room to turn INTO? ------------------
        // Checked before the pivot so a blocked right side costs nothing but a
        // message. The reading is from the RIGHT sensor, which is pointing
        // where the robot is about to travel.
        Timer_WaitMs(GYRO_SETTLE_MS);
        for (i = 0; i < 9; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }
        side_cm  = Sonar_Median(SONAR_RIGHT);
        front_cm = Sonar_Median(SONAR_FRONT);

        // Room to pivot without clipping the obstacle. Re-measured AFTER the
        // stop on purpose: drive.c has no active brake, and a previous run
        // coasted 17cm past its stop point, so the distance that triggered the
        // stop is not the distance the robot actually ended up at.
        Debug_P("[2a] room to pivot: ");
        Debug_Int((int32_t)front_cm);
        Debug_P(" cm after coasting (need ");
        Debug_Int((int32_t)AVOID_PIVOT_CLEARANCE_CM);
        Debug_P(")\r\n");
        if (Sonar_IsTooClose(SONAR_FRONT) ||
            (Sonar_IsValid(SONAR_FRONT) && front_cm < AVOID_PIVOT_CLEARANCE_CM)) {
            Debug_P("    TOO CLOSE to pivot -- the front corners would clip it\r\n");
            Debug_P("    on the way round. Back the robot off and re-run.\r\n");
            Debug_P("OBSTACLE AVOID ABORTED\r\n");
            Debug_Flush();
            Motors_Stop();
            for (;;) { }
        }
        Debug_Flush();

        Debug_P("[2b] clearance to the right: ");
        if (Sonar_Latest(SONAR_RIGHT) == SONAR_NO_ECHO) {
            Debug_P("beyond sensor range -- open, good\r\n");
        } else {
            Debug_Int((int32_t)side_cm);
            Debug_P(" cm (need ");
            Debug_Int((int32_t)AVOID_TURN_CLEARANCE_CM);
            Debug_P(")\r\n");
            if (!Sonar_IsValid(SONAR_RIGHT) ||
                side_cm < AVOID_TURN_CLEARANCE_CM) {
                Debug_P("    NOT ENOUGH ROOM to the right. Refusing to turn --\r\n");
                Debug_P("    the second leg would drive into it. Move the\r\n");
                Debug_P("    obstacle further from the side wall and re-run.\r\n");
                Debug_P("OBSTACLE AVOID ABORTED\r\n");
                Debug_Flush();
                Motors_Stop();
                for (;;) { }
            }
        }
        Debug_Flush();

        // ---- pivot --------------------------------------------------------
        Debug_P("[3] turning right 90\r\n");
        Debug_Flush();
        Turn_90(TURN_RIGHT, &r);
        Debug_P("    ");
        Debug_KVF("ang10", r.achieved_tenths);
        Debug_KVF("fin10", r.final_error_tenths);
        Debug_KVF("conv", r.converged);
        Debug_KVF("wrong", r.wrong_way);
        Debug_NL();
        Debug_Flush();

        // ---- final gate: the path actually ahead now ----------------------
        // Turn_Execute() flushed the sonar, so this has to re-prime. This is
        // the authoritative check: the front sensor is now pointing down the
        // path the robot is about to drive, rather than inferring it from a
        // side reading taken before the pivot.
        for (i = 0; i < 12; i++) { Sonar_Task(); Timer_WaitMs(CONTROL_TICK_MS); }
        front_cm = Sonar_Median(SONAR_FRONT);
        Debug_P("[4] path ahead after the turn: ");
        if (Sonar_Latest(SONAR_FRONT) == SONAR_NO_ECHO) {
            Debug_P("clear beyond sensor range\r\n");
        } else {
            Debug_Int((int32_t)front_cm);
            Debug_P(" cm\r\n");
            if (Sonar_IsValid(SONAR_FRONT) && front_cm < AVOID_TURN_CLEARANCE_CM) {
                Debug_P("    TOO CLOSE to drive the full leg. Stopping here\r\n");
                Debug_P("    rather than running into it.\r\n");
                Debug_P("OBSTACLE AVOID ABORTED\r\n");
                Debug_Flush();
                Motors_Stop();
                for (;;) { }
            }
        }
        Debug_Flush();

        // ---- leg 2 --------------------------------------------------------
        Debug_P("[5] driving the final leg\r\n");
        Debug_Flush();
        Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS, r.residual_raw);

        Debug_P("OBSTACLE AVOID DONE\r\n");
        Debug_Flush();
        Motors_Stop();
        for (;;) { }
    }
#endif

    for (;;) {
        gyro_xyz_t g;
        int16_t rate;
        uint32_t tick_start;
        static uint16_t overruns = 0;

        // ---- fixed control tick -----------------------------------------
        if ((int32_t)(millis() - next_tick) < 0) continue;
        next_tick += CONTROL_TICK_MS;
        tick_start = millis();
        // If we fell badly behind (a long sonar timeout, say), re-base rather
        // than firing a burst of catch-up ticks with no spacing.
        if ((int32_t)(millis() - next_tick) > (int32_t)(CONTROL_TICK_MS * 3)) {
            next_tick = millis() + CONTROL_TICK_MS;
        }

        // ---- sensors -----------------------------------------------------
        MPU6050_ReadAll(&g);
        Motion_Update(&g);                     // pitch/roll -> rocking flag
        Heading_Add(g.z, CONTROL_TICK_MS);
        rate = Gyro_Rate(g.z);

        Sonar_Task();                          // exactly one ping per tick

        // ---- behaviour ---------------------------------------------------
#if BUILD_MODE == 0 || BUILD_MODE == 4
        // Mode 4: motors permanently off. Sonar/heading above still run every
        // tick, so rotating the chassis by hand is exactly what the cone test
        // needs -- only the telemetry format differs (see below).
        Motors_Stop();
        (void)rate;
#elif BUILD_MODE == 1
        {
            static uint8_t begun      = 0;
            static uint8_t stopped    = 0;
            static uint8_t block_hits = 0;

            if (!begun && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                begun = 1;
            }

            if (begun && !stopped) {
                // Debounced the same way junction detection is in maze.c --
                // a single bad ping should not slam the brakes mid-test.
                if (Sonar_FrontBlocked()) {
                    if (block_hits < 255) block_hits++;
                } else {
                    block_hits = 0;
                }

                if (block_hits >= FRONT_STOP_CONFIRM) {
                    Drive_Stop();
                    stopped = 1;
                    Debug_P("MODE1: front obstacle -- stopped\r\n");
                } else {
                    Drive_Tick(rate);
                }
            }
            // once stopped, motors stay off; telemetry keeps printing below
        }
#elif BUILD_MODE == 7
        {
            static uint8_t state      = 0;   // 0 not started, 1 approaching, 2 done
            static uint8_t block_hits = 0;

            if (state == 0 && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                state = 1;
                Debug_P("MODE7: approaching obstacle\r\n");
            }

            if (state == 1) {
                // Same debounce as Mode 1 -- a single bad ping should not
                // trigger the turn early.
                if (Sonar_FrontBlocked()) {
                    if (block_hits < 255) block_hits++;
                } else {
                    block_hits = 0;
                }

                if (block_hits >= FRONT_STOP_CONFIRM) {
                    turn_result_t r;

                    Drive_Stop();
                    Debug_P("MODE7: obstacle detected, stopping\r\n");

                    // Blocking, same as maze.c's own turn handling -- Turn_90
                    // already flushes the sonar filters and resets heading
                    // when it returns, so nothing pointed the wrong way
                    // carries into the leg below.
                    Debug_P("MODE7: turning right\r\n");
                    Turn_90(TURN_RIGHT, &r);
                    Debug_KVF("ang10", r.achieved_tenths);
                    Debug_KVF("wrong", r.wrong_way);
                    Debug_NL();

                    // Forward leg under gyro heading hold, same as Mode 5's
                    // legs, carrying the turn's leftover error in so it gets
                    // steered out rather than baked into the heading.
                    Debug_P("MODE7: forward leg\r\n");
                    Drive_StraightHold(SQUARE_TEST_PWM, SQUARE_LEG_MS,
                                       r.residual_raw);

                    Debug_P("MODE7 DONE\r\n");
                    state = 2;
                } else {
                    Drive_Tick(rate);
                }
            }
            // state 2: motors stay off; telemetry keeps printing below
        }
#elif BUILD_MODE == 10
        {
            static uint8_t  phase       = DEP_WAIT;
            static uint8_t  block_hits  = 0;
            static uint32_t drive_start = 0;
            static uint32_t want_stop_since = 0;  // 0 = not wanting to stop yet
            uint8_t  stop_now  = 0;
            uint8_t  tc_front  = Sonar_IsTooClose(SONAR_FRONT);
            uint8_t  forced    = 0;

            if (phase == DEP_WAIT && millis() - run_start > STARTUP_DELAY_MS) {
                Drive_Begin();
                drive_start = millis();      // time the DRIVE, not the boot
                phase = DEP_CRUISE;
                Debug_P("MODE10: [cruise] driving the corridor, centring on"
                        " both walls\r\n");
            }

            if (phase == DEP_CRUISE || phase == DEP_CREEP) {
                // ---- stage 1 -> stage 2: slow down well before stopping ----
                // Coast at cruise is larger than the whole stopping margin, so
                // the last stretch has to be done slowly. This transition must
                // happen further out than the CRUISE coast, hence the generous
                // DEADEND_SLOW_CM.
                if (phase == DEP_CRUISE &&
                    Sonar_FrontCloserThan(DEADEND_SLOW_CM)) {
                    phase = DEP_CREEP;
                    Debug_P("MODE10: [creep] obstacle within ");
                    Debug_Int((int32_t)DEADEND_SLOW_CM);
                    Debug_P(" cm -- dropping to PWM ");
                    Debug_Int((int32_t)DEADEND_CREEP_PWM);
                    Debug_P(" so the coast is short\r\n");
                }

                // ---- the stop decision ------------------------------------
                // A too-close front bypasses everything below: the wall is
                // nearer than the sensor can even measure, so there is nothing
                // left to confirm or wait for.
                if (tc_front) {
                    stop_now = 1;
                    forced   = 1;
                    Debug_P("MODE10: front TOO CLOSE to measure -- emergency"
                            " stop, no confirmation wait\r\n");
                } else {
                    // DEADEND_STOP_CM, not FRONT_BLOCKED_CM: this closes in
                    // much nearer than the junction classifier does, without
                    // changing what counts as a junction for Mode 3. Same vote
                    // window, so a yaw that swings the beam off the wall still
                    // cannot hide it. Debounced on top of that.
                    if (Sonar_FrontCloserThan(DEADEND_STOP_CM)) {
                        if (block_hits < 255) block_hits++;
                    } else {
                        block_hits = 0;
                    }

                    if (block_hits >= FRONT_STOP_CONFIRM) {
                        // ---- straightness gate ----------------------------
                        // While the chassis is yawing the front beam can be
                        // ranging a SIDE wall, and stopping on that reading
                        // invents a dead end in the middle of a clear
                        // corridor. Wait for a straight moment -- but not
                        // forever, or a delayed stop becomes a collision.
                        int16_t ar = (rate < 0) ? (int16_t)-rate : rate;
                        if (want_stop_since == 0) {
                            want_stop_since = millis();
                            Debug_P("MODE10: front inside ");
                            Debug_Int((int32_t)DEADEND_STOP_CM);
                            Debug_P(" cm -- confirming while straight\r\n");
                        }
                        if (ar < DEADEND_STRAIGHT_LSB) {
                            stop_now = 1;
                        } else if ((millis() - want_stop_since) >
                                   DEADEND_STRAIGHT_MAX_MS) {
                            stop_now = 1;
                            forced   = 1;
                            Debug_P("MODE10: still yawing after ");
                            Debug_Int((int32_t)DEADEND_STRAIGHT_MAX_MS);
                            Debug_P(" ms -- taking the stop anyway. The front"
                                    " reading may be a side wall.\r\n");
                        }
                    } else {
                        want_stop_since = 0;
                    }
                }

                if (stop_now) {
                    Debug_P("MODE10: STOPPING (");
                    if (forced) Debug_P("forced");
                    else        Debug_P("straight and confirmed");
                    Debug_P(")\r\n");
                    deadend_trace(drive_start, phase, rate);
                    Drive_Stop();               // brakes, then releases
                    deadend_sequence();         // blocking, never returns
                } else if ((millis() - drive_start) > DEADEND_APPROACH_MAX_MS) {
                    Drive_Stop();
                    Debug_P("MODE10: no dead end found within the time"
                            " limit\r\n");
                    Debug_P("MODE10 ABORTED\r\n");
                    Debug_Flush();
                    Motors_Stop();
                    for (;;) { }
                } else {
                    if (phase == DEP_CREEP) Drive_TickAt(DEADEND_CREEP_PWM, rate);
                    else                    Drive_Tick(rate);
                    deadend_trace(drive_start, phase, rate);
                }
            }
        }
#else
        Maze_Tick(rate);
#endif

        // Measure how long the tick's real work took. If this exceeds the
        // budget the control loop is no longer running at a fixed rate, which
        // silently invalidates the gyro integration and the PD tuning.
        if ((millis() - tick_start) > TICK_OVERRUN_WARN_MS) {
            if (overruns < 0xFFFF) overruns++;
        }

#if BUILD_MODE == 4
        telemetry_sonar_test();
#elif BUILD_MODE == 10
        // Mode 10 traces every tick itself, right after the control decision
        // it describes. A second periodic stream here would both duplicate it
        // and blow the byte budget.
        (void)g;
#else
        telemetry(rate, g.x, g.y, overruns);
#endif

        // ---- global safety ----------------------------------------------
        if ((millis() - run_start) > MAX_RUN_MS) {
            Motors_Stop();
            Debug_P("RUN LIMIT\r\n");
            for (;;) { }
        }
    }
}
