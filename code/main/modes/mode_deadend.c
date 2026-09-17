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

// Build mode 10 -- treat a front obstacle as a dead end and turn around,
// choosing the rotation direction from which wall is nearer.
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
    Debug_CSV((int32_t)Power_LastMv());
    Debug_CSV((int32_t)Power_MinMv());
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
            "err,wt,gt,corr,pL,pR,rate,hdg10,vcc,vmin,drop\r\n");
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
    Debug_Flush();
    Debug_P("#   vcc     supply mV this tick; vmin lowest since the run began."
            "\r\n");
    Debug_P("#           Bandgap-derived, so the ABSOLUTE value may be ~10%"
            " out --\r\n");
    Debug_P("#           what matters is the DROP. A vmin far below the idle"
            "\r\n");
    Debug_P("#           reading printed at boot means the rail is collapsing"
            "\r\n");
    Debug_P("#           under motor current, which is what resets the MCU."
            "\r\n");
    Debug_P("#           Below ");
    Debug_Int((int32_t)POWER_MIN_SAFE_MV);
    Debug_P(" mV the part is out of spec at 16 MHz.\r\n");
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
        Power_SetActivity(ACT_REVERSING);
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
    // ---- supply verdict --------------------------------------------------
    // Printed on every run, success included. The runs that SURVIVE are the
    // ones that say how much margin is left: a minimum that sits just above
    // POWER_MIN_SAFE_MV is a run that got lucky, not a run that was fine.
    Debug_P("[7] supply: min ");
    Debug_Int((int32_t)Power_MinMv());
    Debug_P(" mV during the run");
    if (Power_SagSeen()) {
        Debug_P("\r\n    *** WENT BELOW ");
        Debug_Int((int32_t)POWER_MIN_SAFE_MV);
        Debug_P(" mV -- out of spec for 16 MHz. This run\r\n");
        Debug_P("    came close to the brown-out that killed the others."
                " Fix the\r\n");
        Debug_P("    supply before trusting any result.\r\n");
    } else {
        Debug_P(", stayed in spec\r\n");
    }
    Debug_Flush();

    // Clean finish: clear the in-motion flag so the next boot is treated as a
    // normal start rather than an interrupted run.
    ResetLog_MarkIdle();

    Debug_P("MODE10 DONE\r\n");
    Debug_Flush();
    Motors_Stop();
    for (;;) { }
}

void Mode_PreGyro(void) { }
void Mode_Begin(void)   { }

void Mode_Header(void) {
    Debug_P("# no periodic CSV in this mode -- see the T trace below");
    Debug_NL();
    deadend_config_header();
}

void Mode_Tick(const tick_ctx_t *t) {
    int16_t  rate      = t->rate;
    uint32_t run_start = t->run_start;
    (void)rate; (void)run_start;
        {
            static uint8_t  phase       = DEP_WAIT;
            static uint8_t  block_hits  = 0;
            static uint32_t drive_start = 0;
            static uint32_t want_stop_since = 0;  // 0 = not wanting to stop yet
            uint8_t  stop_now  = 0;
            uint8_t  tc_front  = Sonar_IsTooClose(SONAR_FRONT);
            uint8_t  forced    = 0;

            if (phase == DEP_WAIT && millis() - run_start > STARTUP_DELAY_MS) {
                // Mark the chassis as in motion BEFORE the first motor command,
                // so a reset during the very first kick is still caught. Lives
                // in .noinit, so it survives the reset that matters.
                ResetLog_MarkMoving();
                Power_ResetMin();        // baseline the sag from here
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
}

void Mode_Telemetry(const tick_ctx_t *t) { (void)t; }
