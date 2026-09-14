#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
//  CENTRAL CONFIGURATION
//  Every tunable number in the project lives here. Nothing else should
//  contain a magic constant.
//
//  Values marked [MEASURE] are placeholders chosen by estimate -- you must
//  replace them with real measurements from your robot/maze before the
//  behaviour will be correct. See README_TUNING.md.
// ============================================================================

#define F_CPU 16000000UL

// ---------------------------------------------------------------------------
//  1. PIN ALLOCATION   (from your CSE 315 notes pin map)
// ---------------------------------------------------------------------------
// PORTA -- Sonar array (all three mounted PERPENDICULAR, not angled)
#define SONAR_PORT      PORTA
#define SONAR_DDR       DDRA
#define SONAR_PIN       PINA
#define LEFT_TRIG_BIT   PA0
#define LEFT_ECHO_BIT   PA1
#define FRONT_TRIG_BIT  PA2
#define FRONT_ECHO_BIT  PA3
#define RIGHT_TRIG_BIT  PA4
#define RIGHT_ECHO_BIT  PA5

// PORTC -- Motor direction (L298N IN1-IN4) + I2C
//   PC0 = SCL, PC1 = SDA (hardware TWI, do not reassign)
#define MOTOR_DIR_PORT  PORTC
#define MOTOR_DIR_DDR   DDRC
#define LEFT_IN1_BIT    PC2
#define LEFT_IN2_BIT    PC3
#define RIGHT_IN3_BIT   PC4
#define RIGHT_IN4_BIT   PC5

// PORTD -- USART + PWM
//   PD0 = RXD, PD1 = TXD
//   PD4 = OC1B = LEFT wheel PWM
//   PD5 = OC1A = RIGHT wheel PWM
#define PWM_DDR         DDRD
#define LEFT_PWM_BIT    PD4
#define RIGHT_PWM_BIT   PD5

// ---------------------------------------------------------------------------
//  2. ROBOT PHYSICAL DIMENSIONS          [MEASURE ALL OF THESE]
// ---------------------------------------------------------------------------
#define ROBOT_LENGTH_CM        22   // measured
#define ROBOT_WIDTH_CM         16   // measured (widest point incl. any overhang)

// Distance from the FRONT sonar face back to the wheel axle (the pivot centre).
// This is what makes the robot stop with its axle -- not its nose -- centred
// in an opening. Single most important number in Phase 4.
#define SONAR_TO_AXLE_CM       15   // measured

// Cruise speed in cm/s at DRIVE_BASE_PWM. With no working encoders, every
// distance in this project is (time x speed), so this must be measured:
// drive a timed 100 cm run and divide.
#define TRAVEL_SPEED_CMS       20   // [MEASURE]

// ---------------------------------------------------------------------------
//  3. MAZE GEOMETRY                       [MATCH TO YOUR BUILT MAZE]
// ---------------------------------------------------------------------------
// Pivot radius at ROBOT_LENGTH_CM=22, ROBOT_WIDTH_CM=16 is ~13.6cm (2R~27.2cm).
// At 30cm this leaves ~1.4cm clearance PER SIDE during an in-place pivot --
// the geometric minimum assuming perfect centring, with no margin for turn
// overshoot or approach-timing error. Verify this physically (Mode 2, watch
// the corners during the pivot) before trusting it unsupervised. If it
// clips, the fix is a wider corridor, not a software change.
#define CORRIDOR_WIDTH_CM      30   // wall face to wall face
#define CORRIDOR_HALF_CM       (CORRIDOR_WIDTH_CM / 2)

// ---------------------------------------------------------------------------
//  4. TIMING
// ---------------------------------------------------------------------------
// Master control tick. One sonar ping is issued per tick (round-robin), so
// each individual sensor refreshes every 3 x CONTROL_TICK_MS = 60 ms.
#define CONTROL_TICK_MS        20

// The turn routine samples faster than the main loop for finer angular
// resolution. Because heading integration is in LSB*ms (see heading.h), the
// two rates coexist without needing separate calibration constants.
#define TURN_TICK_MS           5

// ---------------------------------------------------------------------------
//  5. SONAR
// ---------------------------------------------------------------------------
// Cap the echo window to what the maze actually needs. A long timeout would
// block the control tick; 70 cm -> ~4 ms worst case, comfortably inside a
// 20 ms tick.
#define SONAR_MAX_RANGE_CM     70
#define SONAR_TIMEOUT_US       ((uint32_t)SONAR_MAX_RANGE_CM * 58UL)

// HC-SR04 cannot measure closer than ~3 cm; readings below this are garbage.
#define SONAR_MIN_VALID_CM     3

// Returned when no echo comes back. MUST be a large "nothing there" value,
// never 0 -- a 0 would read as "obstacle touching the bumper" and brake the
// robot permanently on the first missed ping.
#define SONAR_NO_ECHO          999

// Distinct from NO_ECHO: a wall is present but closer than the sensor can
// measure. These MUST NOT share a value -- a sub-minimum reading means
// "wall about to be hit", the exact opposite of "nothing there". Aliasing
// them makes the robot report an opening at the instant it is about to
// collide, and (in maze mode) turn straight into the wall.
#define SONAR_TOO_CLOSE        1

// Physical ambiguity: below ~3 cm the echo can return before the sensor
// finishes transmitting, so a very near wall often produces a timeout that
// is indistinguishable, at the ping level, from open space. Disambiguate
// with history instead: if the previous good reading was within this
// distance, a sudden loss of echo means the wall got CLOSER, not that it
// vanished. Walls do not disappear in 60 ms.
#define SONAR_NEAR_LATCH_CM    12

// A reading older than this is stale and must not be trusted.
#define SONAR_STALE_MS         250

// Physical plausibility gate: a wall cannot appear to move faster than the
// robot can travel. Rejects the wild outliers that chassis rocking produces.
#define SONAR_MAX_JUMP_CM      15

// ---------------------------------------------------------------------------
//  6. MOTION-SUSPECT DETECTION  (your point #5)
// ---------------------------------------------------------------------------
// When the chassis pitches or rolls, a perpendicular sonar beam tilts off the
// wall -- it can hit the floor (short reading) or sail over the wall top
// (NO_ECHO). We watch the gyro's X and Y axes.
//
// Units: raw LSB at +/-500 dps (65.5 LSB per deg/sec).
// History: started at 650, which sat BELOW normal gear-motor vibration and
// latched permanently while driving. Raised to 2200, still tripped ~77% of
// ticks -- measured logs show the ROLL axis (gy) peaking near 2900 LSB
// (~44 deg/sec) from ordinary chassis vibration, while pitch (gx) stays
// under ~700. 3000 clears the measured vibration floor.
//
// Note this flag no longer invalidates sonar data (see sonar.c); it only
// reduces the wall-term gain. That makes the exact value far less critical
// than it used to be -- a wrong threshold now degrades tuning, not function.
#define ROCK_RATE_THRESHOLD    3000

// How long after a rocking event to keep distrusting sonar.
#define ROCK_BLANKING_MS       120

// While rocking, the wall term is applied at REDUCED gain rather than being
// frozen out entirely. Freezing it completely means a sustained vibration
// disables centring for the whole run; halving it keeps the robot correcting
// (just less aggressively) on data that is noisy but not worthless.
#define WALL_ROCK_GAIN_NUM     1
#define WALL_ROCK_GAIN_DEN     2

// ---------------------------------------------------------------------------
//  7. MOTORS
// ---------------------------------------------------------------------------
// Your chassis will not start moving below this. Every commanded speed is
// clamped up to it (or to exactly 0 for a full stop).
// Measured on the actual chassis. MOTOR_MIN_PWM is below the stall floor of
// ~60 on purpose: in a differential turn the slow wheel briefly dropping
// under its own stall point makes it act as a pivot, which tightens the
// correction. Both wheels are never commanded this low at once.
#define MOTOR_MIN_PWM          45
#define MOTOR_MAX_PWM          140

// Straight-line cruise speed. Lowered from 75: at the higher speed the robot
// covered too much ground during each correction and swerved close to the
// walls. Slower means the same angular correction translates into less
// lateral overshoot.
#define DRIVE_BASE_PWM         60

// Breakaway kick to overcome static friction on start.
#define KICK_PWM               120  // you lowered this from 160 to protect a
                                    // connector; keep whatever works
#define KICK_MS                40

// ---------------------------------------------------------------------------
//  8. WALL-CENTERING CONTROLLER
// ---------------------------------------------------------------------------
// Correction = (Kp_num * error_cm)/Kp_den + (Kd_num * gyro_rate)/Kd_den
// Integer fractions are used instead of floats (no FPU on ATmega32).
//
// Sign convention: positive correction steers RIGHT.
//   left_pwm  = base + correction
//   right_pwm = base - correction
#define WALL_KP_NUM            3
#define WALL_KP_DEN            2    // 1.5 PWM counts per cm of centring error

// Gyro damping. Positive gyro Z = turning LEFT (matches your original
// straight-line code, where a positive accumulator slowed the right wheel).
//
// Raised from /120 to /65. At /120 the damping was far too weak to oppose
// the proportional term: measured logs show wt=+19 against gt=-10 at
// -17.8 deg/s, so the controller kept commanding MORE right-turn while
// already turning right hard. The result was a runaway correction turn that
// swung the front sonar off an obstacle. At /65 the two terms balance near
// 18 deg/s, so the correction self-limits instead of winding up.
#define WALL_KD_NUM            1
#define WALL_KD_DEN            65

// The correction is a DIFFERENTIAL about DRIVE_BASE_PWM, so its steering
// authority is the ratio corr/base, not its absolute value. Lowering the
// base speed without lowering this makes steering MORE violent, not less.
// At base 60 a +-35 differential is a 58% split; 37% keeps it proportionate
// to what +-35 gave at the original base of 75.
#define WALL_MAX_CORRECTION_RATIO_PCT  37

// Never let the differential exceed this, or a bad reading can spin the robot.
// Derived from DRIVE_BASE_PWM so lowering the base speed automatically
// softens the steering instead of sharpening it.
#define WALL_MAX_CORRECTION    ((DRIVE_BASE_PWM * WALL_MAX_CORRECTION_RATIO_PCT) / 100)

// Emergency avoidance: inside this distance the proportional correction is
// too slow. Severity now RAMPS with proximity rather than slamming to full
// differential at the threshold -- measured logs show a hard +-35 step at
// exactly 8cm produced 71 deg/s of yaw and bounced the robot from one wall
// straight into the other.
#define WALL_EMERGENCY_CM      8

// GYRO HEADING HOLD for timed straight legs (Modes 5 and 7).
// This is the straight-line autocorrect from the original single-file
// firmware brought forward: P on ACCUMULATED heading error plus D on rate.
// The wall-centring controller cannot do this job -- with no valid sonar it
// falls into CENTER_GYRO_ONLY, where error_cm is 0, so only the rate term
// survives. That damps rotation but never returns to the original heading:
// drift 10 degrees, stop rotating, and the correction goes to zero with the
// robot still 10 degrees off.
//
// Kp is in PWM counts per TENTH of a degree. 1/8 = 1.25 counts per degree,
// which reproduces the original algorithm's authority: it used
// heading_accum/2600 on raw LSB at a 10ms loop, and one degree is 65500
// LSB*ms, so 65500/26000 = 2.5 counts per degree of DIFFERENTIAL. This
// controller is symmetric (base +/- corr), so the differential is 2*corr and
// 1.25 per degree matches.
//
// D gets its OWN constant. Reusing WALL_KD_NUM/WALL_KD_DEN (=1/65) here was
// wrong: that was tuned against a P term measured in CENTIMETRES of wall
// offset, where P contributes 7..22 counts on a normal 5-15cm error. This P
// term is in degrees and contributes only 1.25 counts per degree, so the same
// D is proportionally about 3x too strong -- and a measured run proved it. At
// /65, one deg/sec of rotation cancels 0.8 degrees of heading error, so the
// controller behaved as a rate damper, not a position controller:
//
//   L,1281,62,-503,0,60,60   <- 6.2 deg off heading, correction ZERO
//                               P = 62/8 = +7, D = -503/65 = -7, sum 0
//
// The original algorithm's ratio was 2.52 counts/degree against rate/100 of
// DIFFERENTIAL, i.e. one deg/sec cancelled only 0.26 degrees of error. This
// controller is symmetric (base +/- corr) so the differential is 2*corr, and
// /200 reproduces that ratio exactly.
//
// Deliberately NOT adding an I term yet. The measured bias is a start-up
// transient, not a constant offset, and an integrator on this loop is how
// bug #1 (windup to saturation, first crash) happened. Fix the ratio first.
#define HOLD_TICK_MS           10   // matches the original autocorrect loop
#define HOLD_KP_NUM            1
#define HOLD_KP_DEN            8
#define HOLD_KD_NUM            1
#define HOLD_KD_DEN            200

// Per-sample trace of the heading-hold controller, so a leg that does not
// track straight can be diagnosed from the log instead of guessed at:
//   L,<ms into leg>,<err10>,<rate>,<corr>,<pwmL>,<pwmR>
// err10 is heading error in tenths of a degree (+ = left of target), rate is
// raw LSB, corr is the differential the controller asked for, and pwmL/pwmR
// are what actually reached the motors AFTER clamping -- when those two stop
// differing by 2*corr, the correction is being eaten by MOTOR_MIN_PWM or
// MOTOR_MAX_PWM and the controller has no authority left.
//
// A line is ~34 bytes. At HOLD_TICK_MS=10 every sample would be ~3400 byte/s
// against a 3840 byte/s budget at 38400 baud -- no headroom, so bytes drop.
// Decimating by 2 puts it near 44%. Raise this if `drop` climbs in the
// per-leg summary.
#define HOLD_TRACE             1
#define HOLD_SAMPLE_EVERY      2

// WALL-STUCK RECOVERY.
// The ramped emergency steer above still drives BOTH wheels forward -- it
// only varies the split. If a chassis corner physically catches the wall
// (friction pins it), that forward-biased differential cannot rotate the
// robot away: it just grinds along the wall at an angle instead of turning
// off it. Observed on hardware. If a one-sided emergency stays active this
// long without clearing, stop assuming steering alone will break contact:
// stop pushing forward, back straight off the wall, then pivot away from it
// in place before letting normal centring resume.
#define WALL_STUCK_MS           400
#define WALL_RECOVERY_REV_PWM   100  // straight reverse, both wheels
#define WALL_RECOVERY_REV_MS    250
#define WALL_RECOVERY_PIVOT_PWM 90   // in-place pivot, away from the wall
#define WALL_RECOVERY_PIVOT_MS  300

// YAW GOVERNOR.
// Hard ceiling on how fast the chassis may rotate while centring. Beyond
// this the sonars are pointing far enough off-axis that their readings stop
// describing the corridor -- the front beam in particular walks off whatever
// is ahead. Once exceeded, the controller stops ADDING yaw in that direction
// (it does not reverse; it just stops winding up) and lets the rotation
// decay. Units: raw LSB; 65.5 LSB per deg/sec. 1300 ~= 20 deg/sec.
#define YAW_GOVERNOR_LSB       1300

// ---------------------------------------------------------------------------
//  9. JUNCTION / OPENING DETECTION
// ---------------------------------------------------------------------------
// A side reads "open" beyond this. Corridor half-width plus margin.
#define OPENING_THRESHOLD_CM   (CORRIDOR_HALF_CM + 10)   // 25 cm

// Front is considered blocked closer than this. Must be generous enough that
// at a T-junction the front wall registers as blocked BEFORE the two side
// openings appear -- otherwise a T momentarily looks like the maze exit.
// (Currently equals OPENING_THRESHOLD_CM by coincidence, not by design --
// they are independent constants for different sensors and may diverge if
// either is retuned.)
#define FRONT_BLOCKED_CM       25

// Consecutive confirmations before believing a side opening.
#define OPENING_CONFIRM        2

// Consecutive confirmations before believing a dead end (all three walled).
// Higher than OPENING_CONFIRM: a spurious U-turn is expensive.
#define DEADEND_CONFIRM        3

// Front-obstacle detection uses VOTING over a window, not consecutive hits.
//
// Why: the front sonar is rigidly mounted, so whenever the chassis yaws to
// correct its position the beam swings off-axis. A real obstacle straight
// ahead can then be missed for several consecutive pings -- measured logs
// show F reading 51,60,57,65,65 (INCREASING) while driving into an obstacle
// at ~50cm, because a sustained ~18 deg/s correction turn walked the beam
// off the target. A "2 consecutive" rule can never fire in that situation.
//
// Voting fixes it: an obstacle that shows up in ANY 2 of the last 5 pings is
// treated as real. Obstacles do not vanish; intermittent detection during a
// yaw is expected, so intermittent evidence must be enough.
#define FRONT_VOTE_WINDOW      5
#define FRONT_VOTE_THRESHOLD   2

// Kept for the Mode 1 harness; the voting above is what actually decides.
#define FRONT_STOP_CONFIRM     1

// ---------------------------------------------------------------------------
//  10. APPROACH OFFSET  (front-mounted sonar compensation)
// ---------------------------------------------------------------------------
// The sonar sees an opening while the sensor is level with it, but the robot
// pivots about the AXLE, further back. Drive on by this much so the pivot
// centre ends up in the middle of the opening.
#define APPROACH_DISTANCE_CM   (SONAR_TO_AXLE_CM + CORRIDOR_HALF_CM)  // 30 cm
#define APPROACH_TIME_MS       (((uint32_t)APPROACH_DISTANCE_CM * 1000UL) / TRAVEL_SPEED_CMS)

// When the front is blocked we cannot drive the full approach distance --
// stop this far from the wall instead and pivot there.
#define FRONT_STOP_CM          12

// ---------------------------------------------------------------------------
//  11. END-OF-MAZE DETECTION  (your point #3)
// ---------------------------------------------------------------------------
// All three sensors open => exit. But at a left-or-right T-junction both side
// openings can appear a moment before the front wall closes in, which looks
// identical for a short window. So: on first detection, drive on this far and
// re-check. A real exit stays open; a T-junction's front wall closes in.
#define EXIT_CONFIRM_CM        25
#define EXIT_CONFIRM_MS        (((uint32_t)EXIT_CONFIRM_CM * 1000UL) / TRAVEL_SPEED_CMS)

// ---------------------------------------------------------------------------
//  12. TURNS
// ---------------------------------------------------------------------------
#define TURN_PWM               85   // slow = accurate
#define TURN_KICK_PWM          KICK_PWM
#define TURN_KICK_MS           40
#define TURN_BRAKE_PWM         100
#define TURN_BRAKE_MS          20
// Measured (6-turn Mode 6 run, both directions): after the motors cut, the
// chassis was still rotating above ~3 deg/sec for the WHOLE 300ms window --
// coastms came back 283..305 against a 300 window, and one turn was still at
// 23 deg/sec when it closed. The closed-loop correction below was therefore
// measuring a heading that had not stopped changing. 500 gives real margin.
#define TURN_SETTLE_MS         500  // motors off, still integrating coast

// Cut the main sweep this early. Was 4, which was not a coast estimate at all
// -- measured coast from sweep exit to rest is 36.7/40.2/40.9/41.0/46.4/42.3
// degrees (mean 41.3) because the sweep is still ACCELERATING when it exits
// (rate climbed monotonically to ~24000 LSB, ~370 deg/sec, never reaching
// terminal velocity in 86 degrees). The result was a physical swing to ~128
// degrees followed by 4-5 reverse nudges back to 90: correct final angle,
// wrong mechanism, and a 38 degree excursion the corridor has to absorb.
//
// 35 is derived, not guessed: cutting at 55 degrees leaves the chassis at
// ~20800 LSB instead of ~22700, and coast scales somewhere between linearly
// and quadratically with cut-off rate, which puts the landing at 89.6..92.8
// degrees. The nudge loop trims either end of that easily -- and it corrects
// in BOTH directions, so an over- or under-estimate here is self-healing.
// NEEDS ONE MODE 6 RUN TO CONFIRM: expect err10 near zero and nudges 0-1.
//
// This value is calibrated for 90 degree turns. A single 180 degree sweep
// would exit far faster and coast much further, which is the real reason
// TURN_180_AS_TWO_90S must stay 1.
#define TURN_STOP_MARGIN_DEG   35

// 2 degrees is about the floor worth chasing: the stream shows ~1 degree of
// mechanical settling jitter (tyres unwinding) after the rotation stops.
#define TURN_DEADBAND_DEG      2    // "close enough"
#define TURN_NUDGE_PWM         140

// Nudge duration SCALES with the remaining error instead of firing the same
// fixed-length pulse regardless of how far off the turn is. A fixed pulse
// either wastes correction attempts creeping toward a large residual error,
// or overcorrects a 1 degree residual by the same amount used for a 6 degree
// one -- which is how a "converging" turn ends up oscillating around the
// target instead of settling into TURN_DEADBAND_DEG.
// ms = clamp(error_deg * TURN_NUDGE_MS_PER_DEG, MIN, MAX). Tune
// TURN_NUDGE_MS_PER_DEG by watching `err10`/nudge count in Mode 2 telemetry:
// still 2+ nudges of the same sign in a row -> raise it; nudges routinely
// overshoot the deadband the other way -> lower it.
#define TURN_NUDGE_MS_MIN      8
#define TURN_NUDGE_MS_MAX      40
#define TURN_NUDGE_MS_PER_DEG  6    // ms per whole degree of residual error

#define TURN_MAX_NUDGES        5
#define TURN_TIMEOUT_MS        7000 // safety: abort a turn that never finishes

// Do a 180 as two 90s with a settle between. Usually more accurate than one
// long sweep because momentum has less time to build. Set 0 for a single 180.
#define TURN_180_AS_TWO_90S    1

// ---------------------------------------------------------------------------
//  13. GYRO CALIBRATION      (your point #4)
// ---------------------------------------------------------------------------
// Full calibration at power-up.
#define GYRO_CAL_SAMPLES_INIT  500
// Shorter re-calibration performed after every stop and every pivot, to catch
// thermal bias drift. Kept brief so it does not dominate the run time.
#define GYRO_CAL_SAMPLES_QUICK 150
#define GYRO_CAL_INTERVAL_MS   2

// Calibration is only valid if the chassis is genuinely still. If the spread
// between the highest and lowest sample exceeds this, the robot was moving or
// vibrating: the result is discarded and the previous offset kept.
#define GYRO_CAL_MAX_SPREAD    250
#define GYRO_CAL_RETRIES       3
#define GYRO_SETTLE_MS         250  // wait for the chassis to stop rocking first

// Empirical trim carried over from your straight-line tuning, re-applied after
// every calibration.
#define GYRO_OFFSET_TRIM       3

// Datasheet sensitivity at +/-500 dps is 65.5 LSB per deg/sec. Heading is
// accumulated in LSB*milliseconds, so:
//     degrees = accum / (65.5 * 1000) = accum / 65500
// This is sample-rate independent, which is why the turn loop and the drive
// loop can run at different tick rates without separate constants.
// Tune by protractor: new = old * (commanded_angle / measured_angle).
#define GYRO_LSB_MS_PER_DEGREE 65500L

// ---------------------------------------------------------------------------
//  14. RECOVERY & SAFETY
// ---------------------------------------------------------------------------
// After a turn the sonar history is meaningless. Drive gyro-only for this
// long while the filters refill.
#define RECOVER_MS             400

// Longest a single straight leg may run before forcing a junction decision.
// Safety net for walls the sonar misses entirely (angled/soft surfaces).
#define MAX_LEG_MS             10000UL

// Stall detection: motors commanded but the gyro sees no rotation AND the
// sonar readings are not changing. Usually means a power/connector fault.
#define STALL_CHECK_MS         1500

// Global run limit.
#define MAX_RUN_MS             300000UL   // 5 minutes

// Delay after power-up before moving, so you can put the robot down.
#define STARTUP_DELAY_MS       3000

// ---------------------------------------------------------------------------
//  15. DIAGNOSTIC BUILD MODES
// ---------------------------------------------------------------------------
// Mode 4: sonar cone characterization. Motors stay off; place the robot mid-
// corridor and rotate it BY HAND while watching heading vs L/F/R to see
// exactly what angle makes a beam pick up the wrong wall. Faster than
// TELEMETRY_INTERVAL_MS -- this is a live, watch-it-happen test, not a logged
// run, so finer time resolution on a slow hand rotation is worth more than
// keeping the byte budget small.
#define SONAR_TEST_INTERVAL_MS   50

// Mode 5: open-loop square. No sonar, no wall centring, no corridor -- pure
// forward-drive and turn-accuracy test. Four legs of SQUARE_LEG_MS forward at
// SQUARE_TEST_PWM, each followed by a 90 degree turn, should return the robot
// to its start point facing its start heading.
#define SQUARE_TEST_PWM          60   // requested: keep base speed at 60
#define SQUARE_LEG_MS            2000 // forward time per side, kick included
#define SQUARE_TURN_SETTLE_MS    200  // let the chassis stop coasting before
                                       // the turn's own kick fires -- same gap
                                       // TURN_180_AS_TWO_90S uses between its
                                       // two 90s
#define SQUARE_SIDES             4

// Also stream the per-sample yaw-rate profile through each of the square's
// turns (the same stream Mode 6 uses). Legs and turns never overlap, so this
// costs no extra bandwidth during a leg. Set 0 for a quieter log once the
// turns are trusted and only the legs are in question.
#define SQUARE_TRACE_TURNS       1

// Per-phase turn tracing. A turn is blocking and prints nothing per sample
// today, so a Mode 2 run yields ONE summary line -- not enough to tell a
// too-short settle from a too-long coast from a clipping gyro. With this on,
// each phase boundary prints the heading it ended at (about 10 short lines
// per turn, ~250 bytes over ~1.5 s: no risk to the byte budget). Turn it off
// for Mode 3 runs, where it would interleave with corridor telemetry.
#define TURN_TRACE               1

// Mode 6: dedicated turn debugging. Repeats a pivot with a pause after each
// one so the physical angle can be measured and written down, and streams the
// raw yaw rate through the whole turn so the angular-velocity profile can be
// reconstructed offline -- that profile is what separates "the coast is longer
// than TURN_SETTLE_MS" from "the gyro is clipping" from "the brake pulse does
// nothing", which a single end-of-turn angle cannot.
#define TURNDBG_ANGLE            90
#define TURNDBG_REPEATS          6
// Alternate R,L,R,L... Turn error that differs by direction means a motor or
// tyre asymmetry, not a calibration error -- and it exercises the direction
// sign check (turn_result_t.wrong_way) both ways.
#define TURNDBG_ALTERNATE        1
#define TURNDBG_PAUSE_MS         6000 // protractor the angle, write it down

// Per-sample stream decimation. 1 = every TURN_TICK_MS (5ms) sample, which is
// ~22 bytes per 5ms against a 3840 byte/s budget at 38400 baud -- over 100%,
// so bytes WILL drop. 2 = every 10ms (~57%), which fits with headroom. Watch
// the `drop` field in the per-turn summary: if it climbs, raise this.
#define TURNDBG_SAMPLE_EVERY     2

// |yaw rate| below this counts as "stopped" when measuring how long the
// chassis actually coasts after the motors cut. Raw LSB; 65.5 LSB per deg/sec,
// so 200 ~= 3 deg/sec.
#define TURNDBG_STILL_LSB        200

// ---------------------------------------------------------------------------
//  16. DEBUG
// ---------------------------------------------------------------------------
#define DEBUG_ENABLED          1

// Raised from 9600. At 9600 the old telemetry line already consumed ~54% of
// the byte budget; the detailed CSV format would exceed 100% and the blocking
// writer would then stall the control loop. 38400 leaves ample headroom.
// SET YOUR SERIAL TERMINAL TO MATCH -- 38400 8N1.
#define USART_BAUDRATE         38400

// Ring buffer for non-blocking transmit. One telemetry line must fit
// comfortably, or bytes get dropped every tick.
#define DEBUG_TX_BUF           192

#define TELEMETRY_INTERVAL_MS  100

// Telemetry verbosity:
//   0 = events only (turns, junctions, faults)
//   1 = compact CSV control-loop trace  <-- use this for tuning
//   2 = CSV plus per-tick timing/health counters
#define DEBUG_LEVEL            1

// Warn when a control tick overruns its deadline. A sonar timeout plus a
// blocking print used to be enough to do this; it must stay at zero.
#define TICK_OVERRUN_WARN_MS   (CONTROL_TICK_MS + 5)

#endif // CONFIG_H
