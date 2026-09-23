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

// PORTB -- operator panel (LED + start button)
//
// PORTB was the only port with nothing on it at all. PB0 and PB1 are adjacent,
// so the panel is one 3-pin header (LED, BUTTON, GND), and both are clear of
// PB5/PB6/PB7 = MOSI/MISO/SCK, so the ISP programmer can stay connected while
// the panel is wired -- which it cannot if the button sits on an ISP pin.
//
// Both alternate functions on these pins are inactive in this build: PB0 is
// T0/XCK (Timer0 runs off the internal clock, the USART is asynchronous) and
// PB1 is T1 (Timer1 runs off the internal clock). Nothing to reassign.
//
//   PB0 --[330R]--|>|-- GND     LED, active high
//   PB1 -----------o o-- GND    button to ground, internal pull-up, press = LOW
#define PANEL_PORT      PORTB
#define PANEL_DDR       DDRB
#define PANEL_PIN       PINB
#define LED_BIT         PB0
#define BUTTON_BIT      PB1

// Consecutive agreeing samples before the debounced button level moves. At
// CONTROL_TICK_MS = 20 this is 60 ms, comfortably past the few ms a panel
// button bounces for, and far too short to feel laggy.
#define BUTTON_DEBOUNCE_TICKS  3

// Hold the button this long to throw the saved route away and explore again.
// Without it, re-exploring means editing WALLMEM_FORCE_EXPLORE and reflashing,
// which during a lab session is exactly when you least want to.
#define BUTTON_LONG_PRESS_MS   2000UL

#define LED_BLINK_SLOW_MS      500   // ~1 Hz -- finished
#define LED_BLINK_FAST_MS      120   // ~4 Hz -- fault

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
// distance in this project is (time x speed), so this must be measured.
//
// NOW MEASURED, from the Mode 10 front-range traces rather than a stopwatch.
// Over the steady part of the cruise phase the front range closed at 28 cm/s
// on two runs (log3: 51->33 cm in 0.65 s; log4: 56->32 cm in 0.85 s). The old
// placeholder of 20 was low by 50%, which made every time-based distance in
// Mode 3 -- APPROACH_TIME_MS above all -- overshoot by half as much again.
//
// Caveat worth keeping in mind: late in a run the same measurement gives
// 41-46 cm/s. That is the chassis still accelerating, not a second speed, so
// 30 is the conservative figure for a leg that starts from rest. Re-measure
// over a long straight leg if Mode 3 distances still run long.
#define TRAVEL_SPEED_CMS       30   // measured from Mode 10 traces

// ---------------------------------------------------------------------------
//  3. MAZE GEOMETRY                       [MATCH TO YOUR BUILT MAZE]
// ---------------------------------------------------------------------------
// Pivot radius at ROBOT_LENGTH_CM=22, ROBOT_WIDTH_CM=16 is ~13.6cm (2R~27.2cm).
// At 30cm this leaves ~1.4cm clearance PER SIDE during an in-place pivot --
// the geometric minimum assuming perfect centring, with no margin for turn
// overshoot or approach-timing error. Verify this physically (Mode 2, watch
// the corners during the pivot) before trusting it unsupervised. If it
// clips, the fix is a wider corridor, not a software change.
// *** SET THIS TO YOUR ACTUAL BUILT CORRIDOR WIDTH. *** It is load-bearing in
// more places than it looks: CORRIDOR_HALF_CM is the target distance for
// single-wall centring (get it wrong and the robot deliberately drives
// off-centre by the error), OPENING_THRESHOLD_CM and APPROACH_DISTANCE_CM are
// both derived from it, and so is the emergency band below.
// Raised 30 -> 40 to match the test maze that was actually designed and built.
#define CORRIDOR_WIDTH_CM      40   // wall face to wall face
#define CORRIDOR_HALF_CM       (CORRIDOR_WIDTH_CM / 2)

// Free space each side of a PERFECTLY CENTRED robot. Everything that talks
// about "how near is too near" has to be smaller than this, or the robot is
// in a fault state while doing nothing wrong. See WALL_EMERGENCY_CM.
#define CORRIDOR_SIDE_GAP_CM   ((CORRIDOR_WIDTH_CM - ROBOT_WIDTH_CM) / 2)

// What a side sonar ACTUALLY READS when the robot is centred.
//
// This is deliberately NOT CORRIDOR_HALF_CM, and the logs are why. Across all
// four Mode 10 runs the two side readings summed to 28-31 cm, while the
// geometry above predicts CORRIDOR_WIDTH_CM - ROBOT_WIDTH_CM = 24. The extra
// ~5 cm is real and repeatable: the sensor faces sit inboard of the widest
// part of the chassis, so each one reads a few cm more than the true gap.
//
// It does not matter while both walls are visible -- that mode steers on the
// DIFFERENCE, which cancels the offset. It matters a great deal the moment one
// wall disappears, because CENTER_LEFT_ONLY / CENTER_RIGHT_ONLY steer toward
// an absolute target: aiming at CORRIDOR_HALF_CM (20) when centred actually
// reads ~14.5 would drive the robot 5 cm off-centre on purpose, every time it
// passed an opening.
//
// Re-measure by parking the robot centred in the corridor and halving the
// logged L+R.
#define SIDE_CENTRED_CM        15

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
// too slow. Severity RAMPS with proximity rather than slamming to full
// differential at the threshold -- measured logs show a hard +-35 step at
// exactly 8cm produced 71 deg/s of yaw and bounced the robot from one wall
// straight into the other.
//
// THIS WAS A FIXED 8 AND THAT WAS A BUG. At CORRIDOR_WIDTH_CM=30 the centred
// side gap is (30-16)/2 = 7 cm, so a PERFECTLY CENTRED robot sat inside the
// emergency band on BOTH sides. Being slightly off-centre then put one side
// under 8 while the other was over it, which is exactly the one-sided
// emergency condition: the robot hard-steered away from a wall it was not
// close to, and after WALL_STUCK_MS escalated to a reverse-and-pivot
// recovery. That is the reported "behaves really badly when it isn't placed
// exactly in the middle" -- it was a geometry contradiction, not tuning.
//
// Derived from the actual gap now, so it cannot contradict the corridor:
// 45% of the centred gap, never below 4 cm (under that the sonar's own
// SONAR_MIN_VALID_CM / too-close latch is the operative signal anyway).
#define WALL_EMERGENCY_RAW     ((CORRIDOR_SIDE_GAP_CM * 45) / 100)
#define WALL_EMERGENCY_CM      (WALL_EMERGENCY_RAW >= 4 ? WALL_EMERGENCY_RAW : 4)

// Fail the BUILD rather than the run if the band ever swallows the centred
// position again. A robot that is in a fault state while perfectly centred
// cannot be tuned out of it.
#if (WALL_EMERGENCY_CM) >= (CORRIDOR_SIDE_GAP_CM)
#  error "WALL_EMERGENCY_CM >= CORRIDOR_SIDE_GAP_CM: a centred robot would be in permanent emergency. Widen CORRIDOR_WIDTH_CM or lower the emergency band."
#endif

// A one-sided emergency means "this wall is near AND the other side is where
// the room is". In a corridor barely wider than the robot both sides can be
// near at once; steering hard away from one then just drives into the other.
// Require the far side to be at least this much clearer before treating the
// situation as one-sided -- otherwise fall through to normal centring, which
// splits the difference instead of picking a side.
#define WALL_EMERG_ASYMMETRY_CM 3

// Small differential errors are not worth steering for. error_cm is a
// DIFFERENCE of the two side readings, so it is twice the actual off-centre
// offset: 2 here means "ignore under 1 cm off-centre". Without this the
// controller micro-steers continuously on sonar quantisation noise, and every
// one of those little yaws walks the front beam off whatever is ahead.
#define WALL_DEADBAND_CM        2

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

// The timer alone was too blunt a trigger: it fired whenever the robot was
// still inside the emergency band after WALL_STUCK_MS, even when the steering
// was working and the wall was steadily receding. Recovery is a violent,
// position-destroying maneuver and must only run when steering has genuinely
// FAILED. So the deadline now resets whenever the near wall gets this much
// further away than the closest it had been -- progress restarts the clock,
// and only a distance that refuses to improve escalates.
#define WALL_STUCK_IMPROVE_CM   2

// ACTIVE BRAKE for Drive_Stop().
// Drive_Stop() used to be a bare Motors_Stop(), so the chassis coasted
// unbraked -- around 17 cm was observed, which is most of a corridor width and
// enough to put the nose into a wall the controller had correctly decided to
// stop short of. Turns already brake with a reverse pulse (TURN_BRAKE_*); this
// is the same idea for forward motion. Set DRIVE_BRAKE_MS to 0 to disable.
//
// Too long and the pulse pushes the robot backwards instead of stopping it.
// The Mode 10 log prints the front distance at the stop decision and again
// after the brake, so the residual coast is directly measurable -- tune
// against that, not by eye.
#define DRIVE_BRAKE_PWM         100
#define DRIVE_BRAKE_MS          80

// Below this base speed, do NOT lift both wheels to keep a wheel off the stall
// floor -- clamp the correction instead, so the MEAN speed stays what was
// asked for.
//
// The lift exists to preserve the steering differential, and at cruise that is
// the right trade. At creep it inverts the whole point of creeping: with
// MOTOR_MIN_PWM at 45, a base of 48 left 3 counts of headroom, so nearly every
// correction triggered a lift and the logged mean came back to 50-62 -- cruise
// speed under a "creep" label. Measured means were 50.6, 52.8 and 56.2 against
// a nominal 48. Below this threshold, steering authority yields to speed
// control; the robot is close to the obstacle and moving slowly, so arriving
// at the right SPEED matters more than shaving the last degree off centring.
#define DRIVE_MEAN_PRESERVE_BELOW  58

// KICK RAMP -- brownout mitigation.
//
// Every failed run in the four Mode 10 logs died at a PWM-120 kick, and none
// died anywhere else:
//   log2  t=103 ms, just after Drive_Begin()'s KICK_PWM kick
//   log3  immediately after the first pivot's TURN_KICK_PWM kick
//   log4  immediately after the SECOND pivot's kick
//   log1  boot #1, freshest battery, survived the whole run
// Every one reported BORF with SRAM intact: the rail dipped below the
// brown-out threshold and recovered, rather than a broken connection.
//
// Stepping 0 -> 120 in one PWM period is the largest current transient the
// firmware ever asks for: the motor is stalled, so it draws locked-rotor
// current with no back-EMF to oppose it. Ramping over the kick spreads that
// same impulse over KICK_MS and roughly halves the peak.
//
// THIS IS MITIGATION, NOT A CURE. The root cause is supply, not firmware --
// see the analysis notes. Set to 0 to restore the old instant step.
#define KICK_RAMP              1

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
// All three sensors open => exit. But a junction can look exactly the same for
// a short window, so on first detection the robot drives on and re-checks: a
// real exit stays open, a junction's front wall closes in.
//
// HOW FAR IS "ON"? This used to be a flat 25, which happened to be right. It is
// now derived, because the number is not free -- it is pinned by geometry at
// both ends, and a corridor width change would silently break it.
//
// The moment the front sonar crosses into a junction cell, that cell's far wall
// is CORRIDOR_WIDTH_CM ahead and therefore reads OPEN; both sides have just
// opened too. It keeps reading open until the wall is within FRONT_BLOCKED_CM.
// So the longest an in-maze junction can impersonate the exit is exactly:
#define EXIT_FALSE_WINDOW_CM   (CORRIDOR_WIDTH_CM - FRONT_BLOCKED_CM)   // 15 cm

// Drive clear of that window before believing it. The margin covers the sonar
// refresh (3 x CONTROL_TICK_MS per sensor) and speed error.
#define EXIT_CONFIRM_MARGIN_CM 10
#define EXIT_CONFIRM_CM        (EXIT_FALSE_WINDOW_CM + EXIT_CONFIRM_MARGIN_CM)
#define EXIT_CONFIRM_MS        (((uint32_t)EXIT_CONFIRM_CM * 1000UL) / TRAVEL_SPEED_CMS)

// The margin has to outlast the EVIDENCE, not just the geometry. One sonar
// refresh is CONTROL_TICK_MS x 3 sensors (they are pinged round-robin), and a
// side opening needs OPENING_CONFIRM of them before it counts -- so this much
// travel passes before the classifier can even change its mind:
#define EXIT_CONFIRM_SETTLE_CM \
    (((CONTROL_TICK_MS) * 3 * (OPENING_CONFIRM) * (TRAVEL_SPEED_CMS)) / 1000)

// Below twice that, the window is shorter than the evidence it is waiting for
// and a T-junction can read as the maze exit -- the run then ends in the middle
// of the maze with a route that goes nowhere. Note this checks the MARGIN, not
// EXIT_CONFIRM_CM: that is defined as window + margin, so comparing it against
// the window can never fail and would be a guard that only looks like one.
#if (EXIT_CONFIRM_MARGIN_CM) < (2 * (EXIT_CONFIRM_SETTLE_CM))
#  error "EXIT_CONFIRM_MARGIN_CM is below two sonar-confirm periods of travel: a T-junction could read as the maze exit. Raise it, slow TRAVEL_SPEED_CMS, or lower OPENING_CONFIRM."
#endif

// ASSUMPTION, not checkable at compile time: the maze has no 4-way crossroads.
// At a true crossroads all three sensors stay open for a whole cell, which is
// longer than this window, and the robot would call it the exit. The demo maze
// in WALLMEM.md has none -- deliberately, since a 4-way also leaves nothing for
// the wall-centring to hold on to while crossing it.

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

// ---------------------------------------------------------------------------
//  Mode 9: open-space obstacle avoidance
// ---------------------------------------------------------------------------
// Forward under gyro heading hold (no corridor, no wall centring) until the
// front sonar sees something, then stop, pivot right 90, and drive one more
// leg. Safety bound on the approach so an empty room does not mean driving
// until the battery dies.
#define AVOID_APPROACH_MAX_MS    15000UL

// How far the second leg actually travels, and therefore how much room must
// be clear in that direction BEFORE committing to the pivot. Derived rather
// than guessed so it tracks the leg settings.
//
// CAVEAT: TRAVEL_SPEED_CMS is still the unmeasured placeholder (20), so this
// figure is only as good as that constant. Measure it and this tightens up.
#define AVOID_LEG_CM             (((uint32_t)SQUARE_LEG_MS * TRAVEL_SPEED_CMS) / 1000UL)
#define AVOID_TURN_CLEARANCE_CM  (AVOID_LEG_CM + FRONT_BLOCKED_CM)

// Clearance the PIVOT itself needs ahead of the front sonar. The axle sits
// SONAR_TO_AXLE_CM behind the sonar face, and the front corners swing on a
// radius of sqrt(SONAR_TO_AXLE^2 + (WIDTH/2)^2) = sqrt(15^2 + 8^2) = 17cm
// about that axle -- note that is NOT the 13.6cm half-diagonal, which is the
// radius about the geometric centre, and the axle is 4cm behind it. So the
// corner clears the wall by (front_reading + SONAR_TO_AXLE_CM - 17). Stopping
// with this much showing on the front sensor keeps that positive with margin.
#define AVOID_PIVOT_CLEARANCE_CM 10

// Per-phase turn tracing. A turn is blocking and prints nothing per sample
// today, so a Mode 2 run yields ONE summary line -- not enough to tell a
// too-short settle from a too-long coast from a clipping gyro. With this on,
// each phase boundary prints the heading it ended at (about 10 short lines
// per turn, ~250 bytes over ~1.5 s: no risk to the byte budget). Turn it off
// for Mode 3 runs, where it would interleave with corridor telemetry.
#define TURN_TRACE               1

// Mode 8: gyro / I2C connection diagnostic.
// Bounded TWI wait, used ONLY by I2C_ReadRegs() on the diagnostic path. One
// byte at 100 kHz is ~90us, so 5ms is enormously generous -- if it expires the
// bus really has stopped answering.
#define I2C_TIMEOUT_US           5000UL
#define GYRODIAG_READS           200    // reliability + noise-floor samples
#define GYRODIAG_MONITOR_MS      60000UL// live monitor window
#define GYRODIAG_CHECK_MS        250    // config re-verify interval during it
// A healthy MPU6050 lying still still jitters by this much on one axis. ZERO
// spread means the value is frozen -- stale data rather than a live read, the
// signature of a half-dead bus. A spread far above the ceiling means
// electrical noise on the supply or the signal lines.
#define GYRODIAG_NOISE_MIN       5
#define GYRODIAG_NOISE_MAX       400

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
//  15b. MODE 10: DEAD-END 180
// ---------------------------------------------------------------------------
// Drive the corridor centred, treat a front obstacle as a dead end, and turn
// around -- picking the rotation direction from where the chassis actually
// sits between the walls.
//
// WHY DIRECTION MATTERS. An in-place pivot is NOT symmetric. The chassis
// rotates about the AXLE, which sits SONAR_TO_AXLE_CM behind the nose, so:
//   front corners swing  sqrt(15^2 + 8^2) = 17.0 cm from the axle
//   rear corners swing   sqrt( 7^2 + 8^2) = 10.6 cm from the axle
// The front corners sweep into the side being turned TOWARDS; the rear corners
// sweep out the opposite side. The turning side therefore needs ~6.4 cm more
// free space than the other. If the chassis is hugging the left wall, rotating
// LEFT drags the wide front corner straight into it, while rotating RIGHT
// only puts the narrow rear corner there. Hence: turn AWAY from the near wall.
#define PIVOT_FRONT_RADIUS_CM  17   // sqrt(SONAR_TO_AXLE^2 + (WIDTH/2)^2)

// How close the FRONT sonar may read before a pivot grazes the wall. The wall
// sits (reading + SONAR_TO_AXLE_CM) from the axle and the corner swings
// PIVOT_FRONT_RADIUS_CM, so the corner clears by (reading - 2). At a reading
// of 2 cm it exactly touches.
#define PIVOT_FRONT_NEED_CM    (PIVOT_FRONT_RADIUS_CM - SONAR_TO_AXLE_CM)  // 2 cm

// Distance at which Mode 10 calls the obstacle a dead end and stops.
//
// DELIBERATELY NOT FRONT_BLOCKED_CM (25). That constant is shared with the
// junction classifier, where it has to be generous enough that a T-junction's
// front wall registers as blocked BEFORE the two side openings appear -- see
// section 9. Stopping a dead-end run 25 cm out is far earlier than the pivot
// needs, but lowering the shared constant to fix that would break junction
// detection everywhere. Hence a separate knob, read through
// Sonar_FrontCloserThan() so it keeps the same vote-window robustness.
//
// TUNING: this is the distance the stop is TRIGGERED at, not where the
// chassis ends up.
//
// MEASURED at 12: the chassis came to rest 3-4 cm from the wall on three runs.
// That 8-9 cm total splits into two very different halves, and the log line
// used to blame the wrong one:
//
//   ~6 cm  DETECTION LAG. Front pings are 60 ms apart and the vote needs 2 of
//          them, so at the measured 40+ cm/s the front range falls ~6 cm
//          between "first reading under the threshold" and "stop fires".
//          Logs show Fraw 12 -> 9 -> 6 across those pings.
//   ~2-3cm BRAKE COAST, from the stop command to standstill. The active brake
//          is doing its job; this half is already small.
//
// So the threshold has to cover the lag as well as the coast. 20 puts the nose
// at ~11 cm, clear of DEADEND_BACKUP_TRIGGER_CM so no reverse is needed at all
// on a normal stop.
#define DEADEND_STOP_CM        20

// TWO-STAGE APPROACH -- this is what actually prevents hitting the wall, and
// why DEADEND_STOP_CM no longer has to be guessed against an unknown coast.
//
// The first attempt drove at full DRIVE_BASE_PWM right up to DEADEND_STOP_CM
// and then cut the motors. Coast distance at cruise is larger than the whole
// stopping margin, so it hit the wall -- and no value of DEADEND_STOP_CM fixes
// that reliably while the coast is unmeasured and speed-dependent.
//
// Instead: cruise until the obstacle is DEADEND_SLOW_CM away, then drop to
// DEADEND_CREEP_PWM for the last stretch. Coast scales steeply with speed, so
// creeping the final approach shrinks it to a couple of centimetres, and the
// active brake in Drive_Stop() removes most of what is left. The stop then
// lands where it was asked to regardless of what the cruise coast happens to
// be.
//
// DEADEND_SLOW_CM must be comfortably larger than the CRUISE coast, since
// that is the distance this transition has to happen within. It costs nothing
// to be generous here -- the penalty is only a slower last 20 cm.
// MEASURED FROM LOGS 1/3/4, not estimated. The creep only lasted ~0.5 s at 35
// cm, which is not enough distance for the chassis to actually shed speed --
// the front range closed at 41-46 cm/s during "creep", the same as cruise.
// Starting it at 45 cm gives ~1.5 s for the deceleration to take effect.
#define DEADEND_SLOW_CM        45

// Raised 48 -> 52. At 48 the floor-preservation in tick_at() had almost
// nothing to work with: MOTOR_MIN_PWM is 45, so any correction over 3 counts
// pushed a wheel under the floor and BOTH were lifted, taking the mean back up
// to 56-62 -- i.e. cruise speed. Logged effective means were 50.6 / 52.8 /
// 56.2 against a nominal 48. At 52 there are 7 counts of headroom, and the
// correction is clamped to that (see DRIVE_MEAN_PRESERVE_BELOW) so the mean
// stays where it was asked to be.
#define DEADEND_CREEP_PWM      52

// Do not accept a dead-end stop while the chassis is yawing faster than this:
// off-axis the front beam can be ranging a SIDE wall, and stopping on that
// reading turns a normal corridor into a phantom dead end. Raw LSB, 65.5 per
// deg/sec, so 650 ~= 10 deg/sec. A too-close front bypasses this entirely --
// that is a real collision signal, not a beam artefact.
#define DEADEND_STRAIGHT_LSB   650

// ...but do not wait forever for a straight moment either. If the stop has
// been wanted this long and the chassis still will not settle, take it anyway
// and say so in the log: a delayed stop eventually becomes a collision.
#define DEADEND_STRAIGHT_MAX_MS 700

// Per-tick trace. One line is ~56 bytes; at CONTROL_TICK_MS=20 every tick
// would be ~2800 byte/s against 3840 byte/s at 38400 baud, leaving nothing for
// the event lines. 2 puts it near 36%. Watch the `drop` figure in the run
// summary: if it is climbing, raise this.
#define DEADEND_TRACE          1
#define DEADEND_TRACE_EVERY    2

// Free space the pivot needs on the side it rotates into, measured from the
// chassis flank (which is ROBOT_WIDTH_CM/2 out from the pivot axis) -- i.e.
// how much the side sonar must be reading for the front corner to clear.
#define PIVOT_SIDE_NEED_CM     (PIVOT_FRONT_RADIUS_CM - (ROBOT_WIDTH_CM / 2))  // 9 cm

// Only commit to a side when the two walls differ by at least this much. Below
// it the readings are within sonar noise of each other and "nearer wall" is a
// coin flip, so DEADEND_TIE_DIR is used instead of chasing the noise.
#define DEADEND_DECIDE_MARGIN_CM 3
#define DEADEND_TIE_DIR          TURN_RIGHT

// Drive_Stop() has no active brake: the chassis coasts after the motors cut,
// so where it STOPS is closer to the wall than where it DECIDED to stop. The
// direction rule fixes the lateral clearance problem but does nothing for the
// front one, so when the coast leaves the nose against the wall the run backs
// up to buy the front corners room. Timed, because there are no encoders.
//
// CONDITIONAL: the reverse only runs when the post-coast front reading is
// below DEADEND_BACKUP_TRIGGER_CM (or the front reads too-close to measure).
// Above that there is already room and reversing is wasted travel that just
// puts the chassis somewhere else in the corridor. Set the trigger to 0 to
// disable the reverse entirely, or to a large number to force it every run.
//
// The default leaves (8 - PIVOT_FRONT_NEED_CM) = 6 cm of margin over the
// reading at which a front corner exactly grazes the wall.
#define DEADEND_BACKUP_TRIGGER_CM 8
#define DEADEND_BACKUP_PWM     90

// The reverse is CLOSED-LOOP now: it backs off until the front sensor reads
// DEADEND_BACKUP_TARGET_CM, watching as it goes, instead of running a fixed
// time and hoping.
//
// WHY: the fixed 400 ms pulse moved the chassis a MEASURED 20-22 cm on all
// three runs that used it (front went 3 -> 24, 3 -> 25, 4 -> 24 cm). That is
// ~52 cm/s in reverse, against a pivot that only needs about 6 cm of room --
// so it threw away most of a corridor width every time, and in a real maze
// would reverse straight into whatever was behind it. No fixed duration is
// safe here when the speed is this poorly known; the sensor already knows the
// answer, so use it.
//
// DEADEND_BACKUP_MAX_MS only bounds the loop if the sensor never reports the
// target (a wall behind, a dead sensor). At the measured reverse speed it
// corresponds to ~15 cm, so it cannot run away.
#define DEADEND_BACKUP_TARGET_CM 12
#define DEADEND_BACKUP_MAX_MS    300

// Ceiling on the approach so a mode-10 run in open space ends rather than
// driving off forever.
#define DEADEND_APPROACH_MAX_MS 20000UL

// ---------------------------------------------------------------------------
//  15c. SUPPLY MONITORING AND RESET SAFETY
// ---------------------------------------------------------------------------
// Nominal internal bandgap, millivolts. Datasheet says 1.22 V typical with a
// 1.15-1.35 V spread, so the ABSOLUTE voltage this yields can be ~10% out.
// That does not matter for what it is used for: the error is a fixed scale
// factor, so the SAG (idle reading minus minimum reading) is accurate even
// when the absolute figure is not. To calibrate a board, measure VCC with a
// meter while it idles and scale this until the reported figure matches.
#define POWER_BANDGAP_MV       1220

// The ATmega32A datasheet requires VCC >= 4.5 V to run at 16 MHz. Below this
// the part is out of its safe operating area: it may keep executing, but
// timing margins are gone and behaviour is no longer guaranteed. Any reading
// under this is a real fault, not a preference.
#define POWER_MIN_SAFE_MV      4500

// How often to sample the rail. Every control tick is fine -- a conversion is
// ~0.1 ms at 125 kHz -- and sampling often is the point: the dip that resets
// the MCU lasts a few milliseconds, so a slow sampler simply never sees it.
#define POWER_SAMPLE_EVERY     1

// REFUSE TO AUTO-RESTART AFTER A RESET THAT INTERRUPTED A RUN.
//
// This is the fix for "after the reset, the rest of the behaviour was just
// undefined". A reset does not put the robot back at the start line: it leaves
// the chassis somewhere unknown in the maze, at an unknown heading, possibly
// still coasting. Restarting the mode from scratch then drives blind from that
// unknown pose -- and worse, Gyro_CalibrateFull() runs while the chassis may
// still be moving, which poisons the gyro zero for the whole next run.
//
// With this set, a boot that finds the previous boot died mid-motion (its
// .noinit run-state flag still says MOVING, and SRAM survived so that flag is
// trustworthy) halts with the motors off and says so, instead of setting off
// again. Cycling the power for a few seconds clears SRAM and gives a normal
// cold start, so recovery is deliberate rather than automatic.
#define HALT_ON_UNSAFE_RESTART 1

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

// ---------------------------------------------------------------------------
//  17. WALL-FOLLOWER WITH MEMORY  (MODE=wallmem)
// ---------------------------------------------------------------------------
// Run 1 explores with a strict left-hand rule and logs one byte per decision.
// Run 2 replays the collapsed string. See wallmem.h for the record layout and
// why the collapse is (A + B + 2) & 3.

// Ceiling on the explore log. The designed 14-cell demo maze needs 19 records
// for the full left-hand walk and 5 after collapsing, so this is roughly 2x
// headroom. An explore run that needs more is REPORTED AS A FAILURE rather
// than truncated: a truncated string replays straight into a wall.
#define WALLMEM_MAX_RECORDS    48

// Byte offset of the saved route inside the ATmega32's 1024-byte EEPROM.
// Nothing else in this project uses EEPROM, so the base is arbitrary; it is
// named rather than literal so a second user can be added without a hunt.
#define WALLMEM_EE_BASE        0x0010

// 1 = ignore any saved route and explore again on every power-up.
// Set this to 1 to re-run the explore leg without erasing EEPROM by hand.
#define WALLMEM_FORCE_EXPLORE  0

// What run 2 does when a junction does not match the stored signature.
//   0 = drop back to the plain left-hand rule for the rest of the run
//   1 = stop and halt
// 0 is the default deliberately: a degraded run that still finishes is more
// useful on the day than a robot standing still in the middle of the maze.
#define WALLMEM_HALT_ON_MISMATCH 0

// Consecutive confirmations that the way out really is the way out. Reuses the
// same evidence as the maze solver's exit test, at the same confidence.
#define WALLMEM_EXIT_CONFIRM   OPENING_CONFIRM

#endif // CONFIG_H
