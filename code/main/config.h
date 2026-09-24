#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
//  CENTRAL CONFIGURATION -- the two-run maze solver
//
//  Every tunable number in the firmware lives here. Nothing else should
//  contain a magic constant. Sections marked [MEASURE] describe the physical
//  robot or maze and must match what you actually built; the tuning list in
//  README.md says which ones matter most and how to measure them.
// ============================================================================

#define F_CPU 16000000UL

// ---------------------------------------------------------------------------
//  1. PIN ALLOCATION
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
//   PD0 = RXD, PD1 = TXD  (38400 8N1, see section 13)
//   PD4 = OC1B = LEFT wheel PWM
//   PD5 = OC1A = RIGHT wheel PWM
#define PWM_DDR         DDRD
#define LEFT_PWM_BIT    PD4
#define RIGHT_PWM_BIT   PD5

// OPERATOR PANEL -- ready LED and start button.
//
// The two sit on different ports, so they are declared separately rather than
// sharing one PANEL_PORT. That is the whole reason panel.c has separate LED_*
// and BUTTON_* accessors.
//
// PB0 -- ready LED. PORTB has nothing else on it at all, and PB0 is clear of
// PB5/PB6/PB7 = MOSI/MISO/SCK so the ISP programmer can stay connected.
// Alternate function T0/XCK is inactive: Timer0 runs off the internal clock and
// the USART is asynchronous.
//
//   PB0 --[330R]--|>|-- GND        active high
#define LED_PORT        PORTB
#define LED_DDR         DDRB
#define LED_BIT         PB0

// PD6 -- start button. Free: PORTD carries the USART on PD0/PD1 and the motor
// PWM on PD4/PD5, and nothing else. Alternate function ICP1 (Timer1 input
// capture) is inactive -- motors.c sets TCCR1B to CS11|CS10 only, with no
// ICNC1/ICES1, and TIMSK never enables TICIE1.
//
// No external resistor: the internal pull-up holds the pin high and the switch
// pulls it to ground, so A PRESS READS LOW. panel.c hides that inversion.
//
// Note it sits next to PD5, the right wheel's PWM output. A long button lead
// run alongside that one can pick up switching noise; the pull-up plus the
// BUTTON_DEBOUNCE_TICKS filter below absorbs it, but keep the lead short and
// away from the motor wiring if you have the choice.
//
//   PD6 -----------o o-- GND       press = LOW
#define BUTTON_PORT     PORTD
#define BUTTON_DDR      DDRD
#define BUTTON_PIN      PIND
#define BUTTON_BIT      PD6

// Consecutive agreeing samples before the debounced button level moves. At
// CONTROL_TICK_MS = 20 this is 60 ms, comfortably past the few ms a panel
// button bounces for, and far too short to feel laggy.
#define BUTTON_DEBOUNCE_TICKS  3

// Hold the button this long to throw the saved route away and explore again.
// Without it, re-exploring means editing WALLMEM_FORCE_EXPLORE and reflashing,
// which during a lab session is exactly when you least want to.
#define BUTTON_LONG_PRESS_MS   2000UL

#define LED_BLINK_SLOW_MS      500   // ~1 Hz -- run 2 finished
#define LED_BLINK_FAST_MS      120   // ~4 Hz -- fault / log did not collapse

// ---------------------------------------------------------------------------
//  2. ROBOT PHYSICAL DIMENSIONS          [MEASURE ALL OF THESE]
// ---------------------------------------------------------------------------
#define ROBOT_WIDTH_CM         16   // measured (widest point incl. any overhang)

// Distance from the FRONT sonar face back to the wheel axle (the pivot centre).
// This is what makes the robot stop with its axle -- not its nose -- centred
// in an opening before it turns.
#define SONAR_TO_AXLE_CM       15   // measured

// Cruise speed in cm/s at DRIVE_BASE_PWM. With no working encoders, every
// distance in this firmware is (time x speed), so this must be measured.
//
// Measured from front-range traces rather than a stopwatch: over the steady
// part of the cruise the front range closed at 28 cm/s on two runs (51->33 cm
// in 0.65 s; 56->32 cm in 0.85 s). The old placeholder of 20 was low by 50%,
// which made every time-based distance -- APPROACH_TIME_MS above all --
// overshoot by half as much again.
//
// Caveat worth keeping in mind: late in a run the same measurement gives
// 41-46 cm/s. That is the chassis still accelerating, not a second speed, so
// 30 is the conservative figure for a leg that starts from rest. It also
// drifts with battery charge -- re-measure on a fresh pack.
#define TRAVEL_SPEED_CMS       30   // measured

// ---------------------------------------------------------------------------
//  3. MAZE GEOMETRY                       [MATCH TO YOUR BUILT MAZE]
// ---------------------------------------------------------------------------
// *** SET THIS TO YOUR ACTUAL BUILT CORRIDOR WIDTH. *** It is load-bearing in
// more places than it looks: OPENING_THRESHOLD_CM, APPROACH_DISTANCE_CM,
// EXIT_FALSE_WINDOW_CM and the emergency band below are all derived from it.
// The in-place pivot sweeps a ~34 cm circle about the axle, so at 40 cm there
// are ~3 cm of clearance per side -- if the corners clip during a turn, the
// fix is a wider corridor, not a software change.
#define CORRIDOR_WIDTH_CM      40   // wall face to wall face
#define CORRIDOR_HALF_CM       (CORRIDOR_WIDTH_CM / 2)

// Free space each side of a PERFECTLY CENTRED robot. Everything that talks
// about "how near is too near" has to be smaller than this, or the robot is
// in a fault state while doing nothing wrong. See WALL_EMERGENCY_CM.
#define CORRIDOR_SIDE_GAP_CM   ((CORRIDOR_WIDTH_CM - ROBOT_WIDTH_CM) / 2)

// What a side sonar ACTUALLY READS when the robot is centred.
//
// This is deliberately NOT CORRIDOR_HALF_CM, and the logs are why. Across four
// logged corridor runs the two side readings summed to 28-31 cm, while the
// geometry above predicts CORRIDOR_WIDTH_CM - ROBOT_WIDTH_CM = 24. The extra
// ~5 cm is real and repeatable: the sensor faces sit inboard of the widest
// part of the chassis, so each one reads a few cm more than the true gap.
//
// It does not matter while both walls are visible -- that mode steers on the
// DIFFERENCE, which cancels the offset. It matters a great deal the moment one
// wall disappears, because CENTER_LEFT_ONLY / CENTER_RIGHT_ONLY steer toward
// an absolute target: aiming at CORRIDOR_HALF_CM (20) when centred actually
// reads ~14.5 would drive the robot 5 cm off-centre on purpose, every time it
// passed an opening. In the demo maze most legs have only one wall.
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
// collide, and turn straight into the wall.
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
//  6. MOTION-SUSPECT DETECTION
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

// KICK RAMP -- brownout mitigation.
//
// Every failed run in the early brown-out logs died at a PWM-120 kick, and
// none died anywhere else: just after Drive_Begin()'s kick, immediately after
// the first pivot's kick, immediately after the second pivot's kick. The one
// run on the freshest battery survived. Every failure reported BORF with SRAM
// intact: the rail dipped below the brown-out threshold and recovered, rather
// than a broken connection.
//
// Stepping 0 -> 120 in one PWM period is the largest current transient the
// firmware ever asks for: the motor is stalled, so it draws locked-rotor
// current with no back-EMF to oppose it. Ramping over the kick spreads that
// same impulse over KICK_MS and roughly halves the peak.
//
// THIS IS MITIGATION, NOT A CURE. The root cause is supply, not firmware.
// Set to 0 to restore the old instant step.
#define KICK_RAMP              1

// ---------------------------------------------------------------------------
//  8. WALL-CENTRING CONTROLLER
// ---------------------------------------------------------------------------
// Correction = (Kp_num * error_cm)/Kp_den + (Kd_num * gyro_rate)/Kd_den
// Integer fractions are used instead of floats (no FPU on ATmega32).
//
// Sign convention: positive correction steers RIGHT.
//   left_pwm  = base + correction
//   right_pwm = base - correction
#define WALL_KP_NUM            3
#define WALL_KP_DEN            2    // 1.5 PWM counts per cm of centring error

// Gyro damping. Positive gyro Z = turning LEFT (matches the original
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

// Emergency avoidance: inside this distance the proportional correction is
// too slow. Severity RAMPS with proximity rather than slamming to full
// differential at the threshold -- measured logs show a hard +-35 step at
// exactly 8cm produced 71 deg/s of yaw and bounced the robot from one wall
// straight into the other.
//
// THIS WAS ONCE A FIXED 8 AND THAT WAS A BUG. At a 30 cm corridor the centred
// side gap is (30-16)/2 = 7 cm, so a PERFECTLY CENTRED robot sat inside the
// emergency band on BOTH sides, hard-steered away from walls it was not close
// to, and escalated into reverse-and-pivot recoveries. It was a geometry
// contradiction, not tuning.
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
// Tune against the front distance logged at a dead-end stop, not by eye.
#define DRIVE_BRAKE_PWM         100
#define DRIVE_BRAKE_MS          80

// Below this base speed, do NOT lift both wheels to keep a wheel off the stall
// floor -- clamp the correction instead, so the MEAN speed stays what was
// asked for. At cruise the solver never drops below it, so in practice this
// only matters if DRIVE_BASE_PWM is lowered close to MOTOR_MIN_PWM: with a
// base of 48 and a floor of 45, nearly every correction triggered a lift and
// the logged mean came back at cruise speed (50.6 / 52.8 / 56.2 against 48).
#define DRIVE_MEAN_PRESERVE_BELOW  58

// YAW GOVERNOR.
// Hard ceiling on how fast the chassis may rotate while centring. Beyond
// this the sonars are pointing far enough off-axis that their readings stop
// describing the corridor -- the front beam in particular walks off whatever
// is ahead. Once exceeded, the controller stops ADDING yaw in that direction
// (it does not reverse; it just stops winding up) and lets the rotation
// decay. Units: raw LSB; 65.5 LSB per deg/sec. 1300 ~= 20 deg/sec.
#define YAW_GOVERNOR_LSB       1300

// ---------------------------------------------------------------------------
//  9. JUNCTION DETECTION
// ---------------------------------------------------------------------------
// A side reads "open" beyond this. Corridor half-width plus margin.
#define OPENING_THRESHOLD_CM   (CORRIDOR_HALF_CM + 10)   // 30 cm at 40 cm corridors

// Front is considered blocked closer than this. Must be generous enough that
// at a T-junction the front wall registers as blocked quickly once the two
// side openings appear -- see EXIT_FALSE_WINDOW_CM in section 11, which is
// derived from it.
#define FRONT_BLOCKED_CM       25

// Consecutive confirmations before believing a side opening.
#define OPENING_CONFIRM        2

// Consecutive confirmations before believing a dead end (all three walled).
// Higher than OPENING_CONFIRM: a spurious U-turn is expensive, and in run 1 it
// is also written into the log.
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

// ---------------------------------------------------------------------------
//  10. APPROACH OFFSET  (front-mounted sonar compensation)
// ---------------------------------------------------------------------------
// The sonar sees an opening while the sensor is level with it, but the robot
// pivots about the AXLE, further back. Drive on by this much so the pivot
// centre ends up in the middle of the opening.
#define APPROACH_DISTANCE_CM   (SONAR_TO_AXLE_CM + CORRIDOR_HALF_CM)  // 35 cm at 40 cm corridors
#define APPROACH_TIME_MS       (((uint32_t)APPROACH_DISTANCE_CM * 1000UL) / TRAVEL_SPEED_CMS)

// When the front is blocked we cannot drive the full approach distance --
// stop this far from the wall instead and pivot there. In practice this, not
// APPROACH_TIME_MS, is what ends the approach at any junction with a front
// wall (T, forced turn, dead end).
#define FRONT_STOP_CM          12

// ---------------------------------------------------------------------------
//  11. EXIT DETECTION
// ---------------------------------------------------------------------------
// All three sensors open => exit. But a junction can look exactly the same for
// a short window, so on first detection the robot drives on and re-checks: a
// real exit stays open, a junction's front wall closes in.
//
// HOW FAR IS "ON"? The number is not free -- it is pinned by geometry at both
// ends, and a corridor width change would silently break a hard-coded value.
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
// in README.md has none -- deliberately, since a 4-way also leaves nothing for
// the wall-centring to hold on to while crossing it.

// ---------------------------------------------------------------------------
//  12. TURNS
// ---------------------------------------------------------------------------
#define TURN_PWM               85   // slow = accurate
#define TURN_KICK_PWM          KICK_PWM
#define TURN_KICK_MS           40
#define TURN_BRAKE_PWM         100
#define TURN_BRAKE_MS          20
// Measured (6 logged turns, both directions): after the motors cut, the
// chassis was still rotating above ~3 deg/sec for the WHOLE 300ms window --
// coast_ms came back 283..305 against a 300 window, and one turn was still at
// 23 deg/sec when it closed. The closed-loop correction below was therefore
// measuring a heading that had not stopped changing. 500 gives real margin.
#define TURN_SETTLE_MS         500  // motors off, still integrating coast

// |yaw rate| below this counts as "stopped" when measuring how long the
// chassis actually coasts after the motors cut (turn_result_t.coast_ms).
// Raw LSB; 65.5 LSB per deg/sec, so 200 ~= 3 deg/sec.
#define TURN_STILL_LSB         200

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
// To confirm on the bench, watch the per-turn trace (TURN_TRACE): expect the
// initial error near zero and 0-1 nudges.
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
// TURN_NUDGE_MS_PER_DEG from the per-turn trace: still 2+ nudges of the same
// sign in a row -> raise it; nudges routinely overshoot the deadband the other
// way -> lower it.
#define TURN_NUDGE_MS_MIN      8
#define TURN_NUDGE_MS_MAX      40
#define TURN_NUDGE_MS_PER_DEG  6    // ms per whole degree of residual error

#define TURN_MAX_NUDGES        5
#define TURN_TIMEOUT_MS        7000 // safety: abort a turn that never finishes

// Do a 180 as two 90s with a settle between. Usually more accurate than one
// long sweep because momentum has less time to build. Set 0 for a single 180.
#define TURN_180_AS_TWO_90S    1

// Per-phase turn trace: each phase boundary prints the heading it ended at
// (about 10 short lines per turn, ~250 bytes over ~1.5 s: no risk to the byte
// budget). This is what tells a too-short settle from a too-long coast from a
// clipping gyro. Set 0 for a quieter log once the turns are trusted.
#define TURN_TRACE             1

// U-TURN DIRECTION.
// An in-place pivot is NOT symmetric. The chassis rotates about the AXLE,
// which sits SONAR_TO_AXLE_CM behind the nose, so:
//   front corners swing  sqrt(15^2 + 8^2) = 17.0 cm from the axle
//   rear corners swing   sqrt( 7^2 + 8^2) = 10.6 cm from the axle
// The front corners sweep into the side being turned TOWARDS; the rear corners
// sweep out the opposite side. The turning side therefore needs ~6.4 cm more
// free space than the other. If the chassis is hugging the left wall, rotating
// LEFT drags the wide front corner straight into it, while rotating RIGHT
// only puts the narrow rear corner there. Hence at a dead end the solver
// rotates AWAY from the nearer wall.
//
// Only commit to a side when the two walls differ by at least this much. Below
// it the readings are within sonar noise of each other and "nearer wall" is a
// coin flip, so UTURN_TIE_DIR is used instead of chasing the noise.
#define UTURN_DECIDE_MARGIN_CM 3
#define UTURN_TIE_DIR          TURN_RIGHT

// ---------------------------------------------------------------------------
//  13. GYRO CALIBRATION
// ---------------------------------------------------------------------------
// Full calibration at power-up.
#define GYRO_CAL_SAMPLES_INIT  500
// Shorter re-calibration performed after every stop and every pivot, to catch
// thermal bias drift, and after every button press, since the operator has
// just handled the chassis. Kept brief so it does not dominate the run time.
#define GYRO_CAL_SAMPLES_QUICK 150
#define GYRO_CAL_INTERVAL_MS   2

// Calibration is only valid if the chassis is genuinely still. If the spread
// between the highest and lowest sample exceeds this, the robot was moving or
// vibrating: the result is discarded and the previous offset kept.
#define GYRO_CAL_MAX_SPREAD    250
#define GYRO_CAL_RETRIES       3
#define GYRO_SETTLE_MS         250  // wait for the chassis to stop rocking first

// Empirical trim carried over from the straight-line tuning, re-applied after
// every calibration.
#define GYRO_OFFSET_TRIM       3

// Datasheet sensitivity at +/-500 dps is 65.5 LSB per deg/sec. Heading is
// accumulated in LSB*milliseconds, so:
//     degrees = accum / (65.5 * 1000) = accum / 65500
// This is sample-rate independent, which is why the turn loop and the drive
// loop can run at different tick rates without separate constants.
// Tune by protractor: new = old * (commanded_angle / measured_angle). Every
// turn in both runs inherits this number, and a turn that lands well off 90
// puts run 2 in a corridor the stored route does not describe.
#define GYRO_LSB_MS_PER_DEGREE 65500L

// ---------------------------------------------------------------------------
//  14. RUN SAFETY
// ---------------------------------------------------------------------------
// After a turn the sonar history is meaningless. Drive gyro-only for this
// long while the filters refill.
#define RECOVER_MS             400

// Longest a single straight leg may run before forcing a junction decision.
// Safety net for walls the sonar misses entirely (angled/soft surfaces); the
// solver treats a timeout as a dead end, through the same decision path.
#define MAX_LEG_MS             10000UL

// Global run limit.
#define MAX_RUN_MS             300000UL   // 5 minutes

// Settle time after the start button is pressed, before the robot moves: long
// enough for the operator's hand to be clear and the chassis to stop rocking
// before the gyro is re-zeroed.
#define STARTUP_DELAY_MS       3000

// ---------------------------------------------------------------------------
//  15. SUPPLY MONITORING
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

// ---------------------------------------------------------------------------
//  16. DEBUG / USART
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
#define DEBUG_LEVEL            1

// Warn when a control tick overruns its deadline. A sonar timeout plus a
// blocking print used to be enough to do this; it must stay at zero.
#define TICK_OVERRUN_WARN_MS   (CONTROL_TICK_MS + 5)

// ---------------------------------------------------------------------------
//  17. WALL-FOLLOWER MEMORY
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
// Nothing else in this firmware uses EEPROM, so the base is arbitrary; it is
// named rather than literal so a second user can be added without a hunt.
#define WALLMEM_EE_BASE        0x0010

// 1 = ignore any saved route and explore again on every power-up.
// Holding the button for BUTTON_LONG_PRESS_MS does the same without a rebuild.
#define WALLMEM_FORCE_EXPLORE  0

// What run 2 does when a junction does not match the stored signature.
//   0 = drop back to the plain left-hand rule for the rest of the run
//   1 = stop and halt
// 0 is the default deliberately: a degraded run that still finishes is more
// useful on the day than a robot standing still in the middle of the maze.
#define WALLMEM_HALT_ON_MISMATCH 0

#endif // CONFIG_H
