# Two-run maze solver — ATmega32A

An autonomous maze robot that solves a maze in two runs. **Run 1** explores with
a left-hand wall follower and writes down every decision it makes. Between runs
it collapses that log into the shortest route. **Run 2** replays the route. On a
maze with no loops, run 2 is provably the optimal path, not just a shorter one.

Firmware lives in [`code/main/`](code/main). CSE 315, BRAC University.

```
make            build          (from code/main/)
make flash      build + flash  (USBasp)
make test       host unit tests, no hardware
make sim        both runs of the demo maze, closed loop, 40 simulated robots
make boot-test  boot the real image in the simavr simulator
```

Flash 26.2 KB of 32 KB, RAM 770 B of 2 KB.

---

## Running it

| LED on PB0 | meaning |
|---|---|
| off | armed for **run 1**, or driving |
| **solid** | a collapsed route is loaded — **ready for run 2** |
| slow blink (1 Hz) | run 2 finished |
| fast blink (4 Hz) | run 1's log did not collapse into a route, or a fault |

1. Put the robot in the **start cell, facing into the maze**, and power on. The
   LED is off.
2. **Press the button.** After a 3 s settle (it re-zeros the gyro while you take
   your hand away) it explores.
3. At the exit it collapses the log, saves the route to EEPROM, and the **LED
   comes on**.
4. Carry it back to the start cell, facing in. **Press the button.** It re-zeros
   again (it has just been handled) and drives the route.
5. The LED slow-blinks. A short press runs the route again.

**Hold the button 2 s** in any waiting state to discard the saved route and
explore again. The route is in EEPROM, so a power cycle between the runs also
works: it comes back up with the LED already on.

Nothing moves until the button is pressed. Each run is limited to 5 minutes of
motion (`MAX_RUN_MS`); time spent waiting between runs does not count.

---

## Wiring

| signal | pin | DIP # | notes |
|---|---|---|---|
| sonar L trig / echo | PA0 / PA1 | 40 / 39 | HC-SR04, all three mounted perpendicular |
| sonar F trig / echo | PA2 / PA3 | 38 / 37 | |
| sonar R trig / echo | PA4 / PA5 | 36 / 35 | |
| MPU6050 SCL / SDA | PC0 / PC1 | 22 / 23 | hardware TWI |
| L298N IN1–IN4 | PC2–PC5 | 24–27 | direction |
| left / right PWM | PD4 / PD5 | 18 / 19 | OC1B / OC1A |
| USART RXD / TXD | PD0 / PD1 | 14 / 15 | 38400 8N1 |
| **ready LED** | **PB0** | **1** | `PB0 → 330 Ω → LED → GND` |
| **start button** | **PD6** | **20** | `PD6 → switch → GND`, no resistor |
| RESET | — | 9 | 10 kΩ to +5 V recommended |

**Button.** One leg to PD6, the other to GND, nothing else. The firmware turns
on PD6's internal pull-up, so it reads HIGH released and LOW pressed. It must be
a **momentary** switch: a latching push-on/push-off one only "releases" on its
second push, and a press is classified on release. For a 4-pin tactile switch,
use two **diagonally opposite** legs.

**USART.** TX crosses to RX: MCU TXD (PD1) → adapter RXD, MCU RXD (PD0) →
adapter TXD, and **GND to GND**. Without the shared ground you get silence, not
garbage. `code/main/usart.sh` opens the port and logs to a file.

The LED and button pins were chosen because nothing else uses them: PORTB is
otherwise empty and PB0 is clear of the ISP pins (PB5–PB7), and PD6's alternate
function (Timer1 input capture) is switched off.

---

## How it works

### Why not flood fill

Flood fill needs the robot to know which cell it is in. This chassis has no
encoders, so a cell index would come from `time × speed` plus integrated gyro
heading, and that error is never corrected. This algorithm never needs a
position. It needs only a correct junction classification (three sonars) and a
clean 90°/180° pivot (gyro).

### Run 1 — explore

Strict left-hand rule at every cell: **LEFT > FORWARD > RIGHT > U-TURN**. One
byte is logged per **decision point** — any cell that is not a plain corridor
(forward open, both sides walled). Corridor cells are driven through and log
nothing. Both runs use the same predicate (`WallMem_IsDecision()`), which is
what keeps them aligned: run 2 cannot consume a record at a cell where run 1
wrote none.

### The record byte

```
 bit   7    6    5    4    3    2    1    0
     [ 0 ][ F ][ L ][ R ][ 0 ][ 0 ][  TURN  ]
           \___________/             \_____/
            signature                 what
           (sonar truth)             we did
```

Bits 7, 3 and 2 are always zero. That is a structural check, not padding: erased
EEPROM reads `0xFF`, which fails it, so a blank or half-written cell can never be
mistaken for a record. The signature stores the raw openness of
forward/left/right — the classifier's own input, so it cannot disagree with it.

Turns are **quarter turns clockwise**: `F=0 R=1 U=2 L=3`. That encoding does
real work — it makes the collapse one line, and the turn code doubles as the
heading delta, `head = (head + turn) & 3`.

### Between runs — collapse

Every wasted move in run 1 is an excursion into a dead end: turn `A`, go in,
U-turn, come back, turn `B`. The net rotation is `A + 180° + B`, so

```c
net = (a + b + 2) & 3;
```

replaces the three records with one, keeping `A`'s signature (the first arrival
at that junction, which run 2 repeats). Repeat until no U-turn remains.

Three of the nine combinations fold to another U-turn. That is correct: the
junction's whole subtree was a dead end, so the robot must back out of the
junction too, and that U-turn folds again on the next pass. A U-turn that
survives to the end means the maze has a loop or a junction was misread; the
log is then not a route, and the firmware erases it instead of saving it.

### Run 2 — replay

At every decision point: pop the next byte, **check the stored signature
against what the sonars see now**, and only then act on the turn. If a pivot
overshot and the robot is in the wrong corridor, the junction won't match and
it says so before acting on a stale instruction. It then falls back to the
plain left-hand rule for the rest of the run (`WALLMEM_HALT_ON_MISMATCH 0`) — a
degraded run that finishes beats one that stops mid-maze.

### Storage

```
+0  magic 'W'   <- written LAST       +4  flags (bit0 = solved route)
+1  magic 'M'                         +5  CRC-8 over +2..+4 and the records
+2  format version                    +8  records
+3  record count
```

The magic byte is cleared first and written last, so a brown-out mid-write
leaves a block with no magic, which fails validation — the next boot explores
again instead of driving half a route. Load checks magic, version, count, every
record's reserved bits, and the CRC.

### Classifying a junction

A junction is **not** classified the moment something opens up. At that
instant the side sonar has only just reached the opening, and the front sonar
is still a whole corridor width short of the wall that ends the cell. Deciding
there misread two junctions in the first real test: the E1 corner was logged as
`FWD_OR_LEFT` (front wall still 33 cm off), and the T at D1 was logged as
`FWD_OR_RIGHT`, because the left sonar was still getting an echo off the end of
the C0|D0 wall stub.

So the robot **drives on `JUNCTION_LOOK_CM` (18 cm) and looks** (state 4,
`LOOK`). A side counts as open if it read open at any point in the look; the
front is judged at the end, when a T or corner wall is well inside
`FRONT_BLOCKED_CM`. If it meets a wall ahead first, it decides right there.
Dead ends need no special case: a front wall with both sides shut keeps looking
until `FRONT_STOP_CM`, so the 180 happens in the middle of the dead-end cell.

What the sonars say is debounced in **fresh pings**, not control ticks (each
sensor is pinged every third tick):

- a side is open when **4 of its last 6 pings** read open. A lost echo reads
  as open and HC-SR04 dropouts come in bursts, so a short run is not evidence;
  but one stray echo off a wall end in the middle of a real opening must not
  reset the count either.
- nothing counts until every sonar has been pinged since the last flush — the
  flushed placeholder reads as open, which produced a junction on the first
  tick after `GO` in both real logs.
- the front stop needs **2 of the last 3 pings** under `FRONT_STOP_CM`: one
  garbage echo can't stop the robot, and a sonar losing every other echo near
  a wall can't hide it.

Every opening is **dated from where it really starts**, not where it was
confirmed: the end of a wall echoes from well off-axis, so the reading only
flips to "open" `OPENING_DETECT_LAG_CM` (7 cm) past the edge. The turning point
is measured from that date, so the look costs no positioning accuracy.

Going **straight through** a junction masks its openings until the junction
cell has been crossed (`FWD_PASS_MS`). It used to re-fire every 40 ms for as
long as the opening was in view — 11 records for one junction in the real log.
It is time, not "until a wall is seen", because in the demo maze D0 and C0 sit
side by side with only the end of a stub between them.

### Telling a T-junction from the exit

All three sensors open means the exit — but a T-junction looks the same for a
moment: as the front sonar enters the cell, its far wall is 40 cm away and reads
open while both sides have just opened. So the look is extended: a junction's
front wall closes in, the exit's never does.

```
EXIT_FALSE_WINDOW_CM = CORRIDOR_WIDTH_CM - FRONT_BLOCKED_CM = 40 - 25 = 15 cm
EXIT_CONFIRM_CM      = EXIT_FALSE_WINDOW_CM + margin         = 15 + 15 = 30 cm
```

15 cm is the longest an in-maze junction can impersonate the exit. The exit is
then judged on what the sonars say at that moment, not on everything they said
during the look. **This assumes no 4-way crossroads** — at one, all three
sensors stay open for a whole cell and it would read as the exit.

### Turning square, and staying square

The first real test drove off the first turn at an angle, and everything after
it went wrong. The causes, and what replaced them:

- **The turn stopped short.** The sweep is cut early and the chassis coasts the
  rest. `TURN_STOP_MARGIN_DEG` assumed 35° of coast (measured on an older,
  faster build); this robot coasts about 22°, so every turn landed at 71–81°
  and nudged 2–5 times. The coast is now **learned** from every turn, as is how
  far one nudge moves the robot, starting from 25° and 8 ms/°.
- **Every turn was "90° from wherever it pointed".** Any angle the robot
  already had — a correction in progress when it braked, the yaw from the
  drive-off — was carried straight into the next corridor. The firmware now
  keeps a **maze-grid heading**: zeroed when the run starts (the robot is
  placed square) and moved by exactly 90° per quarter turn. Each turn aims at
  the next grid heading, so it also removes whatever angle the robot had.
- **Nothing held a heading while driving.** The controller only damped yaw, so
  a knock was slowed but never undone; your logs show 25–68°/s of yaw at the
  start of legs, and the robot then sat 8 cm from a wall. The drive now
  **steers back onto the grid heading** (`HEADING_KP_*`), in cooperation with
  the wall centring.
- **Blind spots in the heading.** The brake and drive-off pulses were blocking
  waits that ignored the gyro, and those are exactly where the chassis yaws
  hardest. They now integrate it.
- **The grid is kept honest by the walls.** Integrated gyro drifts with its
  scale error (the MPU-6050's tolerance is ±3%, and run 1 turns up to ~1000°
  net). Driving along a wall, the rate its distance changes *is* the robot's
  angle to it; a least-squares fit over the last 8 pings, at the millimetre
  resolution the echo time really has, nudges the grid toward it. The last few
  cm before a wall ends are ignored (a wall end's echo bends the fit). The
  corrections are also used to **learn the gyro scale** during the run, capped
  at ±5%; the run summary prints it as `gyro_trim_ppt10`.
- **Pivoting off-centre.** Before any turn at a front wall the robot measures
  that wall and creeps until its **axle** is in the middle of the cell (the
  robot's roll-out after braking varies with battery and floor). A 180 is two
  quarter turns, and between them the robot faces a side wall of the dead end
  and re-centres on that too — the pivot sweeps ~17 cm in a 20 cm half-width,
  so this is what makes it clean. Which way to turn is judged from the axle
  offset, corrected for the sonars sitting ahead of the axle.

---

## Tuning on the real robot

Everything is in `code/main/config.h`. Work in this order — later values are
tuned against earlier ones, and a dozen constants are derived from the first
four (don't edit the derived ones; four `#error` guards stop the build if your
measurements contradict each other).

| order | constant | now | how |
|---|---|---|---|
| 1 ruler | `ROBOT_WIDTH_CM` | 16 | widest point, including overhang |
| | `SONAR_TO_AXLE_CM` | 15 | front sonar face back to the driven axle |
| | **`SIDE_SONAR_TO_AXLE_CM`** | 12 | **side** sonar faces back to the axle — *estimated from the logs, measure it* |
| | `CORRIDOR_WIDTH_CM` | 40 | wall face to wall face, as built |
| 2 sonar | `SIDE_CENTRED_CM` | 15 | park centred, halve the logged `L+R` |
| | `FRONT_BLOCKED_CM` | 25 | must stay below `CORRIDOR_WIDTH_CM` |
| 3 speed | `TRAVEL_SPEED_CMS` | 30 | time a straight leg over a measured distance; re-check on a fresh battery |
| | `MOTOR_MIN_PWM` | 45 | lowest PWM where both wheels turn from rest |
| | `DRIVE_BASE_PWM` | 60 | cruise |
| | `DRIVE_SPINUP_MS` | 250 | time lost reaching cruise from rest (the first 400 ms after a turn covered ~4 cm, not 12) |
| 4 turns | **`GYRO_LSB_MS_PER_DEGREE`** | 65500 | protractor: `new = old × commanded / measured` |
| | `TURN_STOP_MARGIN_DEG` | 25 | only the *starting* coast estimate — it is learned per turn |
| | `TURN_SETTLE_MS` | 500 | raise if `coast_ms` lands near it |
| 5 centring | `WALL_KP_*` / `WALL_KD_*` | 3/2, 1/65 | drifts into walls → more P; weaves → less P or more D |
| | `HEADING_KP_*` | 3/2 | legs still start at an angle → raise; weaves after a turn → lower |
| 6 junctions | `OPENING_VOTES` / `OPENING_WINDOW` | 4 / 6 | phantom junctions → raise votes; missed ones → lower |
| | `OPENING_DETECT_LAG_CM` | 7 | turns into a side branch too late → raise, too early → lower |
| | `FRONT_STOP_CM` | 14 | the log's `centre front=` should mostly read 5–7 |
| | `EXIT_CONFIRM_MARGIN_CM` | 15 | see "Telling a T-junction from the exit" |

**Calibrate the gyro scale.** The firmware learns it during a run, but only
after the robot has turned 180° net, so the first dead end relies on the
calibration. In simulation a ±1% gyro passes every robot; at the MPU-6050's
uncalibrated ±3% worst case every robot still solves the maze, but some end
the first dead-end 180 up to ~13° off square. The run summary
prints the learned trim as `gyro_trim_ppt10` (parts per 10,000): if it says
more than about ±15 run after run, apply it to `GYRO_LSB_MS_PER_DEGREE`
instead. Dedicated bench tests for each stage (sonar cone, square, corridor,
turn debugging, dead-end 180) live on the `claude/busy-fermat-axkncy` branch.

### Reading the log

After the banner, one CSV line every 100 ms:

```
st,L,F,R,lok,rok,near,fv,md,br,err,wt,gt,corr,pwmL,pwmR,rock,rate,gx,gy,ovr,drop
```

| column | meaning |
|---|---|
| `st` | solver state: 0 ARMED, 1 STARTUP, 2 DRIVING, 3 APPROACH, 4 LOOK, 5 STOPPING, 6 RECAL, 7 DECIDE, 8 RECOVER, 9 DONE, 10 DONE_IDLE, 11 FAULT |
| `L F R` | sonar medians, cm (999 = no echo) |
| `lok rok` | side readings valid |
| `near` | too-close flags: 1 left, 2 right, 3 both |
| `fv` | front votes (of the last 5 pings) |
| `md` | centring mode: 0 both walls, 1 left only, 2 right only, 3 gyro only |
| `br` | drive branch: 0 normal, 1 rocking, 2/3 emergency L/R, 4 wall-stuck recovery |
| `err wt gt corr` | wall error (cm), wall P term, yaw damping + grid heading hold, final correction |
| `pwmL pwmR` | what reached the motors |
| `rock rate gx gy` | motion-suspect flag, yaw rate, raw X/Y gyro |
| `ovr` | control ticks that overran |
| `drop` | debug bytes lost to a full TX buffer — should stay 0 |

Between the CSV lines:

| line | meaning |
|---|---|
| `junction T_LEFT_RIGHT -> LEFT` | a decision: what it saw, what it did |
| `centre front=9 ms=200` | re-centring on a wall before a pivot: reading, and the step taken (− = back) |
| `uturn L=14 R=15 -> right` | the 180's direction and the readings it came from |
| `T kick/sweep/brake/settle/nudge hdg10=` | per-phase turn trace, tenths of a degree |
| `ang10= grid10= nudges= conv=` | the turn: rotation, where it ended vs the maze grid, nudges used, converged |
| `look: corridor after all` | a look found no junction — fine if rare, a missed record if at a real one |

At the end of run 1 it prints the full explore log, one line per record (raw
hex plus decoded meaning), then the collapsed route.

---

## Tests

- **`make test`** compiles the same `wallmem.c` and `panel.c` the firmware links
  against. `test_wallmem.c` runs the demo maze: run 1 produces the designed
  19-record log, it collapses to 5, survives an EEPROM round trip, and replays
  `E0 E1 D1 C1 C2 D2 E2` and out, consuming the route exactly — plus all nine
  fold cases, a nested dead end, the bit layout, log overflow and four EEPROM
  corruptions. `test_panel.c` checks the button: no launch from a low line at
  power-up, one event per press, long press instead of short, bounce rejection.
- **`make sim`** runs the whole job closed loop. `test/sim/sim.c` compiles the
  real `main.c`, `solver.c`, `drive.c`, `turn.c`, `heading.c`, `sonar.c`,
  `wallmem.c` and friends against a physical model of the demo maze, and
  drives the sonar's trigger and echo pins from it — so the firmware's own
  ping timing, filtering and voting are what get tested. The model is
  calibrated to the real logs (coast, nudges, roll-out, the drive-off yaw) and
  includes what broke the first real run: wide beams that echo off wall ends,
  dropouts and garbage readings, gyro scale error. 40 robots with those
  randomised must each explore with exactly the designed decisions, collapse
  to 5 records, replay the route and exit, never touch a wall, and never end a
  pivot more than 8° off the true maze grid. `test/sim/run.sh [N] [seed]` runs
  more; `SIM_SCALE_PCT=3` sets the gyro error range.
- **`make boot-test`** boots the real image in simavr and fails unless it
  reaches `READY FOR RUN 1` with zero dropped debug bytes. Compiling proves
  nothing about whether the chip starts; this does. Needs simavr (`apt install
  simavr` / `brew install simavr`); skips cleanly without it.

---

## Files

| file | role |
|---|---|
| `main.c` | brings the hardware up in a safe order, calibrates, runs the 20 ms control tick |
| `solver.c/.h` | the two-run state machine: button, LED, explore, collapse, replay |
| `wallmem.c/.h` | the memory: record format, left-hand rule, collapse, replay, EEPROM |
| `drive.c/.h` | wall-centring controller, active brake |
| `turn.c/.h` | closed-loop 90°/180° pivots |
| `heading.c/.h` | gyro calibration, heading integration, rocking detection |
| `sonar.c/.h` | three HC-SR04s, filtering, front voting |
| `motors.c/.h`, `mpu6050.c/.h`, `i2c.c/.h`, `timer.c/.h` | drivers |
| `panel.c/.h` | ready LED and start button |
| `power.c/.h`, `resetlog.c/.h` | supply monitor and reset forensics (brown-out diagnosis) |
| `telemetry.c/.h`, `debug.c/.h` | CSV log line, non-blocking USART |
| `config.h` | every tunable number |
| `test/` | host unit tests; `test/sim/` the closed-loop simulator |
| `usart.sh` | open the serial port and log to a file |
