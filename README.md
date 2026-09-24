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
make boot-test  boot the real image in the simavr simulator
```

Flash 19.5 KB of 32 KB, RAM 486 B of 2 KB.

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

### Telling a T-junction from the exit

All three sensors open means the exit — but a T-junction looks the same for a
moment: as the front sonar enters the cell, its far wall is 40 cm away and reads
open while both sides have just opened. So on first sighting the robot keeps
driving: a junction's front wall closes in, the exit's never does.

```
EXIT_FALSE_WINDOW_CM = CORRIDOR_WIDTH_CM - FRONT_BLOCKED_CM = 40 - 25 = 15 cm
EXIT_CONFIRM_CM      = EXIT_FALSE_WINDOW_CM + margin         = 15 + 10 = 25 cm
```

15 cm is the longest an in-maze junction can impersonate the exit. The log
prints how far into the window the wall appeared (`not the exit -- front wall at
N cm of 25 cm`); if N creeps toward 25 on the real maze, raise
`EXIT_CONFIRM_MARGIN_CM`. **This assumes no 4-way crossroads** — at one, all
three sensors stay open for a whole cell and it would read as the exit.

---

## The demo maze

14 cells, 200 × 120 cm, 40 cm corridors, **21 wall faces = 840 cm**.

```
+    +----+----+----+  ^ +      ^ = exit gap  (north of E2)
     | B2   C2   D2   E2 |      v = entry gap (south of E0)
+----+----+    +----+----+
| A1   B1 | C1   D1   E1 |      1 cell = 40 cm
+    +    +    +    +    +
| A0 | B0   C0 | D0 | E0 |
+----+----+----+----+  v +
```

Interior walls: `A0|B0`, `B1|B2`, `B1|C1`, `C0|D0`, `D0|E0`, `D1|D2`, `E1|E2`. It
is a spanning tree — exactly one route between any two cells, which is what the
optimality guarantee needs.

|                  | run 1  | run 2 |
|---|---|---|
| path             | 840 cm | 280 cm |
| legs             | 21     | 7 |
| 90° turns        | 14     | 4 |
| 180° turns       | 3      | 0 |
| time (estimated) | ~63 s  | ~15 s |
| log records      | 19     | 5 |

Run 1 meets all seven junction types, including a four-turn spiral dead end
(`C1 → C0 → B0 → B1 → A1 → A0`). Run 2 drives `E0 E1 D1 C1 C2 D2 E2` and out.

**Cut list** (11 pieces, origin at the bottom-left corner):

| length | orientation | position |
|---|---|---|
| 160 cm | horizontal | y = 0, x 0 → 160 |
| 80 cm  | horizontal | y = 80, x 0 → 80 |
| 80 cm  | horizontal | y = 80, x 120 → 200 |
| 120 cm | horizontal | y = 120, x 40 → 160 |
| 80 cm  | vertical   | x = 0, y 0 → 80 |
| 40 cm  | vertical   | x = 40, y 0 → 40 |
| 40 cm  | vertical   | x = 40, y 80 → 120 |
| 40 cm  | vertical   | x = 80, y 40 → 80 |
| 40 cm  | vertical   | x = 120, y 0 → 40 |
| 40 cm  | vertical   | x = 160, y 0 → 40 |
| 120 cm | vertical   | x = 200, y 0 → 120 |

Known weak spot: leg 3 (`D1` heading south into the `D0` branch) has openings on
both sides, so for 40 cm the centring has no wall and holds heading on the gyro
alone.

---

## Tuning on the real robot

Everything is in `code/main/config.h`. Work in this order — later values are
tuned against earlier ones, and a dozen constants are derived from the first
four (don't edit the derived ones; two `#error` guards stop the build if your
measurements contradict each other).

| order | constant | now | how |
|---|---|---|---|
| 1 ruler | `ROBOT_WIDTH_CM` | 16 | widest point, including overhang |
| | `SONAR_TO_AXLE_CM` | 15 | front sonar face back to the driven axle |
| | `CORRIDOR_WIDTH_CM` | 40 | wall face to wall face, as built |
| 2 sonar | `SIDE_CENTRED_CM` | 15 | park centred, halve the logged `L+R` |
| | `FRONT_BLOCKED_CM` | 25 | must stay below `CORRIDOR_WIDTH_CM` |
| 3 speed | `TRAVEL_SPEED_CMS` | 30 | time a straight leg over a measured distance; re-check on a fresh battery |
| | `MOTOR_MIN_PWM` | 45 | lowest PWM where both wheels turn from rest |
| | `DRIVE_BASE_PWM` | 60 | cruise |
| 4 turns | **`GYRO_LSB_MS_PER_DEGREE`** | 65500 | protractor: `new = old × commanded / measured` |
| | `TURN_STOP_MARGIN_DEG` | 35 | per-turn trace: initial error consistently + → lower, − → raise |
| | `TURN_SETTLE_MS` | 500 | raise if `coast_ms` lands near it |
| 5 centring | `WALL_KP_*` / `WALL_KD_*` | 3/2, 1/65 | drifts into walls → more P; weaves → less P or more D |
| 6 junctions | `OPENING_CONFIRM` | 2 | missed junctions → lower; phantom ones → raise |
| | `DEADEND_CONFIRM` | 3 | keep above `OPENING_CONFIRM` |
| | `EXIT_CONFIRM_MARGIN_CM` | 10 | see "Telling a T-junction from the exit" |

The gyro scale matters most: every turn in both runs inherits it, and a turn
that lands well off 90° puts run 2 in a corridor the route doesn't describe — you
will see it as `REPLAY MISMATCH`. Dedicated bench tests for each stage (sonar
cone, square, corridor, turn debugging, dead-end 180) live on the
`claude/busy-fermat-axkncy` branch.

### Reading the log

After the banner, one CSV line every 100 ms:

```
st,L,F,R,lok,rok,near,fv,md,br,err,wt,gt,corr,pwmL,pwmR,rock,rate,gx,gy,ovr,drop
```

| column | meaning |
|---|---|
| `st` | solver state: 0 ARMED, 1 STARTUP, 2 DRIVING, 3 APPROACH, 4 CONFIRM_EXIT, 5 STOPPING, 6 RECAL, 7 DECIDE, 8 RECOVER, 9 DONE, 10 DONE_IDLE, 11 FAULT |
| `L F R` | sonar medians, cm (999 = no echo) |
| `lok rok` | side readings valid |
| `near` | too-close flags: 1 left, 2 right, 3 both |
| `fv` | front votes (of the last 5 pings) |
| `md` | centring mode: 0 both walls, 1 left only, 2 right only, 3 gyro only |
| `br` | drive branch: 0 normal, 1 rocking, 2/3 emergency L/R, 4 wall-stuck recovery |
| `err wt gt corr` | wall error (cm), P term, D term, final correction |
| `pwmL pwmR` | what reached the motors |
| `rock rate gx gy` | motion-suspect flag, yaw rate, raw X/Y gyro |
| `ovr` | control ticks that overran |
| `drop` | debug bytes lost to a full TX buffer — should stay 0 |

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
| `test/` | host unit tests |
| `usart.sh` | open the serial port and log to a file |
