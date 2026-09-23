# Wall follower with memory (`make MODE=wallmem`)

Two runs through the same maze. The first explores and writes down what it did;
the second replays the shortest route. On a maze with no loops the second run is
provably optimal, not merely better.

## Why not flood fill

Flood fill needs the robot to know **which cell it is in**. This chassis has no
encoders, so that cell index comes from `time x speed` plus integrated gyro
heading, and the error it accumulates is never corrected.

This algorithm never needs to know where it is. It needs only:

1. a correct junction classification (three sonars), and
2. a clean 90/180 degree pivot (gyro).

Both are things the firmware already does well.

## Run 1 — explore

Strict left-hand rule at every cell: **LEFT > FORWARD > RIGHT > U-TURN**
(`WallMem_LeftHand()`). One byte is logged per **decision point**, where a
decision point is any cell that is not a plain corridor
(`WallMem_IsDecision()`). Corridor cells are driven through and log nothing.

That predicate is used by *both* runs, and that is what keeps the string
aligned: run 2 cannot consume a record at a cell where run 1 did not write one,
because both ask the same question of the same three bits.

## The record byte

```
 bit   7    6    5    4    3    2    1    0
     [ 0 ][ F ][ L ][ R ][ 0 ][ 0 ][  TURN  ]
           \___________/             \_____/
            signature                 what
           (sonar truth)             we did
```

Bits 7, 3 and 2 are always zero. That is a structural check, not padding:
erased EEPROM reads `0xFF`, which fails it, so a blank or half-written cell can
never be mistaken for a record (`WallMem_RecordIsSane()`).

The signature stores raw **openness** of forward/left/right rather than the
derived junction enum. Same three bits, but the raw form is what run 2 compares
against and it cannot disagree with the classifier, because it *is* the
classifier's input.

Turns are **quarter turns clockwise**:

| code | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| | `F` 0&deg; | `R` 90&deg; | `U` 180&deg; | `L` 270&deg; |

This encoding is doing real work. It makes the collapse one line, and it makes
the turn code double as the heading delta — `head = (head + turn) & 3` — which
is exactly how the host test walks the maze.

## Between runs — collapse

Every wasted move in run 1 is an excursion into a dead-end branch: turn `A`, go
in, `U`-turn, come back, turn `B`. The net rotation is `A + 180 + B`, so

```c
net = (a + b + 2) & 3;
```

replaces all three records with one, keeping `A`'s signature (`A` is the first
arrival at that junction, and run 2 arrives the same way). Repeat until no `U`
remains — `WallMem_Reduce()`.

Three of the nine cases fold to a `U` again. That is correct, not a bug: it
means the junction's entire subtree was a dead end, so the robot must reverse
out of the junction too. That `U` then folds with *its* neighbours on the next
pass.

**A `U` that survives to the end means the maze has a loop, or a junction was
misread.** The string is then not a route, and the firmware erases rather than
saves it.

## Run 2 — replay

At every decision point, pop the next byte, **compare the stored signature
against what the sonars see right now**, then act. The signature check is the
safety net: if a pivot overshot and the robot is in the wrong corridor, the
junction will not match, and it says so *before* acting on a stale instruction.

Default response is to fall back to the plain left-hand rule for the rest of the
run (`WALLMEM_HALT_ON_MISMATCH 0`) — a degraded run that finishes beats a robot
standing still in the middle of the maze.

## Storage

The two runs are separate power-ups, so the route lives in EEPROM at
`WALLMEM_EE_BASE`:

```
+0  magic 'W'     <- written LAST
+1  magic 'M'
+2  format version
+3  record count
+4  flags   bit0 = collapsed, solved route
+5  CRC-8 over +2..+4 and every record
+6  reserved
+7  reserved
+8  records
```

The magic byte is cleared first and written last, so a brown-out part way
through a write — a real event on this chassis, not a theoretical one — leaves a
block with no magic. It fails validation, and the next power-up explores again
instead of driving half a route. Load validates magic, version, count ceiling,
every record's reserved bits, and the CRC.

## The operator panel

Two pins on **PORTB**, the only port the firmware had nothing on:

```
PB0 --[330R]--|>|-- GND      LED, active high
PB1 -----------o o-- GND     button to ground, internal pull-up, press = LOW
```

No external resistor on the button — the internal pull-up holds the pin high
and the switch pulls it down. PB0 and PB1 are adjacent, so it is one 3-pin
header (LED, BUTTON, GND), and both sit clear of PB5/PB6/PB7 = MOSI/MISO/SCK,
so **the ISP programmer can stay plugged in** while the panel is wired. Both
alternate functions on these pins are inactive in this build: PB0 is T0/XCK
(Timer0 runs off the internal clock, the USART is asynchronous) and PB1 is T1
(Timer1 runs off the internal clock).

### What the LED means

| LED | state |
|---|---|
| off | ready for **run 1**, or driving |
| **solid** | a collapsed route is loaded — **ready for run 2** |
| slow blink (1 Hz) | run 2 finished |
| fast blink (4 Hz) | the explore log did not collapse to a route, or the robot halted |

"Ready" and "finished" are deliberately different lights, and so are "not ready"
and "not finished yet" — a dark LED after a failed run would look exactly like a
run still in progress.

### Operating procedure

```
make MODE=wallmem flash
```

1. Put the robot in the **start cell facing into the maze**. Power on.
   The LED is **off**: it is armed for run 1.
2. **Press the button.** After a 3 s settle (during which it re-zeros the gyro)
   it explores.
3. It reaches the exit, collapses the log, saves to EEPROM, and the
   **LED comes on**. That is the signal that run 2 is loaded and ready.
4. Carry the robot back to the start cell, facing in. **Press the button.**
   It re-zeros the gyro again — it has just been handled — and drives the
   optimal route.
5. LED slow-blinks. A short press runs it again.

Nothing moves until the button is pressed, run 1 included. A robot that drives
off on a timer while it is still being positioned is the failure that removes.

**Hold the button for 2 s** in any waiting state to throw the saved route away
and explore again. Without it, re-exploring means editing
`WALLMEM_FORCE_EXPLORE` and reflashing, which during a lab session is exactly
when you least want to.

The route is still written to EEPROM, so a power cycle between the runs also
works: the robot comes back up with the LED already on, waiting for the button.
That matters on this chassis, where the power cycle is sometimes not the
operator's choice.

## The demo maze

14 cells, 200 x 120 cm, 40 cm corridors, **21 wall faces = 840 cm**.

```
+    +----+----+----+  ^ +      ^ = exit gap  (north of E2)
     | B2   C2   D2   E2 |      v = entry gap (south of E0)
+----+----+    +----+----+
| A1   B1 | C1   D1   E1 |      1 cell = 40 cm
+    +    +    +    +    +
| A0 | B0   C0 | D0 | E0 |
+----+----+----+----+  v +
```

Seven interior walls: `A0|B0`, `B1|B2`, `B1|C1`, `C0|D0`, `D0|E0`, `D1|D2`,
`E1|E2`. It is a spanning tree — exactly one route between any two cells, which
is the condition the optimality guarantee rests on.

|                  | run 1  | run 2 |
|---|---|---|
| path             | 840 cm | 280 cm |
| junction-to-junction legs | 21 | 7 |
| 90&deg; turns    | 14     | 4 |
| 180&deg; turns   | 3      | 0 |
| time (estimated) | ~63 s  | ~15 s |
| records          | 19     | 5 |

All seven junction types appear in run 1, including three dead ends of different
shapes — one cell deep (`D0`), one cell deep off a T (`B2`), and a four-turn
spiral (`C1 -> C0 -> B0 -> B1 -> A1 -> A0`).

One known weak spot: leg 3 (`D1` heading south into the `D0` branch) has
openings on both sides, so for 40 cm there is nothing for the wall-centring PD
to hold and it runs on gyro heading alone. That is unavoidable at this wall
budget — 7 walls across 20 internal edges leaves the maze 65% open.

## Telling a T-junction from the exit

Both look identical for a moment. When the front sonar crosses into a junction
cell, that cell's far wall is `CORRIDOR_WIDTH_CM` ahead and so reads **open**,
and both sides have just opened too — all three sensors open, exactly like the
way out. It happens at `C1` and `C2`, the two T-junctions, on every run.

The distinction is made by driving. On first sighting the robot enters
`WM_CONFIRM_EXIT` and keeps going: a junction's front wall closes in, the exit's
never does. The window is derived rather than guessed —

```
EXIT_FALSE_WINDOW_CM = CORRIDOR_WIDTH_CM - FRONT_BLOCKED_CM   = 40 - 25 = 15 cm
EXIT_CONFIRM_CM      = EXIT_FALSE_WINDOW_CM + MARGIN          = 15 + 10 = 25 cm
```

15 cm is the *longest* an in-maze junction can impersonate the exit, because the
front wall is only ever one cell away. 25 cm clears it with 10 cm to spare, and
a compile-time guard fails the build if that margin drops below two sonar-confirm
periods of travel.

The confirm costs no extra travel. It bails on the **raw** front reading, which
arrives at the same distance at which the junction would have been classified
anyway — one control tick, then straight into the normal approach. The log
prints how far into the window the wall appeared, which is the margin you cannot
compute from a datasheet: if it creeps toward 25 cm on real cardboard, raise
`EXIT_CONFIRM_MARGIN_CM`.

**This assumes no 4-way crossroads.** At a true crossroads all three sensors
stay open for a whole cell — longer than the window — and the robot would call
it the exit. The maze above has none, deliberately: a 4-way also leaves the
wall-centring nothing to hold while crossing it.

## Test

```
make test
```

Runs two suites, both compiling the **same source the firmware links against**.

`test_wallmem.c` takes `wallmem.c` against a model of
the maze above, with no hardware. It checks that run 1 produces the designed
19-record log, that it collapses to the 5-record route, that the route survives
an EEPROM round trip, and that replaying it drives `E0 E1 D1 C1 C2 D2 E2` and
out — consuming the string exactly. It also checks all nine fold cases, a
four-turn nested dead end, the record bit layout, log overflow, and four ways of
corrupting EEPROM (blank, flipped bit, missing magic, impossible count).

`test_panel.c` takes `panel.c` against shimmed pins and clock: that a low line
at power-up does not launch the robot, that a short press fires exactly once on
release, that a long press fires the hold event *instead of* the short one, that
switch chatter shorter than the debounce fires nothing, that holding the button
does not stream events, and that the LED modes drive and blink the pin.
