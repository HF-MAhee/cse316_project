# AGV Maze Solver — Tuning & Build Guide

All tunables live in `config.h`. Nothing else contains a magic number.

---

## 1. Values you MUST change (marked `[MEASURE]` in config.h)

| Constant | Placeholder | How to get the real value |
|---|---|---|
| `ROBOT_LENGTH_CM` | 20 | Ruler, bumper to bumper |
| `ROBOT_WIDTH_CM` | 15 | Ruler, widest point including overhang |
| `SONAR_TO_AXLE_CM` | 10 | Front sonar face → wheel axle. **Most important single number.** |
| `TRAVEL_SPEED_CMS` | 20 | Drive a timed 100 cm run at `DRIVE_BASE_PWM`, divide |
| `CORRIDOR_WIDTH_CM` | 40 | Match your actual built maze |

Everything else has a sane default but will benefit from tuning.

---

## 2. Build order

Set `BUILD_MODE` in `main.c`. Do not skip ahead — each mode proves one layer.

### Mode 0 — Sonar telemetry (no motion)
Robot stationary. Confirm over serial:
- All three distances read plausibly, and track a hand moved toward each sensor
- Readings don't jump when a *different* sensor fires (crosstalk check)
- `rock=1` appears when you tilt the chassis by hand, and clears after ~120 ms

If crosstalk appears, raise `CONTROL_TICK_MS`.

### Mode 1 — Wall centering
Straight corridor, 1.5 m. Start the robot deliberately off-centre and skewed.
- It should converge to the middle within ~50 cm and hold
- Oscillating → lower `WALL_KP_NUM`, or raise `WALL_KD_NUM`
- Sluggish → raise `WALL_KP_NUM`
- Drifts into a wall → check `mode` in telemetry; if it's flipping between
  0 and 3, the sonar is dropping out (see §4)

**Gyro damping sign:** if adding `WALL_KD_NUM` makes it *worse* rather than
damping, your gyro sign is inverted relative to my assumption — negate
`WALL_KD_NUM`.

Then measure `TRAVEL_SPEED_CMS` here.

### Mode 2 — Turn accuracy
Tape a floor mark. Run, measure the actual angle with a protractor.

```
new GYRO_LSB_MS_PER_DEGREE = old × (90 ÷ measured_angle)
```

Starting value 65500 is *calculated*, not guessed (65.5 LSB/dps × 1000 ms),
so it should be close. Repeat until within ~2°.

**Turn errors compound** — 5° per corner leaves you 20° crooked after four.
Do not proceed until this is tight.

### Mode 3 — Full maze
Build up: straight corridor → one T-junction → dead end → full maze.

---

## 3. Maze construction spec

### Corridor width: **40 cm** (recommended)
- Pivot radius `R = ½√(L² + W²)` = 12.5 cm for a 20×15 cm chassis
- Absolute minimum = 2R = 25 cm
- 40 cm gives **7.5 cm clearance per side** during an in-place pivot
- Also keeps both side walls at 20 cm when centred — comfortably inside
  sonar range and well clear of the 3 cm minimum

Don't go much above 45 cm: the side sonars start missing walls, and
`OPENING_THRESHOLD_CM` loses its margin.

### Corridor length: **1.5 m is good**
Budget per leg: 30 cm approach offset + 40 cm post-turn recovery + centring
convergence. 1.5 m leaves ~80 cm of stable centred driving. Minimum usable is
about 80 cm; below that the robot barely settles before the next junction.

### Wall height: **≥ 15 cm**
HC-SR04 beam cone is ~15° total. With sensors at 7 cm height, at 20 cm range
the beam spreads ±2.6 cm — but the *front* sensor looking 1.5 m down a
corridor spreads ±20 cm. Short walls let the beam pass over the top and read
as a false opening. Walls must also reach the floor.

### Wall surface
- Flat, smooth, rigid: foamboard, cardboard, thin plywood
- **Strictly perpendicular to the floor.** A tilted wall deflects the pulse
  away and reads as `NO_ECHO` → phantom opening
- No fabric, foam, or carpet on wall faces (absorbs the pulse)
- **No gaps between wall segments** — a 3 cm seam reads as an opening

### Junction geometry
- Openings at least as wide as the corridor (40 cm)
- Square corners, no diagonal cuts
- ≥ 80 cm of straight corridor between consecutive junctions
- **No 4-way intersections** (the classifier assumes this)

### Dead ends
Depth ≥ 2R + clearance ≈ 33 cm, so the 180° pivot fits. If you want tighter
dead ends, a reverse-then-turn routine is needed — not implemented yet.

### Maze exit
The exit must open into genuinely clear space on **all three** sides for at
least `EXIT_CONFIRM_CM` (25 cm) of travel. If your exit has a wall 40 cm
beyond it, the robot won't recognise it — extend the clear area.

---

## 4. Handling unreliable sonar (chassis rocking)

Three layers, all already implemented:

1. **Detection** — gyro X/Y axes are read in the same burst as Z. Pitch/roll
   above `ROCK_RATE_THRESHOLD` opens a `ROCK_BLANKING_MS` window during which
   readings are flagged low-confidence.
2. **Rejection** — `SONAR_MAX_JUMP_CM` discards physically impossible steps;
   median-of-3 absorbs single bad pings; stale readings expire.
3. **Graceful degradation** — while rocking, `Drive_Tick()` freezes the wall
   term and steers on gyro damping alone. The robot keeps moving straight
   instead of lurching at a phantom wall.

Tuning: if `rock=1` shows constantly in telemetry, raise `ROCK_RATE_THRESHOLD`.
If the robot lurches at bumps, lower it or raise `ROCK_BLANKING_MS`.

---

## 5. Gyro recalibration

Recalibration runs at every stop and after every pivot. Critically, it
**verifies stillness**: if the spread between the highest and lowest sample
exceeds `GYRO_CAL_MAX_SPREAD`, the chassis was moving and the result is
discarded — the previous offset is kept rather than corrupted.

Watch for `recal SKIPPED` in telemetry. Occasional is fine; constant means
either the chassis isn't settling (raise `GYRO_SETTLE_MS`) or the threshold is
too tight (raise `GYRO_CAL_MAX_SPREAD`).

Cost is ~450 ms per recalibration (150 samples × 2 ms + settle). Reduce
`GYRO_CAL_SAMPLES_QUICK` if that's too slow, at some cost in bias accuracy.

---

## 6. Known limitations

- **Approach offset is open-loop.** With no encoders, `APPROACH_TIME_MS` is
  `distance ÷ speed`. Battery sag changes actual speed, so it drifts as the
  pack drains. Expect the most tuning here. A fresh-battery calibration will
  over-shoot on a low battery.
- **Turns are blocking.** Sonar isn't serviced during a pivot (deliberate —
  readings are meaningless mid-rotation), but it also means no obstacle
  reaction during that window.
- **Random choice can revisit corridors.** Expected; a real solving algorithm
  replaces `choose()` in `maze.c` later.
- **Power stability.** Three sonars add draw on the same 5 V rail as the gyro
  and MCU. Given the connector faults found earlier, solder those joints
  before trusting Mode 3 — intermittent sonar looks exactly like a phantom
  opening, and the robot will turn into a wall.
