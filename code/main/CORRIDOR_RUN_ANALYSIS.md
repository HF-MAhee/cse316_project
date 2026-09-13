# Straight Corridor Run — Full Behavioural Walkthrough

What decides what, at every moment from the start line to the stop, and what
the three logged runs actually showed.

---

## Part 1 — What the three logs proved

### Run 1 (`succ_init_center`) — the good case

Started near centre: `L=16, R=14`. Sensor-to-wall sum stayed 27–31 throughout.

| Observation | Reading |
|---|---|
| `err` range while driving | −2 to −6 |
| `corr` range | −5 to +6 |
| peak yaw | 1101 LSB ≈ 17 °/s |
| front stop | fired at F=30→24 |

Small error, small correction, low yaw. The front sonar stayed pointed down
the corridor and caught the obstacle cleanly. **This is what "working" looks
like.**

One thing to note even here: after the stop, F continued falling 24 → 7. That
is **17 cm of coast** with the motors off. `Drive_Stop()` has no active brake.

### Run 2 (`succ_with_a_bit_right`) — worked, but violently

Started at `R=8` — already inside `WALL_EMERGENCY_CM`, so the very first drive
tick fired `br=3` (EMERG_R) with a full −35 differential.

```
L: 20 20 21 18 16 16 15 12 12  9  5  4  3  4
R:  8  8  8 10 12 14 17 17 20 22 24 25 27 27
```

It crossed the corridor and hit `L=3` — the *other* wall — triggering EMERG_L
at +35 the other way. Peak yaw **4674 LSB ≈ 71 °/s**.

It recovered, so it reads as a success, but the mechanism was a full-authority
slam in one direction followed by a full-authority slam back. That is an
oscillation that happened to damp out, not control.

### Run 3 (`front_sensor_failed`) — the failure

The critical rows:

```
 L    F   R  err  wt  gt corr  pwmL pwmR   rate  yaw°/s
10   51  24   14  10  -1    9    84   66   -205    -3.1
10   60  24   14  21  -4   17    94   60   -597    -9.1
10   57  23   13  19  -9   10    85   65  -1164   -17.8
11   65  22   11  16  -9    7    82   68  -1102   -16.8
12   65  22   10   7 -11   -4    71   79  -1360   -20.8
```

**F went 51 → 60 → 57 → 65 → 65 while driving forward into an obstacle.** The
reported distance *increased* as the robot approached. Minimum F ever seen
while driving: **51 cm**. `FRONT_BLOCKED_CM` is 25. The stop condition was
never even close to true.

Your diagnosis was right. The robot started 7 cm left of centre (`L=10, R=24`),
commanded a right correction, and yawed steadily right at 3 → 21 °/s. The
rigidly-mounted front sonar swung with the chassis and walked off the obstacle,
reading down the corridor instead.

**Why the yaw ran away** — the P term overwhelmed the D term:

| rate | yaw °/s | wt (P) | gt (D) | net | |
|---|---|---|---|---|---|
| −205 | −3.1 | +21 | −2 | **+19** | still commanding right |
| −597 | −9.1 | +21 | −5 | **+16** | still commanding right |
| −1164 | −17.8 | +19 | −10 | **+9** | still commanding right |
| −1360 | −20.8 | +15 | −12 | **+3** | still commanding right |

At nearly 21 °/s of right rotation the controller was *still* adding right
steer. `WALL_KD_DEN = 120` made the damping term roughly half of what it needed
to be.

---

## Part 2 — Answering the transition question directly

> When the car moves right to correct itself, the sensors are tilted, much
> sonar data is invalid, and the front sensor might see the right wall as an
> obstacle. Are we handling this?

**Before these fixes: no, not adequately.** Here is each sub-case.

### Case A — side readings during a correction turn

A yawed chassis measures a *slant range*, not perpendicular distance. At yaw
angle θ, a wall at true perpendicular distance d reads `d / cos θ` — always
**longer** than reality.

| yaw | over-read |
|---|---|
| 10° | +1.5 % |
| 20° | +6.4 % |
| 30° | +15.5 % |

At the yaw rates in run 3, this is a 1–2 cm error on a 15 cm reading. Real but
not dangerous, and it self-corrects as the robot straightens. **This case was
already tolerable** — the median filter and the jump gate absorb it.

### Case B — the front beam walking off target

This is the one that bit you, and it was **not** handled. The front sonar is
rigid; yaw θ points it θ degrees off the corridor axis. The HC-SR04 cone is
about ±7.5°, so past roughly 8–10° of yaw an object dead ahead falls outside
the beam entirely. At 18–21° it is long gone.

The old rule was "2 *consecutive* pings under threshold". During a sustained
correction turn there are no such pings — the count never even starts.

### Case C — front sonar seeing a side wall

The mirror hazard you raised. Yawed far enough, the front beam intersects the
side wall ahead of the robot and returns a genuine short reading — a phantom
obstacle where the corridor is actually clear.

Geometry: with the robot 15 cm from the right wall, the front beam strikes it
at range `15 / sin θ`:

| yaw | range to side wall |
|---|---|
| 15° | 58 cm |
| 25° | 36 cm |
| 35° | 26 cm |

So below about 30° of yaw the side wall is farther than `FRONT_BLOCKED_CM`
(25 cm) and cannot trigger a false stop. **The yaw governor caps rotation at
20 °/s**, which keeps the geometry well outside this regime. This case is now
structurally prevented rather than filtered.

### Case D — what happens to the gyro during all of this

Worth separating, because the gyro behaves differently from the sonars:

- **Yaw (Z)** stays valid throughout. It is *measuring* the turn, not being
  confused by it. This is why gyro-only fallback is a safe degradation.
- **Pitch/roll (X, Y)** spike from vibration. Measured: roll peaks near 2900
  LSB (≈44 °/s), pitch under 700. This drives the `rock` flag.
- **Bias drift** is not affected by a correction turn — it is thermal, which is
  why recalibration happens at stops, not during driving.

The important design consequence: **rocking no longer invalidates sonar data**
(that change was made earlier). It only reduces the wall-term gain. Letting it
invalidate readings was what previously forced `md=3` and silently disabled
centring.

---

## Part 3 — The full run, step by step

### Stage 0 — Power-on to start line

1. `Debug_Init()`, `Timer_Init()` (before `sei()`), `I2C_Init()`,
   `Motors_Init()` (Timer1 PWM), `Sonar_Init()`, then `sei()`.
2. `report_reset_cause()` decodes MCUCSR. `power-on BROWNOUT` together on a
   cold boot is normal — the rail ramps through the brown-out threshold.
   `BROWNOUT` *alone* on a restart is a real supply fault.
3. `MPU6050_Init()` — wake, set ±500 dps (this fixes the 65.5 LSB/dps scale).
4. `Gyro_CalibrateFull()` — 500 samples, all three axes, with a stillness
   check. If the Z spread exceeds `GYRO_CAL_MAX_SPREAD` the result is
   discarded and the old offset kept, because averaging a moving chassis
   produces a meaningless bias.
5. `STARTUP_DELAY_MS` (3 s) so you can step away.

**Decides:** `offZ` (heading accuracy), `offX`/`offY` (rocking threshold
reference).

### Stage 1 — Launch

`Drive_Begin()` fires `KICK_PWM` (120) on both wheels for `KICK_MS` (40 ms).
Necessary because the motors will not start from rest at cruise PWM.

Deliberately *not* gyro-tracked: both wheels forward together barely yaws the
chassis, unlike a pivot where the kick is the manoeuvre itself.

### Stage 2 — The control tick (every 20 ms)

Fixed order, and the order matters:

```
1. MPU6050_ReadAll()      one burst, all 3 axes
2. Motion_Update()        pitch/roll -> rocking flag
3. Heading_Add()          integrate yaw in LSB*ms
4. Sonar_Task()           exactly ONE ping, round-robin L->F->R
5. Drive_Tick(rate)       the controller
6. telemetry()            non-blocking, drops rather than stalls
```

One ping per tick means each sensor refreshes every 60 ms, which also
guarantees >20 ms of acoustic settling between pings — the crosstalk
requirement is satisfied for free.

### Stage 3 — Sonar processing

Per ping, in order:

1. **Range gate.** `< SONAR_MIN_VALID_CM` → `TOO_CLOSE`; `> SONAR_MAX_RANGE_CM`
   → `NO_ECHO`. These must stay distinct — aliasing them made a wall about to
   be hit report as open space.
2. **Lost-echo disambiguation.** A timeout when the last good reading was
   ≤ `SONAR_NEAR_LATCH_CM` (12 cm) is reclassified as `TOO_CLOSE`. Walls do not
   vanish in 60 ms; below 3 cm the echo returns during transmit and produces a
   genuine timeout.
3. **Jump gate.** A step larger than `SONAR_MAX_JUMP_CM` marks the sample
   low-confidence.
4. **Median-of-3** for steering (smooth); **raw latest** for detection (fast).
5. **Front vote recorded** — see Stage 5.

### Stage 4 — Centring controller

**Mode selection** (recomputed every tick):

| Condition | Mode | Error used |
|---|---|---|
| both valid | 0 BOTH | `R − L` — *width-independent* |
| left only | 1 LEFT | `CORRIDOR_HALF_CM − L` |
| right only | 2 RIGHT | `R − CORRIDOR_HALF_CM` |
| neither | 3 GYRO | 0 — heading hold only |

Mode 0 is the important one: the differential error is zero when centred
*regardless of corridor width*. Only modes 1 and 2 depend on
`CORRIDOR_HALF_CM` being correct.

**Control law**, in the order the terms are applied:

```
1. emergency check        (proximity ramp, bypasses everything below)
2. wt = KP * err          proportional, from wall positions
3. if rocking: wt /= 2    reduced gain, NOT frozen
4. gt = KD * yaw_rate     damping, opposes rotation
5. corr = wt + gt
6. YAW GOVERNOR           refuse to add yaw past 20 deg/s
7. clamp to +-WALL_MAX_CORRECTION
8. pwmL = base + corr, pwmR = base - corr
9. floor preservation     lift both rather than clipping one
```

Sign convention throughout: **positive `corr` steers right; positive gyro Z is
turning left.**

### Stage 5 — Front obstacle detection

Voting, not consecutive hits: an obstacle in **any 2 of the last 5** front
pings counts as real.

The reasoning is specific to the failure mode. A real obstacle intermittently
seen during a correction turn produces a scattered pattern like
`[22, 60, 58, 23, 65]` — two hits, non-adjacent. The old "2 consecutive" rule
scores that as zero. Voting scores it as blocked.

**Important honesty about scope:** voting alone would *not* have saved run 3.
There, F was `[51, 60, 57, 65, 65]` — **zero** pings under threshold, because
the beam never intersected the obstacle at all. Voting handles partial misses;
only the yaw governor prevents total ones.

### Stage 6 — Stop

`Drive_Stop()` cuts PWM and clears the diagnostic snapshot (otherwise
telemetry keeps reporting live PWM values on a stationary robot).

**Known gap:** there is no active brake. Run 1 coasted 17 cm after the stop
fired. For Mode 1 that is harmless; for Mode 3's approach-offset positioning it
is a real error source.

---

## Part 4 — What changed, and why

| Change | From | To | Reason |
|---|---|---|---|
| `WALL_KD_DEN` | 120 | **65** | P overwhelmed D; net stayed positive at 21 °/s of right yaw |
| Front detection | 2 consecutive | **2 of last 5** | consecutive can never fire during a sustained yaw |
| Yaw governor | — | **20 °/s cap** | keeps the front beam on target; prevents Case C geometry |
| Emergency | hard ±max | **ramped by proximity** | full slam at 8 cm caused 71 °/s and wall-to-wall bouncing |
| `WALL_MAX_CORRECTION` | fixed 35 | **37 % of base** | steering authority is `corr/base`; lowering base alone made it *more* violent |
| `DRIVE_BASE_PWM` | 75 | **60** | your measurement |
| `MOTOR_MIN/MAX` | 60/160 | **45/140** | your measurement |

### On your speed change

Lowering the base speed was correct, and the reason is worth stating: steering
authority is the **ratio** `corr / base`, not the absolute correction. At base
75 a ±35 correction is a 47 % split. Dropping base to 60 while keeping ±35
raises that to **58 % — sharper steering, not gentler.**

The improvement you observed came from the lower speed (less ground covered per
degree of yaw), partly *despite* the ratio moving the wrong way. Deriving
`WALL_MAX_CORRECTION` from the base holds the ratio at 37 % so the two now move
together.

Replaying run 3 with all fixes:

| rate | yaw °/s | wt | gt old | gt new | net new | |
|---|---|---|---|---|---|---|
| −205 | −3.1 | +21 | −2 | −4 | +17 | |
| −597 | −9.1 | +21 | −5 | −10 | +11 | |
| −1164 | −17.8 | +19 | −10 | −18 | **+1** | governor arms |
| −1360 | −20.8 | +15 | −12 | −21 | **0** | **governor engaged** |

---

## Part 5 — Two things the logs revealed that are not yet fixed

### 1. Corridor width mismatch

`L + R` measures **27–35 cm** across all three runs. With
`CORRIDOR_WIDTH_CM = 30` and a 16 cm robot, the expected sum is about 14.

The target distance `CORRIDOR_HALF_CM = 15` happens to match the observed ~15 cm
when centred, so modes 1 and 2 behave correctly by luck. But:

```
OPENING_THRESHOLD_CM = CORRIDOR_HALF_CM + 10 = 25
```

and run 3 shows a perfectly normal `R = 24` during ordinary off-centre driving.
**One centimetre from being classified as an opening.** In Mode 1 that is
harmless. In Mode 3 it means a junction where there is none — and a turn into a
wall.

Measure the real corridor before enabling Mode 3, and widen that margin.

### 2. No active braking

17 cm of coast in run 1. `turn.c` already has a reverse-pulse brake; `drive.c`
does not. This matters for Mode 3's approach positioning, where stopping
distance directly sets whether the pivot centre lands in the opening.

---

## Part 6 — What to watch on the next run

New telemetry column `fv` (front votes, 0–5) sits between `near` and `md`.

| Check | Expect |
|---|---|
| `fv` | climbs to ≥2 before the stop; a lingering 1 is a near-miss worth investigating |
| peak `rate` | should now stay near ±1300 (20 °/s), not 4674 |
| `br` | brief 2/3 (emergency) only near walls, never sustained |
| `gt` vs `wt` | at high yaw they should now be comparable, not 2:1 |
| `md` | mostly 0; frequent 3 means sonar is dropping out |
| `ovr` | must stay 0 |

Run Mode 1 with a hand on the power switch, starting off-centre deliberately.
The yaw governor is the fix that matters most and it has only been verified
against replayed log data, never on hardware.
