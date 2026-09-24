#include "config.h"
#include "heading.h"
#include "mpu6050.h"
#include "timer.h"

static int32_t  s_offset      = 0;   // Z bias, used for heading integration
static int32_t  s_offset_x    = 0;   // X bias, used only for rocking detection
static int32_t  s_offset_y    = 0;   // Y bias, used only for rocking detection
static int32_t  s_accum       = 0;   // LSB * ms
static int32_t  s_abs         = 0;   // LSB * ms, since the run started
static int32_t  s_grid        = 0;   // LSB * ms, the grid heading to be on
static uint32_t s_last_ms     = 0;   // when the heading was last integrated

// Learned gyro scale correction, parts per 10000 (see Heading_GridStep).
static int16_t  s_scale_c     = 0;
static int32_t  s_rot_sum     = 0;   // grid turned since the last estimate, LSB*ms
static int32_t  s_adj_sum     = 0;   // wall corrections over the same span
static uint32_t s_rock_until  = 0;
static int16_t  s_last_z_rate = 0;

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

// ---------------------------------------------------------------------------
//  Calibration
// ---------------------------------------------------------------------------
// Averages N stationary samples to find each axis's zero-rate bias, and
// checks the Z spread specifically (Z is what heading integration depends
// on for accuracy). A large spread means the chassis was NOT still, so the
// average is meaningless -- the old offsets are kept rather than corrupted.
//
// X and Y are calibrated here too, even though only Z is used for heading.
// Motion_Update() compares raw X/Y against ROCK_RATE_THRESHOLD to detect
// chassis rocking -- and cheap MPU6050 boards commonly have a static bias
// of several hundred LSB on X or Y even at rest. Left uncorrected, that
// bias alone can sit permanently above the threshold, making the robot
// look "constantly rocking" and freezing Drive_Tick() in its gyro-only
// fallback (mode 3) even on a perfectly still table.
static uint8_t calibrate(uint16_t samples) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t zmin = 32767, zmax = -32768;
    uint16_t i;
    gyro_xyz_t g;

    for (i = 0; i < samples; i++) {
        MPU6050_ReadAll(&g);
        sum_x += g.x;
        sum_y += g.y;
        sum_z += g.z;
        if (g.z < zmin) zmin = g.z;
        if (g.z > zmax) zmax = g.z;
        Timer_WaitMs(GYRO_CAL_INTERVAL_MS);
    }

    if ((int32_t)zmax - (int32_t)zmin > GYRO_CAL_MAX_SPREAD) {
        return 0;                       // chassis was moving -- reject
    }

    s_offset   = (sum_z / (int32_t)samples) + GYRO_OFFSET_TRIM;
    s_offset_x = sum_x / (int32_t)samples;
    s_offset_y = sum_y / (int32_t)samples;
    return 1;
}

uint8_t Gyro_CalibrateFull(void) {
    uint8_t tries;
    for (tries = 0; tries < GYRO_CAL_RETRIES; tries++) {
        if (calibrate(GYRO_CAL_SAMPLES_INIT)) return 1;
        Timer_WaitMs(GYRO_SETTLE_MS);
    }
    // All attempts rejected: fall back to an unchecked average so the robot
    // still has *some* offsets rather than zero.
    //
    // THE RETURN VALUE MATTERS. This used to be void, so this fallback was
    // completely silent: every rejection means the chassis would not hold still,
    // and the bias captured here is therefore measured DURING MOTION and wrong
    // for the whole run. Every heading, every turn and every straight-line
    // correction afterwards is built on it. The caller must be able to say so.
    {
        int32_t sum_x = 0, sum_y = 0, sum_z = 0;
        uint16_t i;
        gyro_xyz_t g;
        for (i = 0; i < GYRO_CAL_SAMPLES_INIT; i++) {
            MPU6050_ReadAll(&g);
            sum_x += g.x; sum_y += g.y; sum_z += g.z;
            Timer_WaitMs(GYRO_CAL_INTERVAL_MS);
        }
        s_offset   = (sum_z / GYRO_CAL_SAMPLES_INIT) + GYRO_OFFSET_TRIM;
        s_offset_x = sum_x / GYRO_CAL_SAMPLES_INIT;
        s_offset_y = sum_y / GYRO_CAL_SAMPLES_INIT;
    }
    return 0;                   // offsets exist, but none of them are trusted
}

uint8_t Gyro_CalibrateQuick(void) {
    uint8_t tries;
    // Let the chassis stop rocking before sampling, otherwise the spread
    // check will just reject everything.
    Timer_WaitMs(GYRO_SETTLE_MS);
    for (tries = 0; tries < GYRO_CAL_RETRIES; tries++) {
        if (calibrate(GYRO_CAL_SAMPLES_QUICK)) return 1;
        Timer_WaitMs(GYRO_SETTLE_MS);
    }
    return 0;                           // keep previous offset
}

int32_t Gyro_GetOffset(void)  { return s_offset; }
int32_t Gyro_GetOffsetX(void) { return s_offset_x; }
int32_t Gyro_GetOffsetY(void) { return s_offset_y; }

// ---------------------------------------------------------------------------
//  Heading accumulation
// ---------------------------------------------------------------------------
void Heading_Reset(void) { s_accum = 0; }

int16_t Gyro_Rate(int16_t raw_z) {
    return (int16_t)((int32_t)raw_z - s_offset);
}

void Heading_Add(int16_t raw_z, uint16_t dt_ms) {
    int32_t rate = (int32_t)raw_z - s_offset;
    rate += (rate * s_scale_c) / 10000L;
    s_accum += rate * (int32_t)dt_ms;
    s_abs   += rate * (int32_t)dt_ms;
    s_last_z_rate = (int16_t)rate;
    s_last_ms = millis();
}

void Heading_AddNow(int16_t raw_z) {
    uint32_t now = millis();
    uint32_t dt  = now - s_last_ms;
    // A gap longer than this means nobody was sampling -- a calibration, the
    // robot sitting armed. The chassis was still, so charging the whole gap
    // at the current rate would only integrate noise.
    if (dt > HEADING_MAX_DT_MS) dt = HEADING_MAX_DT_MS;
    Heading_Add(raw_z, (uint16_t)dt);
    s_last_ms = now;
}

int32_t Heading_Raw(void)     { return s_accum; }

void Heading_GridReset(void) { s_abs = 0; s_grid = 0; s_rot_sum = 0; s_adj_sum = 0; }

// Every turn is also a measurement of the gyro's scale. If the gyro reads 3%
// high, a "90" is really 87, and the wall alignment then has to move the grid
// by 3% of the rotation to put it back on the corridors -- so the ratio of
// wall corrections to rotation, over enough turning to see past the noise, IS
// the scale error. Half of it is applied each time, so a noisy estimate
// cannot throw the scale far, and it is capped at HEADING_SCALE_MAX_PPT.
// Estimated before the grid moves, so the corrections for the rotation just
// finished are (mostly) already in.
void Heading_GridStep(int8_t quarters) {
    const int32_t window = (int32_t)HEADING_SCALE_WINDOW_DEG * GYRO_LSB_MS_PER_DEGREE;
    if (s_rot_sum >= window || s_rot_sum <= -window) {
        // e = corrections / rotation, in parts per 10000; rotation in units of
        // 1/10000 of itself keeps this inside 32 bits.
        int32_t e = s_adj_sum / (s_rot_sum / 10000L);
        int32_t c = (int32_t)s_scale_c - e / 2;
        if (c >  HEADING_SCALE_MAX_PPT * 10L) c =  HEADING_SCALE_MAX_PPT * 10L;
        if (c < -HEADING_SCALE_MAX_PPT * 10L) c = -HEADING_SCALE_MAX_PPT * 10L;
        s_scale_c = (int16_t)c;
        s_rot_sum = 0;
        s_adj_sum = 0;
    }
    s_grid    += (int32_t)quarters * 90L * GYRO_LSB_MS_PER_DEGREE;
    s_rot_sum += (int32_t)quarters * 90L * GYRO_LSB_MS_PER_DEGREE;
}
int16_t Heading_ScaleCorrection(void) { return s_scale_c; }
void Heading_GridResync(void) { s_grid = s_abs; }
void Heading_GridAdjust(int32_t raw) { s_grid += raw; s_adj_sum += raw; }
int32_t Heading_GridError(void) { return s_abs - s_grid; }
int32_t Heading_GridErrorTenths(void) {
    return ((s_abs - s_grid) * 10L) / GYRO_LSB_MS_PER_DEGREE;
}
int32_t Heading_Degrees(void) { return s_accum / GYRO_LSB_MS_PER_DEGREE; }

int32_t Heading_DegreesTenths(void) {
    return (s_accum * 10L) / GYRO_LSB_MS_PER_DEGREE;
}

// ---------------------------------------------------------------------------
//  Rocking detection  (your point #5)
// ---------------------------------------------------------------------------
// The side sonars are mounted perpendicular to the walls. If the chassis
// pitches or rolls, that beam tilts: it can strike the floor (impossibly short
// reading) or pass above the wall (NO_ECHO). Rather than try to correct the
// geometry, we simply detect the rocking and mark the affected readings
// low-confidence, then coast on the last good correction.
void Motion_Update(const gyro_xyz_t *g) {
    int32_t px = abs32((int32_t)g->x - s_offset_x);
    int32_t py = abs32((int32_t)g->y - s_offset_y);

    if (px > ROCK_RATE_THRESHOLD || py > ROCK_RATE_THRESHOLD) {
        s_rock_until = millis() + ROCK_BLANKING_MS;
    }
}

uint8_t Motion_IsSuspect(void) {
    // Signed compare handles the case where the deadline has already passed.
    return ((int32_t)(millis() - s_rock_until) < 0) ? 1 : 0;
}

uint8_t Motion_IsStill(void) {
    if (Motion_IsSuspect()) return 0;
    return (abs32(s_last_z_rate) < (ROCK_RATE_THRESHOLD / 4)) ? 1 : 0;
}
