#include "config.h"
#include "heading.h"
#include "mpu6050.h"
#include "timer.h"

static int32_t  s_offset      = 0;   // Z bias, used for heading integration
static int32_t  s_offset_x    = 0;   // X bias, used only for rocking detection
static int32_t  s_offset_y    = 0;   // Y bias, used only for rocking detection
static int32_t  s_accum       = 0;   // LSB * ms
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

void Gyro_CalibrateFull(void) {
    uint8_t tries;
    for (tries = 0; tries < GYRO_CAL_RETRIES; tries++) {
        if (calibrate(GYRO_CAL_SAMPLES_INIT)) return;
        Timer_WaitMs(GYRO_SETTLE_MS);
    }
    // All attempts rejected: fall back to an unchecked average so the robot
    // still has *some* offsets rather than zero.
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
    s_accum += rate * (int32_t)dt_ms;
    s_last_z_rate = (int16_t)rate;
}

int32_t Heading_Raw(void)     { return s_accum; }
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
