#include "config.h"
#include <avr/io.h>
#include "gyrodiag.h"
#include "i2c.h"
#include "mpu6050.h"
#include "motors.h"
#include "timer.h"
#include "debug.h"

// MPU6050 registers. Defined here rather than shared from mpu6050.c so this
// diagnostic cannot disturb the tested driver; they are datasheet-fixed.
#define MPU_ADDR_W   0xD0
#define REG_WHOAMI   0x75
#define REG_PWR1     0x6B
#define REG_GYROCFG  0x1B
#define REG_GYRO_X   0x43
#define WHOAMI_VALUE 0x68

// Every line is flushed. This is a bench diagnostic with the motors off, so
// there is no control loop to protect, and a dropped byte in a report that
// exists to be read is worse than the wait.
//
// The text lives in FLASH. This file alone holds ~4 KB of it, against 2 KB of
// total SRAM -- as .data it could not possibly fit, and that is exactly why an
// earlier build of this mode printed nothing at all. Use LINE("..."), never a
// bare string.
static void line_p(const char *flash_str) {
    Debug_StrP(flash_str);
    Debug_NL();
    Debug_Flush();
}
#define LINE(s) line_p(PSTR(s))

static void print_hex8(uint8_t v) {
    static const char digits[] = "0123456789ABCDEF";
    char out[5];
    out[0] = '0';
    out[1] = 'x';
    out[2] = digits[(v >> 4) & 0x0F];
    out[3] = digits[v & 0x0F];
    out[4] = 0;
    Debug_Str(out);
}

// Decode GYRO_CONFIG's FS_SEL into the scale the firmware depends on. Getting
// this wrong silently scales every angle in the project.
static void print_range(uint8_t cfg) {
    switch ((cfg >> 3) & 0x03) {
        case 0: Debug_P(" = +/-250 dps, 131 LSB per deg/sec");  break;
        case 1: Debug_P(" = +/-500 dps, 65.5 LSB per deg/sec"); break;
        case 2: Debug_P(" = +/-1000 dps, 32.8 LSB per deg/sec");break;
        default:Debug_P(" = +/-2000 dps, 16.4 LSB per deg/sec");break;
    }
}

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

// ---------------------------------------------------------------------------
//  Step 1: are the bus lines even idle-high?
// ---------------------------------------------------------------------------
static void check_bus_lines(void) {
    uint8_t scl, sda;

    LINE("");
    LINE("[1] I2C bus lines -- both should idle HIGH via their pull-ups");

    // PC0 = SCL, PC1 = SDA. TWEN leaves them as open-drain with pull-ups, so a
    // line stuck LOW while idle means a short, a seized device, or no pull-up.
    scl = (PINC & (1 << PC0)) ? 1 : 0;
    sda = (PINC & (1 << PC1)) ? 1 : 0;

    Debug_P("    SCL (PC0): ");
    if (scl) Debug_P("HIGH  ok"); else Debug_P("LOW   *** FAULT ***");
    Debug_NL(); Debug_Flush();
    Debug_P("    SDA (PC1): ");
    if (sda) Debug_P("HIGH  ok"); else Debug_P("LOW   *** FAULT ***");
    Debug_NL(); Debug_Flush();

    if (scl && sda) {
        LINE("    -> bus is idle and pulled up correctly");
    } else {
        LINE("    -> a line held LOW while idle means a short to ground, a");
        LINE("       device holding the bus, or a missing pull-up. Nothing");
        LINE("       below will work until this is fixed.");
    }
}

// ---------------------------------------------------------------------------
//  Step 2: does the device identify itself?
// ---------------------------------------------------------------------------
static uint8_t check_identity(void) {
    uint8_t who = 0;

    LINE("");
    LINE("[2] Device identity -- WHO_AM_I (0x75) must read 0x68");

    if (!I2C_ReadRegs(MPU_ADDR_W, REG_WHOAMI, &who, 1)) {
        LINE("    read TIMED OUT -- the device did not answer at all");
        LINE("    -> NOT CONNECTED. Check 5V and GND first, then SDA/SCL.");
        LINE("       (this is also the fault that FREEZES the normal");
        LINE("       firmware, because its I2C waits are unbounded)");
        return 0;
    }

    Debug_P("    read ");
    print_hex8(who);
    if (who == WHOAMI_VALUE) {
        Debug_P("  PASS -- MPU6050 present and answering");
        Debug_NL(); Debug_Flush();
        return 1;
    }

    Debug_P("  *** WRONG ***");
    Debug_NL(); Debug_Flush();
    if (who == 0x00) {
        LINE("    -> 0x00 means the bus answered but returned nothing. Usually");
        LINE("       SDA shorted to ground, or the device is unpowered while");
        LINE("       something else holds the line.");
    } else if (who == 0xFF) {
        LINE("    -> 0xFF means nothing drove the bus and the pull-up won.");
        LINE("       SDA is effectively disconnected.");
    } else {
        LINE("    -> some device answered, but it is not an MPU6050 at 0x68.");
    }
    return 0;
}

// ---------------------------------------------------------------------------
//  Step 3: did our configuration actually stick?
// ---------------------------------------------------------------------------
static uint8_t read_config(uint8_t *pwr, uint8_t *cfg) {
    if (!I2C_ReadRegs(MPU_ADDR_W, REG_PWR1, pwr, 1))    return 0;
    if (!I2C_ReadRegs(MPU_ADDR_W, REG_GYROCFG, cfg, 1)) return 0;
    return 1;
}

static void check_config(uint8_t *cfg_out) {
    uint8_t pwr = 0xFF, cfg = 0xFF;

    LINE("");
    LINE("[3] Configuration readback -- proves MPU6050_Init() took effect");

    if (!read_config(&pwr, &cfg)) {
        LINE("    readback TIMED OUT");
        *cfg_out = 0xFF;
        return;
    }

    Debug_P("    PWR_MGMT_1  (0x6B) = ");
    print_hex8(pwr);
    if (pwr & 0x40) Debug_P("  *** ASLEEP ***"); else Debug_P("  awake  ok");
    Debug_NL(); Debug_Flush();

    Debug_P("    GYRO_CONFIG (0x1B) = ");
    print_hex8(cfg);
    print_range(cfg);
    Debug_NL(); Debug_Flush();

    if (cfg == 0x08) {
        LINE("    -> matches what the firmware wrote. Scale is correct, so");
        LINE("       GYRO_LSB_MS_PER_DEGREE = 65500 is the right constant.");
    } else {
        LINE("    *** SCALE MISMATCH ***");
        LINE("    -> the firmware writes 0x08 (+/-500 dps) at init. Reading");
        LINE("       anything else means the device RESET ITSELF since then,");
        LINE("       reverting to its power-on default. That is a POWER fault,");
        LINE("       not a code fault -- and while it lasts every angle is");
        LINE("       scaled wrong, so turns land at the wrong physical angle");
        LINE("       while the log still reports 90 degrees.");
    }
    if (pwr & 0x40) {
        LINE("    -> ASLEEP also means it reset: init clears this bit. A");
        LINE("       sleeping gyro returns frozen values, not an error.");
    }
    *cfg_out = cfg;
}

// ---------------------------------------------------------------------------
//  Step 4 + 5: reliability and noise floor, from one sample run
// ---------------------------------------------------------------------------
static void check_reads(void) {
    uint8_t  buf[6], prev[6];
    uint16_t i, ok = 0, fail = 0, zero = 0, ones = 0, frozen = 0;
    uint8_t  have_prev = 0;
    int32_t  sum[3]   = {0, 0, 0};
    int16_t  vmin[3]  = {32767, 32767, 32767};
    int16_t  vmax[3]  = {-32768, -32768, -32768};
    uint8_t  a;

    LINE("");
    LINE("[4] Read reliability + [5] noise floor");
    LINE("    KEEP THE ROBOT PERFECTLY STILL for this part.");

    for (i = 0; i < GYRODIAG_READS; i++) {
        if (!I2C_ReadRegs(MPU_ADDR_W, REG_GYRO_X, buf, 6)) {
            fail++;
            have_prev = 0;
            Timer_WaitMs(5);
            continue;
        }
        ok++;

        if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 &&
            buf[3] == 0 && buf[4] == 0 && buf[5] == 0) zero++;
        if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF &&
            buf[3] == 0xFF && buf[4] == 0xFF && buf[5] == 0xFF) ones++;

        if (have_prev) {
            uint8_t same = 1, k;
            for (k = 0; k < 6; k++) if (buf[k] != prev[k]) { same = 0; break; }
            if (same) frozen++;
        }
        for (a = 0; a < 6; a++) prev[a] = buf[a];
        have_prev = 1;

        for (a = 0; a < 3; a++) {
            int16_t v = be16(&buf[a * 2]);
            sum[a] += v;
            if (v < vmin[a]) vmin[a] = v;
            if (v > vmax[a]) vmax[a] = v;
        }
        Timer_WaitMs(5);
    }

    Debug_P("    attempted "); Debug_Int(GYRODIAG_READS);
    Debug_P(", succeeded ");   Debug_Int(ok);
    Debug_P(", TIMED OUT ");   Debug_Int(fail);
    Debug_NL(); Debug_Flush();
    Debug_P("    all-zero ");  Debug_Int(zero);
    Debug_P(", all-0xFF ");    Debug_Int(ones);
    Debug_P(", frozen (identical to previous) "); Debug_Int(frozen);
    Debug_NL(); Debug_Flush();

    if (fail == 0 && zero == 0 && ones == 0 && frozen == 0) {
        LINE("    -> PASS, every read was live and plausible");
    } else {
        if (fail)   LINE("    *** timeouts: the link drops out intermittently.");
        if (zero)   LINE("    *** all-zero reads: SDA likely shorted low.");
        if (ones)   LINE("    *** all-0xFF reads: SDA likely open.");
        if (frozen) LINE("    *** frozen reads: the same bytes came back twice,");
        if (frozen) LINE("        so that value was stale, not measured. A live");
        if (frozen) LINE("        MPU6050 always jitters in its low bits.");
    }

    if (ok == 0) {
        LINE("    no successful reads -- skipping the noise floor");
        return;
    }

    LINE("");
    LINE("    noise floor at rest (raw LSB; 65.5 LSB = 1 deg/sec)");
    for (a = 0; a < 3; a++) {
        int32_t spread = (int32_t)vmax[a] - (int32_t)vmin[a];
        Debug_P("      gyro ");
        if (a == 0)      Debug_P("X");
        else if (a == 1) Debug_P("Y");
        else             Debug_P("Z");
        Debug_KVF(": mean", sum[a] / (int32_t)ok);
        Debug_KVF("min", vmin[a]);
        Debug_KVF("max", vmax[a]);
        Debug_KVF("spread", spread);
        if (spread < GYRODIAG_NOISE_MIN)      Debug_P(" *** TOO QUIET (frozen?)");
        else if (spread > GYRODIAG_NOISE_MAX) Debug_P(" *** TOO NOISY (supply/wiring)");
        else                                  Debug_P(" normal");
        Debug_NL(); Debug_Flush();
    }
    LINE("    (mean is the bias the firmware subtracts; Z's mean is the one");
    LINE("     that matters for heading. X/Y means only feed rock detection.)");
}

// ---------------------------------------------------------------------------
//  Step 6: live monitor -- wiggle the wiring and watch for events
// ---------------------------------------------------------------------------
static void monitor(uint8_t cfg_at_start) {
    uint32_t t0 = millis();
    uint32_t next_check = millis();
    uint8_t  buf[6], prev[6];
    uint8_t  have_prev = 0;
    uint16_t fails = 0, cfg_changes = 0, freezes = 0;
    // Edge-triggered reporting. A bus that dies outright, or a sensor that
    // freezes, would otherwise print thousands of identical lines across the
    // monitor window and bury the one transition that mattered.
    uint8_t  was_failing = 0, was_frozen = 0;

    LINE("");
    LINE("[6] LIVE MONITOR -- wiggle the gyro wires and connectors now.");
    LINE("    Only EVENTS print. Silence means the link is holding.");
    Debug_P("    running for ");
    Debug_Int((int32_t)(GYRODIAG_MONITOR_MS / 1000));
    Debug_P(" seconds...");
    Debug_NL(); Debug_Flush();

    while ((millis() - t0) < GYRODIAG_MONITOR_MS) {
        uint8_t k, same;

        if (!I2C_ReadRegs(MPU_ADDR_W, REG_GYRO_X, buf, 6)) {
            fails++;
            have_prev = 0;
            if (!was_failing) {
                was_failing = 1;
                Debug_P("    t=");
                Debug_Int((int32_t)(millis() - t0));
                Debug_P("ms  READ FAILED -- bus stopped answering");
                Debug_NL(); Debug_Flush();
            }
            Timer_WaitMs(20);
            continue;
        }
        if (was_failing) {
            was_failing = 0;
            Debug_P("    t=");
            Debug_Int((int32_t)(millis() - t0));
            Debug_P("ms  recovered -- reads answering again");
            Debug_NL(); Debug_Flush();
        }

        same = have_prev ? 1 : 0;
        if (have_prev) {
            for (k = 0; k < 6; k++) if (buf[k] != prev[k]) { same = 0; break; }
        }
        if (same) {
            freezes++;
            if (!was_frozen) {
                was_frozen = 1;
                Debug_P("    t=");
                Debug_Int((int32_t)(millis() - t0));
                Debug_P("ms  FROZEN -- identical bytes twice, value is stale");
                Debug_NL(); Debug_Flush();
            }
        } else if (was_frozen) {
            was_frozen = 0;
            Debug_P("    t=");
            Debug_Int((int32_t)(millis() - t0));
            Debug_P("ms  unfrozen -- values changing again");
            Debug_NL(); Debug_Flush();
        }
        for (k = 0; k < 6; k++) prev[k] = buf[k];
        have_prev = 1;

        // Periodically re-verify the scale register. If the device browns out
        // and restarts, this is the only way to notice: the readings keep
        // arriving, just scaled by a different factor.
        if ((int32_t)(millis() - next_check) >= 0) {
            uint8_t pwr = 0, cfg = 0;
            next_check = millis() + GYRODIAG_CHECK_MS;
            if (read_config(&pwr, &cfg)) {
                if (cfg != cfg_at_start || (pwr & 0x40)) {
                    cfg_changes++;
                    Debug_P("    t=");
                    Debug_Int((int32_t)(millis() - t0));
                    Debug_P("ms  CONFIG CHANGED: GYRO_CONFIG now ");
                    print_hex8(cfg);
                    print_range(cfg);
                    Debug_NL(); Debug_Flush();
                    LINE("        *** THE GYRO RESET ITSELF -- POWER FAULT ***");
                    LINE("        Readings keep coming but at the wrong scale,");
                    LINE("        so every angle from here is wrong while the");
                    LINE("        log still looks healthy. This is the fault");
                    LINE("        that would make turns land short or long at");
                    LINE("        random. Fix the 5V rail and the connectors.");
                    cfg_at_start = cfg;   // report each change once
                }
            }
        }
        Timer_WaitMs(10);
    }

    LINE("");
    Debug_P("    monitor finished: read failures ");
    Debug_Int(fails);
    Debug_P(", config resets ");
    Debug_Int(cfg_changes);
    Debug_P(", frozen samples ");
    Debug_Int(freezes);
    Debug_NL(); Debug_Flush();
    if (!fails && !cfg_changes && !freezes) {
        LINE("    -> the gyro link held up the whole time. If the robot still");
        LINE("       misbehaves, the gyro connection is not the cause.");
    } else {
        LINE("    -> the link is INTERMITTENT. Solder the joints, add bulk");
        LINE("       capacitance at the buck output, and re-run before");
        LINE("       chasing any control-loop tuning: no amount of it can");
        LINE("       compensate for a sensor that drops out.");
    }
}

// ---------------------------------------------------------------------------
void GyroDiag_Run(void) {
    uint8_t cfg = 0xFF;

    Motors_Stop();

    LINE("");
    LINE("========================================");
    LINE(" GYRO / I2C CONNECTION DIAGNOSTIC");
    LINE(" motors stay off for the whole test");
    LINE("========================================");

    check_bus_lines();

    if (check_identity()) {
        LINE("");
        LINE("    initialising the gyro (wake + set +/-500 dps)...");
        MPU6050_Init();
        check_config(&cfg);
        check_reads();
        monitor(cfg);
    } else {
        LINE("");
        LINE("    stopping here: nothing further can be tested until the");
        LINE("    device answers. Fix the wiring and re-run this mode.");
    }

    LINE("");
    LINE("GYRO DIAGNOSTIC DONE");
    Debug_Flush();
    for (;;) { }
}
