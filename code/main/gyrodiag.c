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
static void line(const char *s) {
    Debug_Str(s);
    Debug_NL();
    Debug_Flush();
}

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
        case 0: Debug_Str(" = +/-250 dps, 131 LSB per deg/sec");  break;
        case 1: Debug_Str(" = +/-500 dps, 65.5 LSB per deg/sec"); break;
        case 2: Debug_Str(" = +/-1000 dps, 32.8 LSB per deg/sec");break;
        default:Debug_Str(" = +/-2000 dps, 16.4 LSB per deg/sec");break;
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

    line("");
    line("[1] I2C bus lines -- both should idle HIGH via their pull-ups");

    // PC0 = SCL, PC1 = SDA. TWEN leaves them as open-drain with pull-ups, so a
    // line stuck LOW while idle means a short, a seized device, or no pull-up.
    scl = (PINC & (1 << PC0)) ? 1 : 0;
    sda = (PINC & (1 << PC1)) ? 1 : 0;

    Debug_Str("    SCL (PC0): ");
    Debug_Str(scl ? "HIGH  ok" : "LOW   *** FAULT ***");
    Debug_NL(); Debug_Flush();
    Debug_Str("    SDA (PC1): ");
    Debug_Str(sda ? "HIGH  ok" : "LOW   *** FAULT ***");
    Debug_NL(); Debug_Flush();

    if (scl && sda) {
        line("    -> bus is idle and pulled up correctly");
    } else {
        line("    -> a line held LOW while idle means a short to ground, a");
        line("       device holding the bus, or a missing pull-up. Nothing");
        line("       below will work until this is fixed.");
    }
}

// ---------------------------------------------------------------------------
//  Step 2: does the device identify itself?
// ---------------------------------------------------------------------------
static uint8_t check_identity(void) {
    uint8_t who = 0;

    line("");
    line("[2] Device identity -- WHO_AM_I (0x75) must read 0x68");

    if (!I2C_ReadRegs(MPU_ADDR_W, REG_WHOAMI, &who, 1)) {
        line("    read TIMED OUT -- the device did not answer at all");
        line("    -> NOT CONNECTED. Check 5V and GND first, then SDA/SCL.");
        line("       (this is also the fault that FREEZES the normal");
        line("       firmware, because its I2C waits are unbounded)");
        return 0;
    }

    Debug_Str("    read ");
    print_hex8(who);
    if (who == WHOAMI_VALUE) {
        Debug_Str("  PASS -- MPU6050 present and answering");
        Debug_NL(); Debug_Flush();
        return 1;
    }

    Debug_Str("  *** WRONG ***");
    Debug_NL(); Debug_Flush();
    if (who == 0x00) {
        line("    -> 0x00 means the bus answered but returned nothing. Usually");
        line("       SDA shorted to ground, or the device is unpowered while");
        line("       something else holds the line.");
    } else if (who == 0xFF) {
        line("    -> 0xFF means nothing drove the bus and the pull-up won.");
        line("       SDA is effectively disconnected.");
    } else {
        line("    -> some device answered, but it is not an MPU6050 at 0x68.");
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

    line("");
    line("[3] Configuration readback -- proves MPU6050_Init() took effect");

    if (!read_config(&pwr, &cfg)) {
        line("    readback TIMED OUT");
        *cfg_out = 0xFF;
        return;
    }

    Debug_Str("    PWR_MGMT_1  (0x6B) = ");
    print_hex8(pwr);
    Debug_Str((pwr & 0x40) ? "  *** ASLEEP ***" : "  awake  ok");
    Debug_NL(); Debug_Flush();

    Debug_Str("    GYRO_CONFIG (0x1B) = ");
    print_hex8(cfg);
    print_range(cfg);
    Debug_NL(); Debug_Flush();

    if (cfg == 0x08) {
        line("    -> matches what the firmware wrote. Scale is correct, so");
        line("       GYRO_LSB_MS_PER_DEGREE = 65500 is the right constant.");
    } else {
        line("    *** SCALE MISMATCH ***");
        line("    -> the firmware writes 0x08 (+/-500 dps) at init. Reading");
        line("       anything else means the device RESET ITSELF since then,");
        line("       reverting to its power-on default. That is a POWER fault,");
        line("       not a code fault -- and while it lasts every angle is");
        line("       scaled wrong, so turns land at the wrong physical angle");
        line("       while the log still reports 90 degrees.");
    }
    if (pwr & 0x40) {
        line("    -> ASLEEP also means it reset: init clears this bit. A");
        line("       sleeping gyro returns frozen values, not an error.");
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

    line("");
    line("[4] Read reliability + [5] noise floor");
    line("    KEEP THE ROBOT PERFECTLY STILL for this part.");

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

    Debug_Str("    attempted "); Debug_Int(GYRODIAG_READS);
    Debug_Str(", succeeded ");   Debug_Int(ok);
    Debug_Str(", TIMED OUT ");   Debug_Int(fail);
    Debug_NL(); Debug_Flush();
    Debug_Str("    all-zero ");  Debug_Int(zero);
    Debug_Str(", all-0xFF ");    Debug_Int(ones);
    Debug_Str(", frozen (identical to previous) "); Debug_Int(frozen);
    Debug_NL(); Debug_Flush();

    if (fail == 0 && zero == 0 && ones == 0 && frozen == 0) {
        line("    -> PASS, every read was live and plausible");
    } else {
        if (fail)   line("    *** timeouts: the link drops out intermittently.");
        if (zero)   line("    *** all-zero reads: SDA likely shorted low.");
        if (ones)   line("    *** all-0xFF reads: SDA likely open.");
        if (frozen) line("    *** frozen reads: the same bytes came back twice,");
        if (frozen) line("        so that value was stale, not measured. A live");
        if (frozen) line("        MPU6050 always jitters in its low bits.");
    }

    if (ok == 0) {
        line("    no successful reads -- skipping the noise floor");
        return;
    }

    line("");
    line("    noise floor at rest (raw LSB; 65.5 LSB = 1 deg/sec)");
    for (a = 0; a < 3; a++) {
        int32_t spread = (int32_t)vmax[a] - (int32_t)vmin[a];
        Debug_Str("      gyro ");
        Debug_Str(a == 0 ? "X" : (a == 1 ? "Y" : "Z"));
        Debug_KV(": mean", sum[a] / (int32_t)ok);
        Debug_KV("min", vmin[a]);
        Debug_KV("max", vmax[a]);
        Debug_KV("spread", spread);
        if (spread < GYRODIAG_NOISE_MIN)      Debug_Str(" *** TOO QUIET (frozen?)");
        else if (spread > GYRODIAG_NOISE_MAX) Debug_Str(" *** TOO NOISY (supply/wiring)");
        else                                  Debug_Str(" normal");
        Debug_NL(); Debug_Flush();
    }
    line("    (mean is the bias the firmware subtracts; Z's mean is the one");
    line("     that matters for heading. X/Y means only feed rock detection.)");
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

    line("");
    line("[6] LIVE MONITOR -- wiggle the gyro wires and connectors now.");
    line("    Only EVENTS print. Silence means the link is holding.");
    Debug_Str("    running for ");
    Debug_Int((int32_t)(GYRODIAG_MONITOR_MS / 1000));
    Debug_Str(" seconds...");
    Debug_NL(); Debug_Flush();

    while ((millis() - t0) < GYRODIAG_MONITOR_MS) {
        uint8_t k, same;

        if (!I2C_ReadRegs(MPU_ADDR_W, REG_GYRO_X, buf, 6)) {
            fails++;
            have_prev = 0;
            if (!was_failing) {
                was_failing = 1;
                Debug_Str("    t=");
                Debug_Int((int32_t)(millis() - t0));
                Debug_Str("ms  READ FAILED -- bus stopped answering");
                Debug_NL(); Debug_Flush();
            }
            Timer_WaitMs(20);
            continue;
        }
        if (was_failing) {
            was_failing = 0;
            Debug_Str("    t=");
            Debug_Int((int32_t)(millis() - t0));
            Debug_Str("ms  recovered -- reads answering again");
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
                Debug_Str("    t=");
                Debug_Int((int32_t)(millis() - t0));
                Debug_Str("ms  FROZEN -- identical bytes twice, value is stale");
                Debug_NL(); Debug_Flush();
            }
        } else if (was_frozen) {
            was_frozen = 0;
            Debug_Str("    t=");
            Debug_Int((int32_t)(millis() - t0));
            Debug_Str("ms  unfrozen -- values changing again");
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
                    Debug_Str("    t=");
                    Debug_Int((int32_t)(millis() - t0));
                    Debug_Str("ms  CONFIG CHANGED: GYRO_CONFIG now ");
                    print_hex8(cfg);
                    print_range(cfg);
                    Debug_NL(); Debug_Flush();
                    line("        *** THE GYRO RESET ITSELF -- POWER FAULT ***");
                    line("        Readings keep coming but at the wrong scale,");
                    line("        so every angle from here is wrong while the");
                    line("        log still looks healthy. This is the fault");
                    line("        that would make turns land short or long at");
                    line("        random. Fix the 5V rail and the connectors.");
                    cfg_at_start = cfg;   // report each change once
                }
            }
        }
        Timer_WaitMs(10);
    }

    line("");
    Debug_Str("    monitor finished: read failures ");
    Debug_Int(fails);
    Debug_Str(", config resets ");
    Debug_Int(cfg_changes);
    Debug_Str(", frozen samples ");
    Debug_Int(freezes);
    Debug_NL(); Debug_Flush();
    if (!fails && !cfg_changes && !freezes) {
        line("    -> the gyro link held up the whole time. If the robot still");
        line("       misbehaves, the gyro connection is not the cause.");
    } else {
        line("    -> the link is INTERMITTENT. Solder the joints, add bulk");
        line("       capacitance at the buck output, and re-run before");
        line("       chasing any control-loop tuning: no amount of it can");
        line("       compensate for a sensor that drops out.");
    }
}

// ---------------------------------------------------------------------------
void GyroDiag_Run(void) {
    uint8_t cfg = 0xFF;

    Motors_Stop();

    line("");
    line("========================================");
    line(" GYRO / I2C CONNECTION DIAGNOSTIC");
    line(" motors stay off for the whole test");
    line("========================================");

    check_bus_lines();

    if (check_identity()) {
        line("");
        line("    initialising the gyro (wake + set +/-500 dps)...");
        MPU6050_Init();
        check_config(&cfg);
        check_reads();
        monitor(cfg);
    } else {
        line("");
        line("    stopping here: nothing further can be tested until the");
        line("    device answers. Fix the wiring and re-run this mode.");
    }

    line("");
    line("GYRO DIAGNOSTIC DONE");
    Debug_Flush();
    for (;;) { }
}
