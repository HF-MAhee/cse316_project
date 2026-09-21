#include "config.h"
#include <string.h>
#include <stdint.h>
#include "wallmem.h"

#ifdef __AVR__
#  include <avr/eeprom.h>
#  include "debug.h"
#else
#  include <stdio.h>
#endif

// ============================================================================
//  The log.
//
//  One byte per decision. WALLMEM_MAX_RECORDS is a hard ceiling: an explore run
//  that needs more is reported as a failure rather than silently truncated,
//  because a truncated string replays into a wall.
// ============================================================================
static uint8_t s_log[WALLMEM_MAX_RECORDS];
static uint8_t s_count      = 0;
static uint8_t s_overflow   = 0;
static uint8_t s_play_index = 0;
static uint8_t s_solved     = 0;

void WallMem_Reset(void) {
    s_count = 0;
    s_overflow = 0;
    s_play_index = 0;
    s_solved = 0;
    memset(s_log, 0, sizeof(s_log));
}

uint8_t WallMem_Count(void)      { return s_count; }
uint8_t WallMem_Overflowed(void) { return s_overflow; }
uint8_t WallMem_HaveSolution(void) { return s_solved; }
uint8_t WallMem_At(uint8_t i)    { return (i < s_count) ? s_log[i] : 0xFFu; }

uint8_t WallMem_Record(uint8_t f_open, uint8_t l_open, uint8_t r_open, uint8_t turn) {
    if (s_count >= WALLMEM_MAX_RECORDS) { s_overflow = 1; return 0; }
    // Assembling the byte by OR-ing two masked fields is the only place a
    // record is built, so the invariant "bits 7,3,2 are zero" holds by
    // construction rather than by everyone remembering it.
    s_log[s_count++] = (uint8_t)(WallMem_PackSig(f_open, l_open, r_open) |
                                 (turn & WM_TURN_MASK));
    return 1;
}

// ============================================================================
//  Collapse.
//
//  Find a U-turn, fold it and its two neighbours into one record, repeat.
//
//  net = (A + B + 2) & 3  is the whole rule: quarter turns clockwise, plus the
//  180 in the middle. The surviving record keeps A's SIGNATURE, because A is
//  the first arrival at that junction and run 2 will arrive the same way.
//
//  Termination is not in doubt: every fold removes one U and shortens the log
//  by two, so the loop runs at most count/2 times.
// ============================================================================
uint8_t WallMem_Reduce(uint8_t *stray_u_out) {
    uint8_t i, folded, stray = 0;

    do {
        folded = 0;
        for (i = 1; (uint8_t)(i + 1) < s_count; i++) {
            if (WallMem_Turn(s_log[i]) == WM_TURN_U) {
                uint8_t a   = WallMem_Turn(s_log[i - 1]);
                uint8_t b   = WallMem_Turn(s_log[i + 1]);
                uint8_t net = (uint8_t)((a + b + 2u) & WM_TURN_MASK);

                s_log[i - 1] = (uint8_t)(WallMem_Sig(s_log[i - 1]) | net);
                memmove(&s_log[i], &s_log[i + 2],
                        (size_t)(s_count - (uint8_t)(i + 2)));
                s_count = (uint8_t)(s_count - 2);
                folded = 1;
                break;                      // indices moved; rescan
            }
        }
    } while (folded);

    // A U-turn that survives has no pair of neighbours to fold into: it sits at
    // one end of the string, or the maze has a loop so the walk never came back
    // the way it went in. Either way the remaining string is not a route.
    for (i = 0; i < s_count; i++) {
        if (WallMem_Turn(s_log[i]) == WM_TURN_U) stray++;
    }
    if (stray_u_out) *stray_u_out = stray;
    if (stray == 0) s_solved = 1;
    s_play_index = 0;
    return s_count;
}

// ============================================================================
//  Replay.
//
//  The signature check is the safety net. If a pivot overshot and the robot is
//  in the wrong corridor, the junction it is standing in will not match the one
//  that was stored -- and it says so BEFORE acting on a stale instruction,
//  which is the difference between a caught fault and a robot confidently
//  driving into a wall.
// ============================================================================
void    WallMem_RewindPlayback(void) { s_play_index = 0; }
uint8_t WallMem_PlaybackIndex(void)  { return s_play_index; }

wm_play_t WallMem_Play(uint8_t f_open, uint8_t l_open, uint8_t r_open,
                       uint8_t *turn_out) {
    uint8_t rec, seen;

    if (s_play_index >= s_count) return WM_PLAY_EXHAUSTED;

    rec  = s_log[s_play_index];
    seen = WallMem_PackSig(f_open, l_open, r_open);

    if (WallMem_Sig(rec) != seen) return WM_PLAY_MISMATCH;

    s_play_index++;
    if (turn_out) *turn_out = WallMem_Turn(rec);
    return WM_PLAY_OK;
}

// ============================================================================
//  EEPROM.
//
//  Layout at WALLMEM_EE_BASE:
//
//      +0  magic 'W'      written LAST
//      +1  magic 'M'
//      +2  format version
//      +3  record count
//      +4  flags   bit0 = the string is a collapsed, solved route
//      +5  CRC-8 over bytes +2..+4 and every record
//      +6  reserved
//      +7  reserved
//      +8  records
//
//  The magic byte is cleared first and written last. A brown-out part way
//  through -- which on this robot is a real event, not a theoretical one --
//  therefore leaves a block with no magic. That fails validation, so the next
//  power-up explores again instead of driving half a route.
// ============================================================================
#define WM_EE_MAGIC0   0x57u      /* 'W' */
#define WM_EE_MAGIC1   0x4Du      /* 'M' */
#define WM_EE_VERSION  1u
#define WM_EE_HDR      8u
#define WM_FLAG_SOLVED 0x01u

#ifdef __AVR__
static uint8_t ee_rd(uint16_t a)             { return eeprom_read_byte((uint8_t *)a); }
static void    ee_wr(uint16_t a, uint8_t v)  { eeprom_update_byte((uint8_t *)a, v); }
#else
/* Host builds (the unit test) keep the same code path over a RAM array, so the
   save/load logic under test is the same logic that runs on the robot. */
static uint8_t s_fake_ee[512];
static uint8_t ee_rd(uint16_t a)             { return (a < sizeof(s_fake_ee)) ? s_fake_ee[a] : 0xFFu; }
static void    ee_wr(uint16_t a, uint8_t v)  { if (a < sizeof(s_fake_ee)) s_fake_ee[a] = v; }
void WallMem_HostEraseEeprom(void)           { memset(s_fake_ee, 0xFF, sizeof(s_fake_ee)); }
void WallMem_HostPokeEeprom(uint16_t a, uint8_t v) { ee_wr(a, v); }
uint8_t WallMem_HostPeekEeprom(uint16_t a)   { return ee_rd(a); }
#endif

static uint8_t crc8(uint8_t crc, uint8_t v) {
    uint8_t i;
    crc ^= v;
    for (i = 0; i < 8; i++) {
        crc = (uint8_t)((crc & 0x80u) ? (uint8_t)(((unsigned)crc << 1) ^ 0x07u)
                                     : (uint8_t)((unsigned)crc << 1));
    }
    return crc;
}

static uint8_t compute_crc(uint8_t count, uint8_t flags, const uint8_t *recs) {
    uint8_t c = 0xFFu, i;
    c = crc8(c, WM_EE_VERSION);
    c = crc8(c, count);
    c = crc8(c, flags);
    for (i = 0; i < count; i++) c = crc8(c, recs[i]);
    return c;
}

void WallMem_Erase(void) {
    ee_wr(WALLMEM_EE_BASE + 0u, 0xFFu);
    ee_wr(WALLMEM_EE_BASE + 1u, 0xFFu);
}

uint8_t WallMem_Save(void) {
    uint8_t flags = (uint8_t)(s_solved ? WM_FLAG_SOLVED : 0u);
    uint8_t crc   = compute_crc(s_count, flags, s_log);
    uint8_t i;

    if (s_count > WALLMEM_MAX_RECORDS) return 0;

    ee_wr(WALLMEM_EE_BASE + 0u, 0xFFu);            // invalidate first
    for (i = 0; i < s_count; i++) ee_wr((uint16_t)(WALLMEM_EE_BASE + WM_EE_HDR + i), s_log[i]);
    ee_wr(WALLMEM_EE_BASE + 2u, WM_EE_VERSION);
    ee_wr(WALLMEM_EE_BASE + 3u, s_count);
    ee_wr(WALLMEM_EE_BASE + 4u, flags);
    ee_wr(WALLMEM_EE_BASE + 5u, crc);
    ee_wr(WALLMEM_EE_BASE + 6u, 0u);
    ee_wr(WALLMEM_EE_BASE + 7u, 0u);
    ee_wr(WALLMEM_EE_BASE + 1u, WM_EE_MAGIC1);
    ee_wr(WALLMEM_EE_BASE + 0u, WM_EE_MAGIC0);     // ...and commit last

    // Read it back. An EEPROM cell that will not take the write is worth
    // knowing about now, not at the start of run 2.
    for (i = 0; i < s_count; i++) {
        if (ee_rd((uint16_t)(WALLMEM_EE_BASE + WM_EE_HDR + i)) != s_log[i]) return 0;
    }
    return (uint8_t)(ee_rd(WALLMEM_EE_BASE + 5u) == crc);
}

uint8_t WallMem_Load(void) {
    uint8_t count, flags, crc, i;
    uint8_t tmp[WALLMEM_MAX_RECORDS];

    if (ee_rd(WALLMEM_EE_BASE + 0u) != WM_EE_MAGIC0) return 0;
    if (ee_rd(WALLMEM_EE_BASE + 1u) != WM_EE_MAGIC1) return 0;
    if (ee_rd(WALLMEM_EE_BASE + 2u) != WM_EE_VERSION) return 0;

    count = ee_rd(WALLMEM_EE_BASE + 3u);
    flags = ee_rd(WALLMEM_EE_BASE + 4u);
    crc   = ee_rd(WALLMEM_EE_BASE + 5u);
    if (count > WALLMEM_MAX_RECORDS) return 0;

    for (i = 0; i < count; i++) {
        tmp[i] = ee_rd((uint16_t)(WALLMEM_EE_BASE + WM_EE_HDR + i));
        // Structural check on every byte: bits 7, 3 and 2 must be clear. This
        // is what makes an erased or half-written cell unmistakable.
        if (!WallMem_RecordIsSane(tmp[i])) return 0;
    }
    if (compute_crc(count, flags, tmp) != crc) return 0;

    memcpy(s_log, tmp, count);
    s_count      = count;
    s_overflow   = 0;
    s_play_index = 0;
    s_solved     = (uint8_t)((flags & WM_FLAG_SOLVED) ? 1u : 0u);
    return 1;
}

// ============================================================================
//  Reporting
// ============================================================================
const char *WallMem_TurnName(uint8_t turn) {
    switch (turn & WM_TURN_MASK) {
        case WM_TURN_F: return WM_TEXT("FWD");
        case WM_TURN_R: return WM_TEXT("RIGHT");
        case WM_TURN_U: return WM_TEXT("180");
        default:        return WM_TEXT("LEFT");
    }
}

const char *WallMem_TypeName(uint8_t f, uint8_t l, uint8_t r) {
    if ( f &&  l &&  r) return WM_TEXT("ALL_OPEN");
    if ( f &&  l && !r) return WM_TEXT("FWD_OR_LEFT");
    if ( f && !l &&  r) return WM_TEXT("FWD_OR_RIGHT");
    if (!f &&  l &&  r) return WM_TEXT("T_LEFT_RIGHT");
    if ( f && !l && !r) return WM_TEXT("CORRIDOR");
    if (!f &&  l && !r) return WM_TEXT("FORCED_LEFT");
    if (!f && !l &&  r) return WM_TEXT("FORCED_RIGHT");
    return WM_TEXT("DEAD_END");
}

#ifdef __AVR__
void WallMem_Dump(void) {
    uint8_t i;
    Debug_P("  n=");
    Debug_Int((int32_t)s_count);
    Debug_P(" solved=");
    Debug_Int((int32_t)s_solved);
    Debug_P("\r\n");
    for (i = 0; i < s_count; i++) {
        uint8_t rec = s_log[i];
        uint8_t f = (uint8_t)((rec & WM_OPEN_F) ? 1u : 0u);
        uint8_t l = (uint8_t)((rec & WM_OPEN_L) ? 1u : 0u);
        uint8_t r = (uint8_t)((rec & WM_OPEN_R) ? 1u : 0u);
        Debug_P("  [");
        Debug_Int((int32_t)i);
        Debug_P("] 0x");
        // Two hex nibbles, so the raw byte can be checked against the layout
        // in wallmem.h by eye from the log.
        {
            const char *hex = "0123456789ABCDEF";
            char buf[3];
            buf[0] = hex[(rec >> 4) & 0x0Fu];
            buf[1] = hex[rec & 0x0Fu];
            buf[2] = 0;
            Debug_Str(buf);
        }
        Debug_P(" FLR=");
        Debug_Int((int32_t)f); Debug_Int((int32_t)l); Debug_Int((int32_t)r);
        Debug_P(" ");
        Debug_StrP(WallMem_TypeName(f, l, r));
        Debug_P(" -> ");
        Debug_StrP(WallMem_TurnName(WallMem_Turn(rec)));
        Debug_P("\r\n");
    }
    Debug_Flush();
}
#else
void WallMem_Dump(void) {
    uint8_t i;
    printf("  n=%u solved=%u\n", s_count, s_solved);
    for (i = 0; i < s_count; i++) {
        uint8_t rec = s_log[i];
        uint8_t f = (rec & WM_OPEN_F) ? 1u : 0u;
        uint8_t l = (rec & WM_OPEN_L) ? 1u : 0u;
        uint8_t r = (rec & WM_OPEN_R) ? 1u : 0u;
        printf("  [%2u] 0x%02X FLR=%u%u%u %-13s -> %s\n", i, rec, f, l, r,
               WallMem_TypeName(f, l, r), WallMem_TurnName(WallMem_Turn(rec)));
    }
}
#endif
