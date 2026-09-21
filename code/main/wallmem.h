#ifndef WALLMEM_H
#define WALLMEM_H
#include <stdint.h>

// Every literal this module returns for printing lives in FLASH on the AVR.
// String literals otherwise land in SRAM, and this part has 2048 bytes of it
// total -- see the warning at the top of debug.h. Callers on the AVR must
// print these with Debug_StrP(), not Debug_Str().
#ifdef __AVR__
#  include <avr/pgmspace.h>
#  define WM_TEXT(s) PSTR(s)
#else
#  define WM_TEXT(s) (s)
#endif

// ============================================================================
//  WALL-FOLLOWER MEMORY
//
//  Run 1 walks the maze with a strict left-hand rule and writes down one byte
//  per decision. Between runs those bytes are collapsed: every excursion into
//  a dead end is replaced by the single turn it is equivalent to. Run 2 replays
//  what is left, which on a maze with no loops is the shortest route.
//
//  The robot never learns WHERE IT IS. It has no encoders, so a cell index
//  derived from time x speed drifts and is never corrected. Everything here
//  depends only on two things the hardware does well: classify the junction it
//  is standing in (three sonars) and pivot 90/180 degrees (gyro). That is the
//  whole reason this is not a flood fill.
//
//  ---- THE RECORD BYTE -----------------------------------------------------
//
//      bit   7    6    5    4    3    2    1    0
//          [ 0 ][ F ][ L ][ R ][ 0 ][ 0 ][  TURN  ]
//                \___________/             \_____/
//                  signature                 what
//                 (what the                 we did
//                sonars saw)
//
//  Bits 7, 3 and 2 are always zero. That is not padding, it is a structural
//  check: erased EEPROM reads 0xFF, which fails it, so a blank or half-written
//  cell can never be mistaken for a real record. WallMem_RecordIsSane() is the
//  test, and load-time validation runs it over every byte.
//
//  The signature stores the RAW OPENNESS of forward/left/right, not the derived
//  junction enum. Same three bits either way, but the raw form is what run 2
//  compares against, and it cannot disagree with the classifier because it IS
//  the classifier's input.
//
//  ---- THE TURN CODE -------------------------------------------------------
//
//  Turns are stored as QUARTER TURNS CLOCKWISE, and this specific encoding is
//  what makes the collapse a one-liner:
//
//      WM_TURN_F = 0   (  0 degrees)
//      WM_TURN_R = 1   ( 90 degrees)
//      WM_TURN_U = 2   (180 degrees)
//      WM_TURN_L = 3   (270 degrees)
//
//  Going into a dead-end branch and coming back out is: turn A, travel, turn
//  180, travel back, turn B. The net rotation is A + 180 + B, so
//
//      net = (A + B + 2) & 3
//
//  replaces all three with one. No nine-case lookup table to mistype. See
//  WallMem_Reduce().
// ============================================================================

#define WM_TURN_F      0u
#define WM_TURN_R      1u
#define WM_TURN_U      2u
#define WM_TURN_L      3u
#define WM_TURN_MASK   0x03u

#define WM_OPEN_F      0x40u
#define WM_OPEN_L      0x20u
#define WM_OPEN_R      0x10u
#define WM_SIG_MASK    0x70u

// Bits that MUST be zero in a well-formed record.
#define WM_ZERO_MASK   0x8Cu

// ---- record field access ---------------------------------------------------
static inline uint8_t WallMem_Turn(uint8_t rec) { return (uint8_t)(rec & WM_TURN_MASK); }
static inline uint8_t WallMem_Sig (uint8_t rec) { return (uint8_t)(rec & WM_SIG_MASK); }
static inline uint8_t WallMem_RecordIsSane(uint8_t rec) { return (rec & WM_ZERO_MASK) ? 0u : 1u; }

// Pack the three sonar answers into the signature field.
static inline uint8_t WallMem_PackSig(uint8_t f_open, uint8_t l_open, uint8_t r_open) {
    return (uint8_t)((f_open ? WM_OPEN_F : 0u) |
                     (l_open ? WM_OPEN_L : 0u) |
                     (r_open ? WM_OPEN_R : 0u));
}

// ---- the two decisions, in one place --------------------------------------
//
// A cell is a DECISION POINT unless it is a plain corridor -- forward open,
// both sides walled. Corridor cells are driven through and record nothing.
//
// This predicate is used by BOTH runs. That is deliberate and it is the thing
// that keeps the string aligned: run 2 cannot consume a record at a cell where
// run 1 did not write one, because both ask the same question of the same
// three bits.
static inline uint8_t WallMem_IsDecision(uint8_t f_open, uint8_t l_open, uint8_t r_open) {
    return (f_open && !l_open && !r_open) ? 0u : 1u;
}

// The left-hand rule, in its entirety: LEFT > FORWARD > RIGHT > U-TURN.
static inline uint8_t WallMem_LeftHand(uint8_t f_open, uint8_t l_open, uint8_t r_open) {
    if (l_open) return WM_TURN_L;
    if (f_open) return WM_TURN_F;
    if (r_open) return WM_TURN_R;
    return WM_TURN_U;
}

// ---- run 1: log -----------------------------------------------------------
void    WallMem_Reset(void);
uint8_t WallMem_Record(uint8_t f_open, uint8_t l_open, uint8_t r_open, uint8_t turn);
uint8_t WallMem_Count(void);
uint8_t WallMem_At(uint8_t i);
uint8_t WallMem_Overflowed(void);

// ---- between runs: collapse ----------------------------------------------
// Returns the record count afterwards. *stray_u_out, if given, is set to the
// number of U-turns that could NOT be collapsed. On a maze with no loops that
// is zero; anything else means a junction was misread or the maze has a loop,
// and the result must not be driven.
uint8_t WallMem_Reduce(uint8_t *stray_u_out);

// ---- run 2: replay --------------------------------------------------------
typedef enum {
    WM_PLAY_OK = 0,        // record consumed, *turn_out is what to do
    WM_PLAY_MISMATCH,      // the junction does not look like the stored one
    WM_PLAY_EXHAUSTED      // ran off the end of the string
} wm_play_t;

void      WallMem_RewindPlayback(void);
uint8_t   WallMem_PlaybackIndex(void);
wm_play_t WallMem_Play(uint8_t f_open, uint8_t l_open, uint8_t r_open, uint8_t *turn_out);

// ---- persistence ----------------------------------------------------------
// The two runs are separate power-ups, so the collapsed string has to survive
// one. It lives in EEPROM behind a magic word, a length and a checksum; a
// brown-out part way through a write leaves a block that fails validation
// rather than one that drives the robot somewhere wrong.
uint8_t WallMem_Save(void);          // 1 = written and read back clean
uint8_t WallMem_Load(void);          // 1 = a valid solved route is now loaded
void    WallMem_Erase(void);
uint8_t WallMem_HaveSolution(void);  // 1 = the loaded string came from a solved run

// ---- reporting ------------------------------------------------------------
// Both return FLASH pointers on the AVR: print with Debug_StrP().
void        WallMem_Dump(void);
const char *WallMem_TurnName(uint8_t turn);
const char *WallMem_TypeName(uint8_t f_open, uint8_t l_open, uint8_t r_open);
#endif
