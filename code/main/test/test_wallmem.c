// ============================================================================
//  Host-side test for wallmem.c.
//
//  This compiles the REAL wallmem.c -- the same record packing, the same
//  collapse, the same EEPROM save/load, the same replay -- against a model of
//  the maze the firmware is meant to solve. It is the only way to prove the
//  logic without a robot and a room full of cardboard.
//
//      cc -I.. -o /tmp/twm test/test_wallmem.c wallmem.c && /tmp/twm
//
//  The maze (1 cell = 40 cm, 200 x 120 cm, 21 wall faces = 840 cm):
//
//      +    +----+----+----+  ^ +      ^ exit gap  (north of E2)
//           | B2   C2   D2   E2 |      v entry gap (south of E0)
//      +----+----+    +----+----+
//      | A1   B1 | C1   D1   E1 |
//      +    +    +    +    +    +
//      | A0 | B0   C0 | D0 | E0 |
//      +----+----+----+----+  v +
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "config.h"
#include "wallmem.h"

void    WallMem_HostEraseEeprom(void);
void    WallMem_HostPokeEeprom(uint16_t a, uint8_t v);
uint8_t WallMem_HostPeekEeprom(uint16_t a);

// ---- direction encoding ---------------------------------------------------
// N,E,S,W in clockwise order, which is what makes the turn code double as the
// heading delta: head = (head + turn) & 3. WM_TURN_R is +1 and that IS one
// clockwise quarter turn. The firmware relies on exactly this relationship.
enum { N = 0, E = 1, S = 2, W = 3 };
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

#define COLS 5
#define ROWS 3
static int floor_cell[COLS][ROWS];

// wall_v[c][r] = wall between (c-1,r) and (c,r);  wall_h[c][r] = between (c,r-1) and (c,r)
static int wall_v[COLS + 1][ROWS];
static int wall_h[COLS][ROWS + 1];

static int gap_start_c = 4, gap_start_r = 0;   // E0, south side
static int gap_exit_c  = 4, gap_exit_r  = 2;   // E2, north side

static int in_floor(int c, int r) {
    return (c >= 0 && c < COLS && r >= 0 && r < ROWS) ? floor_cell[c][r] : 0;
}

static int is_open(int c, int r, int dir) {
    int nc = c + DX[dir], nr = r + DY[dir];
    if (in_floor(nc, nr)) {
        if (dir == N) return !wall_h[c][r + 1];
        if (dir == S) return !wall_h[c][r];
        if (dir == E) return !wall_v[c + 1][r];
        return !wall_v[c][r];
    }
    if (dir == S && c == gap_start_c && r == gap_start_r) return 1;
    if (dir == N && c == gap_exit_c  && r == gap_exit_r)  return 1;
    return 0;
}

static void build_maze(void) {
    int c, r;
    memset(floor_cell, 0, sizeof(floor_cell));
    memset(wall_v, 0, sizeof(wall_v));
    memset(wall_h, 0, sizeof(wall_h));
    for (c = 0; c < COLS; c++) for (r = 0; r < ROWS; r++) floor_cell[c][r] = 1;
    floor_cell[0][2] = 0;                       // A2 is not part of the maze

    // the seven interior walls
    wall_v[1][0] = 1;   // A0 | B0
    wall_h[1][2] = 1;   // B1 | B2
    wall_v[2][1] = 1;   // B1 | C1
    wall_v[3][0] = 1;   // C0 | D0
    wall_v[4][0] = 1;   // D0 | E0
    wall_h[3][2] = 1;   // D1 | D2
    wall_h[4][2] = 1;   // E1 | E2
    // perimeter
    for (c = 0; c < COLS; c++) {
        if (in_floor(c, 0))        wall_h[c][0] = 1;
        if (in_floor(c, ROWS - 1)) wall_h[c][ROWS] = 1;
        if (!in_floor(c, 2) && in_floor(c, 1)) wall_h[c][2] = 1;
    }
    for (r = 0; r < ROWS; r++) {
        if (in_floor(0, r)) wall_v[0][r] = 1;
        if (in_floor(COLS - 1, r)) wall_v[COLS][r] = 1;
    }
    wall_v[1][2] = 1;   // B2's west face, where A2 is missing
}

static const char *cellname(int c, int r) {
    static char b[4];
    b[0] = (char)('A' + c); b[1] = (char)('0' + r); b[2] = 0;
    return b;
}
static char turn_ch(uint8_t t) { return "FRUL"[t & 3]; }

// ---- the walk -------------------------------------------------------------
// One routine for both runs: only the decision differs, exactly as in the
// firmware's decide_turn().
static int walk(int explore, char *turns_out, char *path_out, int *legs_out) {
    int c = gap_start_c, r = gap_start_r, head = N, steps = 0, nt = 0;
    path_out[0] = 0;
    strcat(path_out, cellname(c, r));

    for (steps = 0; steps < 200; steps++) {
        uint8_t f = (uint8_t)is_open(c, r, head);
        uint8_t l = (uint8_t)is_open(c, r, (head + 3) & 3);
        uint8_t rr = (uint8_t)is_open(c, r, (head + 1) & 3);
        uint8_t turn;

        if (WallMem_IsDecision(f, l, rr)) {
            if (explore) {
                turn = WallMem_LeftHand(f, l, rr);
                if (!WallMem_Record(f, l, rr, turn)) { printf("  LOG FULL\n"); return -1; }
            } else {
                wm_play_t p = WallMem_Play(f, l, rr, &turn);
                if (p == WM_PLAY_MISMATCH)  { printf("  MISMATCH at %s\n", cellname(c, r)); return -1; }
                if (p == WM_PLAY_EXHAUSTED) { printf("  EXHAUSTED at %s\n", cellname(c, r)); return -1; }
            }
            turns_out[nt++] = turn_ch(turn);
            turns_out[nt] = 0;
        } else {
            turn = WM_TURN_F;
        }

        head = (head + turn) & 3;              // the turn code IS the heading delta
        if (!is_open(c, r, head)) { printf("  turn faces a WALL at %s\n", cellname(c, r)); return -1; }
        c += DX[head]; r += DY[head];
        if (!in_floor(c, r)) { *legs_out = steps + 1; return 1; }   // out through a gap
        strcat(path_out, " ");
        strcat(path_out, cellname(c, r));
    }
    printf("  did not terminate\n");
    return -1;
}

// ---------------------------------------------------------------------------
static int fails = 0;
static void check(int cond, const char *what) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}

// Every one of the nine ways a dead-end excursion can fold, checked against
// the table derived by hand, not against the formula that produced them.
static void test_fold_table(void) {
    static const uint8_t A[3] = { WM_TURN_L, WM_TURN_F, WM_TURN_R };
    static const uint8_t B[3] = { WM_TURN_L, WM_TURN_F, WM_TURN_R };
    static const uint8_t EXPECT[3][3] = {
        /*        B=L         B=F         B=R    */
        /* A=L */ { WM_TURN_F, WM_TURN_R, WM_TURN_U },
        /* A=F */ { WM_TURN_R, WM_TURN_U, WM_TURN_L },
        /* A=R */ { WM_TURN_U, WM_TURN_L, WM_TURN_F },
    };
    int i, j, ok = 1;
    printf("\n-- fold table (A, U, B) -> net --\n");
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            uint8_t stray = 0, got;
            WallMem_Reset();
            WallMem_Record(1, 1, 1, A[i]);
            WallMem_Record(0, 0, 0, WM_TURN_U);
            WallMem_Record(1, 1, 1, B[j]);
            WallMem_Reduce(&stray);
            got = WallMem_Turn(WallMem_At(0));
            // Three of the nine fold to a U, and that is correct, not a bug:
            // it means the junction's whole subtree was a dead end, so the
            // robot has to reverse back out of the junction as well. In a real
            // log that U folds again with ITS neighbours on the next pass --
            // test_nested_dead_end() is that case. Here the record stands
            // alone with nothing to fold into, so it is reported stray.
            printf("   %c U %c -> %c (expect %c)%s%s\n", turn_ch(A[i]), turn_ch(B[j]),
                   turn_ch(got), turn_ch(EXPECT[i][j]),
                   (WallMem_Count() == 1 && got == EXPECT[i][j]) ? "" : "   <-- WRONG",
                   (got == WM_TURN_U) ? "   (net U: folds again when nested)" : "");
            if (WallMem_Count() != 1 || got != EXPECT[i][j]) ok = 0;
            if (stray != ((got == WM_TURN_U) ? 1u : 0u)) ok = 0;
        }
    }
    check(ok, "all nine folds correct, each collapsing 3 records to 1");
}

static void test_nested_dead_end(void) {
    // A dead end four turns deep: R R L L in, the mirror out. Collapses to one
    // record only if the fold is applied repeatedly from the inside out.
    uint8_t stray = 0;
    WallMem_Reset();
    WallMem_Record(1, 1, 0, WM_TURN_L);   // junction: into the branch
    WallMem_Record(0, 0, 1, WM_TURN_R);
    WallMem_Record(0, 0, 1, WM_TURN_R);
    WallMem_Record(0, 1, 0, WM_TURN_L);
    WallMem_Record(0, 1, 0, WM_TURN_L);
    WallMem_Record(0, 0, 0, WM_TURN_U);   // the dead end
    WallMem_Record(0, 0, 1, WM_TURN_R);
    WallMem_Record(0, 0, 1, WM_TURN_R);
    WallMem_Record(0, 1, 0, WM_TURN_L);
    WallMem_Record(0, 1, 0, WM_TURN_L);
    WallMem_Record(1, 1, 0, WM_TURN_F);   // back at the junction, carrying on
    printf("\n-- nested dead end --\n");
    WallMem_Reduce(&stray);
    printf("   collapsed 11 -> %u, stray=%u, turn=%c\n",
           WallMem_Count(), stray, turn_ch(WallMem_Turn(WallMem_At(0))));
    // L then (four turns in, U, four turns out) then F, all folding pairwise.
    check(WallMem_Count() == 1 && stray == 0, "11 records collapse to 1");
}

static void test_storage(void) {
    uint8_t i, rec;
    printf("\n-- record packing --\n");
    WallMem_Reset();
    WallMem_Record(1, 0, 1, WM_TURN_R);
    rec = WallMem_At(0);
    printf("   F=1 L=0 R=1 turn=R packs to 0x%02X\n", rec);
    check(rec == (WM_OPEN_F | WM_OPEN_R | WM_TURN_R), "bit layout matches wallmem.h");
    check(WallMem_RecordIsSane(rec), "reserved bits are clear");
    check(!WallMem_RecordIsSane(0xFF), "erased EEPROM (0xFF) is rejected as a record");
    check(WallMem_Sig(rec) == WallMem_PackSig(1, 0, 1), "signature round-trips");

    printf("\n-- overflow --\n");
    WallMem_Reset();
    for (i = 0; i < WALLMEM_MAX_RECORDS; i++) WallMem_Record(1, 1, 0, WM_TURN_L);
    check(WallMem_Record(1, 1, 0, WM_TURN_L) == 0, "the record past the ceiling is refused");
    check(WallMem_Overflowed() == 1, "overflow is reported, not silent");
}

static void test_eeprom_corruption(void) {
    uint8_t stray = 0;
    printf("\n-- EEPROM validation --\n");
    WallMem_HostEraseEeprom();
    check(WallMem_Load() == 0, "a blank EEPROM does not load as a route");

    WallMem_Reset();
    WallMem_Record(0, 1, 1, WM_TURN_L);
    WallMem_Record(1, 0, 1, WM_TURN_F);
    WallMem_Reduce(&stray);
    check(WallMem_Save() == 1, "save succeeds and reads back");
    WallMem_Reset();
    check(WallMem_Load() == 1, "a clean block loads");
    check(WallMem_Count() == 2 && WallMem_HaveSolution(), "count and solved flag survive");

    // Corrupt one stored record. The CRC has to catch it -- this is the
    // failure that matters, because a single flipped bit in a turn code sends
    // the robot down the wrong corridor with total confidence.
    {
        uint8_t victim = WallMem_HostPeekEeprom(WALLMEM_EE_BASE + 8);
        WallMem_HostPokeEeprom(WALLMEM_EE_BASE + 8, (uint8_t)(victim ^ 0x01u));
        WallMem_Reset();
        check(WallMem_Load() == 0, "a single flipped bit in a record is rejected");
        WallMem_HostPokeEeprom(WALLMEM_EE_BASE + 8, victim);
        WallMem_Reset();
        check(WallMem_Load() == 1, "...and the same block loads again once restored");
    }

    // A brown-out part way through a write is the realistic corruption on this
    // chassis. The magic byte is written LAST precisely so that case leaves a
    // block that fails cleanly rather than one that half-drives.
    WallMem_HostPokeEeprom(WALLMEM_EE_BASE + 0, 0xFFu);
    WallMem_Reset();
    check(WallMem_Load() == 0, "a block with no magic (interrupted write) is rejected");

    // A count past the ceiling must not be trusted enough to read that far.
    WallMem_Reset();
    WallMem_Record(0, 1, 1, WM_TURN_L);
    WallMem_Reduce(&stray);
    WallMem_Save();
    WallMem_HostPokeEeprom(WALLMEM_EE_BASE + 3, (uint8_t)(WALLMEM_MAX_RECORDS + 1));
    WallMem_Reset();
    check(WallMem_Load() == 0, "an impossible record count is rejected");
}

int main(void) {
    char t1[256] = { 0 }, t2[256] = { 0 }, p1[512] = { 0 }, p2[512] = { 0 };
    int legs1 = 0, legs2 = 0, ok1, ok2;
    uint8_t stray = 0, before;

    build_maze();
    WallMem_HostEraseEeprom();

    printf("======== RUN 1: explore, left-hand rule ========\n");
    WallMem_Reset();
    ok1 = walk(1, t1, p1, &legs1);
    printf("  path : %s -> out\n  turns: %s (%u records)\n", p1, t1, WallMem_Count());
    check(ok1 == 1, "run 1 reaches the exit");
    check(strcmp(t1, "LLULLRRLLURRLLFLUFL") == 0, "run-1 turn string is as designed");
    check(WallMem_Count() == 19, "19 decision points logged");
    WallMem_Dump();

    printf("\n======== COLLAPSE ========\n");
    before = WallMem_Count();
    WallMem_Reduce(&stray);
    printf("  %u -> %u records, stray U-turns = %u\n", before, WallMem_Count(), stray);
    check(stray == 0, "no U-turn survives (the maze has no loops)");
    check(WallMem_Count() == 5, "collapses to 5 records");
    WallMem_Dump();

    printf("\n======== EEPROM ROUND TRIP ========\n");
    check(WallMem_Save() == 1, "route saved");
    WallMem_Reset();
    check(WallMem_Load() == 1, "route loaded back after a power cycle");
    check(WallMem_HaveSolution() == 1, "loaded route is flagged solved");

    printf("\n======== RUN 2: replay ========\n");
    WallMem_RewindPlayback();
    ok2 = walk(0, t2, p2, &legs2);
    printf("  path : %s -> out\n  turns: %s\n", p2, t2);
    check(ok2 == 1, "run 2 reaches the exit");
    check(strcmp(t2, "LFRRL") == 0, "run-2 turn string is the collapsed one");
    check(strcmp(p2, "E0 E1 D1 C1 C2 D2 E2") == 0, "run 2 drives the optimal path");
    check(WallMem_PlaybackIndex() == WallMem_Count(), "the whole route is consumed, exactly");

    printf("\n  legs: run1 %d (%d cm)   run2 %d (%d cm)   %.2fx shorter\n",
           legs1, legs1 * 40, legs2, legs2 * 40, (double)legs1 / (double)legs2);
    check(legs1 == 21 && legs2 == 7, "21 legs explored, 7 legs on the fast run");

    test_fold_table();
    test_nested_dead_end();
    test_storage();
    test_eeprom_corruption();

    printf("\n%s  (%d failures)\n", fails ? "*** FAILED ***" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
