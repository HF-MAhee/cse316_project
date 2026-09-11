#ifndef MAZE_H
#define MAZE_H
#include <stdint.h>

typedef enum {
    ST_STARTUP = 0,
    ST_DRIVING,          // centring forward, watching for junctions
    ST_APPROACHING,      // junction seen; drive on so the AXLE reaches it
    ST_CONFIRM_EXIT,     // all three looked open; drive on and re-check
    ST_STOPPING,         // brake and let the chassis settle
    ST_RECALIBRATING,    // gyro bias refresh while stationary
    ST_DECIDING,         // classify junction, pick a direction
    ST_TURNING,          // blocking pivot
    ST_RECOVERING,       // gyro-only while the sonar filters refill
    ST_FINISHED,         // maze exit found
    ST_FAULT
} maze_state_t;

typedef enum {
    J_CORRIDOR = 0,      // front open, both sides walled
    J_FWD_OR_LEFT,
    J_FWD_OR_RIGHT,
    J_LEFT_OR_RIGHT,
    J_FORCED_LEFT,
    J_FORCED_RIGHT,
    J_DEAD_END,
    J_ALL_OPEN           // candidate maze exit
} junction_t;

void         Maze_Init(void);
void         Maze_Tick(int16_t gyro_rate);
maze_state_t Maze_State(void);
const char  *Maze_StateName(void);
#endif
