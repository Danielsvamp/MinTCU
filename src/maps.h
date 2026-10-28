#ifndef MAPS_H
#define MAPS_H

#include <stdint.h>

/*
    Default maps from UN52 maps.h / maps.cpp (Only the ones MinTCU uses).
    UN52 stores these in NVS (StoredMap) and uses these values when nothing is stored.
    MinTCU has no NVS, so these are always used.
*/

// -- FILL TIME --
#define FILL_TIME_MAP_SIZE 5*4 // 4 temp positions, 5 clutch groups
extern const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

#define FILL_PRESSURE_MAP_SIZE 6 // 6 clutches (Including B3 for reverse), 4 load points
extern const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];

#define LOW_FILL_PRESSURE_MAP_SIZE 5 // Only for forward clutches (K1, K2, K3, B1, B2)
extern const int16_t NAG_FILL_LOW_PRESSURE_MAP[LOW_FILL_PRESSURE_MAP_SIZE];

#define SHIFT_TIME_MAP_SIZE 30
extern const int16_t M_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t M_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

#endif
