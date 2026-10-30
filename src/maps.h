#ifndef MAPS_H
#define MAPS_H

#include <stdint.h>

/*
    Default maps from UN52 maps.h / maps.cpp (Only the ones MinTCU uses).
    UN52 stores these in NVS (StoredMap) and uses these values when nothing is stored.
    MinTCU only saves the TCC adaptation maps (nvs/eeprom_impl.cpp), so the others are always these.
*/

// -- FILL TIME --
#define FILL_TIME_MAP_SIZE 5*4 // 4 temp positions, 5 clutch groups
extern const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE];

#define FILL_PRESSURE_MAP_SIZE 6 // 6 clutches (Including B3 for reverse), 4 load points
extern const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE];

#define LOW_FILL_PRESSURE_MAP_SIZE 5 // Only for forward clutches (K1, K2, K3, B1, B2)
extern const int16_t NAG_FILL_LOW_PRESSURE_MAP[LOW_FILL_PRESSURE_MAP_SIZE];

#define TCC_PWM_MAP_SIZE 7*5 // 5 temp positions, 7 pressure readings
extern const int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE];

#define TCC_RPM_TARGET_MAP_SIZE 11*8
extern const int16_t TCC_RPM_TARGET_MAP[TCC_RPM_TARGET_MAP_SIZE];

// From UN52 adapt_maps.h
#define TCC_ADAPT_MAP_Z_SIZE 6*6
extern const int16_t TCC_ADAPT_MAP_X[6];
extern const int16_t TCC_ADAPT_MAP_Y[6];
extern const int16_t TCC_ADAPT_MAP_Z[TCC_ADAPT_MAP_Z_SIZE];

// All maps here except TCC_ADAPT_MAP_X/Y are in flash (TCU_FLASH). The shift maps are read with
// LookupFlashMap, the others are copied into their map with data_in_flash = true. Never read them as
// a normal array.
#define SHIFT_MAP_SIZE 44
extern const int16_t S_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t S_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t C_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t A_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_DIESEL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_DIESEL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_PETROL_UPSHIFT_MAP[SHIFT_MAP_SIZE];
extern const int16_t M_PETROL_DOWNSHIFT_MAP[SHIFT_MAP_SIZE];

#define SHIFT_TIME_MAP_SIZE 30
extern const int16_t M_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t M_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t C_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t C_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t S_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t S_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t A_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t A_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t W_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t W_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t R_UPSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];
extern const int16_t R_DOWNSHIFT_TIME_MAP[SHIFT_TIME_MAP_SIZE];

#endif
