#include "maps.h"

// Copied from UN52 maps.cpp

const int16_t LARGE_NAG_FILL_TIME_MAP[FILL_TIME_MAP_SIZE] = {
    /* ATF TEMP             */
    /*  -20C,  5C, 25C, 60C */
         600, 360, 220, 160, // K1 clutch
        1620, 560, 260, 160, // K2 clutch
         860, 500, 160, 160, // K3 clutch
         600, 380, 220, 180, // B1 brake
         820, 680, 260, 120  // B2 brake
};

const int16_t NAG_FILL_PRESSURE_MAP[FILL_PRESSURE_MAP_SIZE] = {
    /* Clutch                    */
    /* K1    K2    K3    B1    B2    B3 */
     1500, 1500, 1500, 1500, 1500, 1500
};

const int16_t NAG_FILL_LOW_PRESSURE_MAP[LOW_FILL_PRESSURE_MAP_SIZE] = {
    /* Clutch                    */
    /* K1    K2    K3    B1    B2 */
      700,  700,  750,  700,  620
};

const int16_t M_UPSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1500, 1000,  750,  500,  500,
    /* 25% Redline     */ 1000,  900,  800,  600,  475,  450,
    /* 50% Redline     */  700,  650,  600,  500,  450,  400,
    /* 75% Redline     */  500,  475,  450,  425,  400,  375,
    /* Redline (100%)  */  450,  425,  400,  375,  350,  350
};

const int16_t M_DOWNSHIFT_TIME_MAP[] = { // Value = Target time in ms to shift (overlap duration)
    /*                       0    20    40   60    80  100 <- Pedal % */
    /* < 1000 RPM (0%) */ 2000, 1750, 1500, 1000,  750,  500,
    /* 25% Redline     */ 1750, 1500, 1200,  800,  650,  450,
    /* 50% Redline     */ 1500, 1250,  900,  600,  600,  400,
    /* 75% Redline     */ 1000, 1000,  600,  550,  550,  350,
    /* Redline (100%)  */  800,  750,  500,  500,  400,  300
};
