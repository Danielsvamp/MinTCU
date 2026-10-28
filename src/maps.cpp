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

const int16_t TCC_PWM_MAP[TCC_PWM_MAP_SIZE] = { // values are in /4096
 /*         TCC PRESSURE (mBar)              */
 /* 0   2000  4000  5000  7500  10000  15000 */
    0,   480,  960, 1280, 1920,  2560,  4096, // 0C
    0,   560, 1040, 1280, 1920,  2560,  4096, // 30C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 60C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 90C
    0,   640, 1120, 1280, 1920,  2560,  4096, // 120C
};

// VALUES (Target TCC slip):
// Values <=10 - Sets TCC is closed
// Values > 100 - Sets TCC is open
// Values between 10 and 100 - Tcc is slipping with desired target slip
const int16_t TCC_RPM_TARGET_MAP[TCC_RPM_TARGET_MAP_SIZE] = {
    /*  0    10   20   30   40   50   60   70   80   90   100 <- Pedal pos (%) */
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, // Input 1000 RPM
         50,  50,  50,  55,  60,  65,  70,  75,  80, 100, 100, // Input 1200 RPM
         10,  20,  40,  40,  50,  50,  55,  60,  60,  70,  80, // Input 1400 RPM
          0,  10,  20,  30,  30,  40,  40,  50,  50,  60,  70, // Input 1600 RPM
          0,   0,  10,  10,  20,  20,  30,  30,  40,  50,  50, // Input 1800 RPM
          0,   0,   0,   0,   0,   0,  20,  20,  25,  30,  30, // Input 2000 RPM
          0,   0,   0,   0,   0,   0,   0,   0,  10,  10,  20, // Input 4000 RPM
          0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10, // Input 6000 RPM
};

// From UN52 adapt_maps.cpp
const int16_t TCC_ADAPT_MAP_X[6] = {0, 5, 10, 25, 50, 100};
const int16_t TCC_ADAPT_MAP_Y[6] = {-20, 0, 20, 40, 80, 120};

const int16_t TCC_ADAPT_MAP_Z[TCC_ADAPT_MAP_Z_SIZE] = { // Z - Hydraulic Pressure
    //   0, 5, 10, 25, 50, 100  Torque (%)
         0, 0, 0, 0, 0, 0, // -20 C
    0, 0, 0, 0, 0, 0,// 0 C
    0, 0, 0, 0, 0, 0,// 20 C
    0, 0, 0, 0, 0, 0,// 40 C
    0, 0, 0, 0, 0, 0,// 80 C
    0, 0, 0, 0, 0, 0,// 120C
};
