#include "shifting_algo_helpers.h"
#include "egs_calibration/calibration_structs.h"

// Same as UN52, the maths is done in int32_t (int is 16 bit on AVR)
uint16_t ShiftHelpers::correct_shift_shift_pressure(PressureManager* pm, int16_t pressure, uint8_t map_idx) {
    // TODO - Move max_p to global constant so it can be referred in other functions
    int32_t max_p = pm->get_max_shift_pressure(map_idx);
    int32_t p = pressure;
    if (p <= 0) {
        p = 0;
    } else if (p >= max_p) {
        p = max_p;
    }
    // P*1000 as shift_spc_gain is *1000
    return (uint16_t)(((p * 1000) / HYDR_PTR->shift_spc_gain[map_idx]) + HYDR_PTR->shift_reg_spring_pressure);
}
