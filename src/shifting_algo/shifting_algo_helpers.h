#ifndef SHIFTING_ALGO_HELPERS_H
#define SHIFTING_ALGO_HELPERS_H

#include <stdint.h>
#include "pressure_manager.h"

// From UN52 shifting_algo/s_algo.h (Only what the garage shift needs)
namespace ShiftHelpers {
    uint16_t correct_shift_shift_pressure(PressureManager* pm, int16_t pressure, uint8_t map_idx);
}

#endif
