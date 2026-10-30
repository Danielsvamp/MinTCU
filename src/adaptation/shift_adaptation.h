#ifndef SHIFT_ADAPTATION_H
#define SHIFT_ADAPTATION_H

#include <stdint.h>
#include "stored_map.h"

/*
    MinTCU has no shift adaptation. UN52's adaptation (adaptation/shift_adaptation.cpp) stores its
    values in NVS, which the Mega does not have.

    The shift algorithms only use it through ShiftInterfaceData::adaptation_mgr, and check it for
    nullptr first. MinTCU always passes nullptr, which is the same as UN52 without an adaptation manager.
    These declarations are only here so the UN52 shift algorithm code compiles unchanged, nothing
    creates a ShiftAdaptationSystem.
*/

// Same as UN52 adaptation/shift_adaptation.cpp
const int16_t TRQ_ADAPT_X[] = {1000, 1500, 2000, 4000};
const int16_t TRQ_ADAPT_Y_PULL[] = {0, 20, 50, 100, 150};

class ShiftAdaptationSystem {
public:
    int8_t get_prefill_cycles_offset(uint8_t shift_idx) { return 0; }
    int16_t get_adapt_spc_offset(uint8_t shift_idx) { return 0; }
    int16_t get_pulling_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage) { return 0; }
    int16_t get_pushing_torque_offset(uint8_t shift_idx, uint16_t input_rpm, uint16_t load_percentage) { return 0; }
    void offset_prefill_cycles(uint8_t shift_idx, int8_t offset) {}
    void offset_spc_pressure(uint8_t shift_idx, int16_t offset) {}
    StoredMap* get_pulling_torque_map(uint8_t shift_idx) { return nullptr; }
    StoredMap* get_pushing_torque_map(uint8_t shift_idx) { return nullptr; }
};

#endif
