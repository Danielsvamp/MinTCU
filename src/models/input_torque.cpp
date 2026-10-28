#include "input_torque.hpp"
#include "tcu_maths.h"
#include "egs_calibration/calibration_structs.h"

// Same as UN52 models/input_torque.cpp (get_pump_torque not ported yet, it needs the TCC calibration)
// 'input*1000' is done in int32_t, it overflows a 16 bit int on AVR

int16_t InputTorqueModel::get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque) {
    int16_t ret = 0;
    if (static_torque == INT16_MAX) {
        ret = INT16_MAX;
    } else if (engine_rpm == 0 || engine_rpm == INT16_MAX || engine_rpm == UINT16_MAX) {
        ret = static_torque;
    } else {
        float multi = InputTorqueModel::get_input_torque_factor(engine_rpm, input_rpm);
        ret = static_torque * multi;
    }
    return ret;
}

float InputTorqueModel::get_input_torque_factor(uint16_t engine, uint16_t input) {
    if (0 == engine) {
        return 1.0;
    }
    float rpm_multi = ((int32_t)input*1000L) / (int32_t)engine;

    //Interpolate map, but faster, since its just 4 values, no need to allocate a whole map for this
    float x1 = (float)(TCC_CFG_PTR->multiplier_map_x[0]);
    float x2 = (float)(TCC_CFG_PTR->multiplier_map_x[1]);
    float z1 = (float)(TCC_CFG_PTR->multiplier_map_z[0]) / 100.0;
    float z2 = (float)(TCC_CFG_PTR->multiplier_map_z[1]) / 100.0;
    return interpolate_float(rpm_multi, z1, z2, x1, x2, InterpType::Linear);
}
