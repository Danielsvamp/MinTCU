#include "clutch_speed.hpp"

// Same as UN52 models/clutch_speed.cpp
ShiftClutchData ClutchSpeedModel::get_shifting_clutch_speeds(const SpeedSensors speeds, const GearChange req, const GearRatioInfo* ratios) {
    ShiftClutchData ret = {0,0,0};
    if (
        req == GearChange::_1_2 || req == GearChange::_2_1 ||
        req == GearChange::_4_5 || req == GearChange::_5_4
    ) {
        int16_t vk1 = (int16_t)speeds.n2 - (int16_t)speeds.n3;
        int16_t vb1 = (int16_t)speeds.n3;
        ret.on_clutch_speed = (req == GearChange::_1_2 || req == GearChange::_5_4) ? vk1 : vb1;
        ret.off_clutch_speed = (req == GearChange::_1_2 || req == GearChange::_5_4) ? vb1 : vk1;
        if (req == GearChange::_4_5 || req == GearChange::_5_4) {
            // B2 is open, calculate the speed
            ret.rear_sun_speed = (ratios[RAT_3_IDX].ratio*(float)speeds.output - (float)speeds.turbine)/(ratios[RAT_3_IDX].ratio - ratios[RAT_4_IDX].ratio);
        } // Else it is 0
    } else if (req == GearChange::_2_3 || req == GearChange::_3_2) {
        int16_t vk2 = (int16_t)speeds.n3 - (ratios[RAT_3_IDX].ratio * (int16_t)speeds.output);
        int16_t vk3 = (ratios[RAT_3_IDX].ratio*(ratios[RAT_2_IDX].ratio*(float)speeds.output - (float)speeds.n3)) / (ratios[RAT_2_IDX].ratio - ratios[RAT_3_IDX].ratio);
        ret.on_clutch_speed = req == GearChange::_2_3 ? vk2 : vk3;
        ret.off_clutch_speed = req == GearChange::_2_3 ? vk3 : vk2;
    } else if (req == GearChange::_3_4 || req == GearChange::_4_3) {
        int16_t vb2 = (ratios[RAT_3_IDX].ratio*(float)speeds.output - (float)speeds.n3)/(ratios[RAT_3_IDX].ratio - ratios[RAT_4_IDX].ratio);
        int16_t vk3 = ((int16_t)speeds.n3 - vb2);
        ret.on_clutch_speed = req == GearChange::_3_4 ? vk3 : vb2;
        ret.off_clutch_speed = req == GearChange::_3_4 ? vb2 : vk3;
        ret.rear_sun_speed = vb2;
    }
    return ret;
}
