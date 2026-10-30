#ifndef CLUTCH_SPEED_H
#define CLUTCH_SPEED_H

#include <stdint.h>
#include "common_structs.h"

// From UN52 models/clutch_speed.hpp (What the shift algorithms use)
struct SpeedSensors {
    uint16_t n2;
    uint16_t n3;
    uint16_t turbine;
    uint16_t output;
};

namespace ClutchSpeedModel {
    ShiftClutchData get_shifting_clutch_speeds(const SpeedSensors speeds, const GearChange req, const GearRatioInfo* ratios);
}

#endif
