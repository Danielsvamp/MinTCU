#ifndef CLUTCH_SPEED_H
#define CLUTCH_SPEED_H

#include <stdint.h>

// From UN52 models/clutch_speed.hpp (Only the speed sensor struct so far)
struct SpeedSensors {
    uint16_t n2;
    uint16_t n3;
    uint16_t turbine;
    uint16_t output;
};

#endif
