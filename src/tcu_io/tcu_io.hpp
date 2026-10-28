//
// Created by ashcon on 9/2/24.
// Slimmed down for MinTCU (No RPM sensors yet)
//

#ifndef __TCU_IO_H
#define __TCU_IO_H

#include <stdint.h>
#include "canbus/can_hal.h"

namespace TCUIO {

    struct SmoothedSensor {
        uint8_t e_counter;
        uint8_t sample_count;
        // Multiplied by 100
        int32_t last_value;
    };

    template <typename T>
    struct OnePollSensor {
        uint8_t e_counter;
        T current_value;
    };

    void setup_io_layer();
    // Polled every 20ms (Gearbox loop)
    void update_io_layer();

    // INT16_MAX if not available
    int16_t atf_temperature();
    // UINT16_MAX if not available
    uint16_t battery_mv();
    // 1 = P/N, 0 = in gear, UINT8_MAX if not available
    uint8_t parking_lock();
    // INT16_MAX if not available
    int16_t motor_temperature();
};

#endif //__TCU_IO_H
