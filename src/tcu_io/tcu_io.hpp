//
// Created by ashcon on 9/2/24.
// Slimmed down for MinTCU
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

    // false if the sensors could not be set up (Output sensor enabled with 0 pulses/rev)
    bool setup_io_layer();
    // Polled every 20ms (Gearbox loop)
    void update_io_layer();

    void set_2_1_ratio(float ratio);
    uint16_t calc_turbine_rpm(const uint16_t n2, const uint16_t n3);

    // INT16_MAX if not available
    int16_t atf_temperature();
    // UINT16_MAX if not available
    uint16_t battery_mv();
    // 1 = P/N, 0 = in gear, UINT8_MAX if not available
    uint8_t parking_lock();
    // UINT16_MAX if not available
    uint16_t n2_rpm();
    uint16_t n3_rpm();
    // UINT16_MAX if not available (No dedicated output sensor configured. UN52 then uses the
    // rear wheel speeds from CAN, MinTCU does not, a W124 has no wheel speeds on CAN)
    uint16_t output_rpm();
    // INT16_MAX if not available
    int16_t motor_temperature();
};

#endif //__TCU_IO_H
