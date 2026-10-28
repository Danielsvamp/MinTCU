#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

struct SensorDataRaw {
    uint16_t battery_mv;  // UINT16_MAX if not available
    int16_t atf_temp_c;   // INT16_MAX if not available (Also when parking lock is engaged)
    uint8_t parking_lock; // 1 = P/N (TFT circuit open), 0 = in gear, UINT8_MAX = not available
};

namespace Sensors {
    /**
     * @brief Reads the analog sensors on the Mega (battery voltage, ATF temperature / parking lock)
     */
    void update(SensorDataRaw* dest);
}

#endif // SENSORS_H
