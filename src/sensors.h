/** @file */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

struct SensorDataRaw {
    uint16_t rpm_n2;      // UINT16_MAX if not available
    uint16_t rpm_n3;      // UINT16_MAX if not available
    uint16_t rpm_out;     // UINT16_MAX if not available (Or no dedicated output sensor)
    uint16_t battery_mv;  // UINT16_MAX if not available
    int16_t atf_temp_c;   // INT16_MAX if not available (Also when parking lock is engaged)
    uint8_t parking_lock; // 1 = P/N (TFT circuit open), 0 = in gear, UINT8_MAX = not available
};

namespace Sensors {
    /**
     * @brief Initializes the sensor subsystem of the gearbox (RPM sensor interrupts)
     *
     * @return true If initialization OK
     * @return false if initialization failed (Output sensor enabled with 0 pulses/rev)
     */
    bool init_sensors(void);
    /**
     * @brief Reads the sensors on the Mega (N2/N3/output RPM, battery voltage, ATF temperature / parking lock)
     */
    void update(SensorDataRaw* dest);
    bool using_dedicated_output_rpm();
}

#endif // SENSORS_H
