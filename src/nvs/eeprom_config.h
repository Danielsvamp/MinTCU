/** @file */
#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <stdint.h>

/*
    UN52's vehicle config struct (nvs/eeprom_config.h), same fields and names.
    VEHICLE_CONFIG is compiled in here (eeprom_config.cpp), it is not stored in EEPROM.
    Only egs_can_type, io_0_usage and input_sensor_pulses_per_rev are used so far.
*/
struct __attribute__ ((packed)) TCM_CORE_CONFIG{
    uint8_t deprecated_is_large_nag;
    uint16_t diff_ratio;
    uint16_t wheel_circumference;
    uint8_t is_four_matic;
    uint16_t transfer_case_high_ratio;
    uint16_t transfer_case_low_ratio;
    uint8_t default_profile;
    uint16_t red_line_rpm_diesel;
    uint16_t red_line_rpm_petrol;
    uint8_t engine_type; // 0 for diesel, 1 for petrol
    uint8_t egs_can_type;
    // 0 - EWM
    // 1 - TRRS
    // 2 - SLR
    uint8_t shifter_style;
    // 0 - N/C
    // 1 - Vehicle speed sensor
    // 2 - Output (TBA)
    // 3 - TCC Manual MOSFET (1.3 ONLY)
    uint8_t io_0_usage;
    uint8_t input_sensor_pulses_per_rev;
    uint8_t output_pulse_width_per_kmh;
    uint8_t gen_mosfet_purpose;
    // values required for classic Hfm
    // maximum opening angle of the throttle valve (in [°] to be more universal)
    uint8_t throttlevalve_maxopeningangle;
    // constant factor required to convert mdot and engine speed to the engine torque (in [m²/s²] * 1000)
    uint16_t c_eng;
    // Engine drag torque (Nm/10)
    uint16_t engine_drag_torque;
    // Boolean for Jeep/Chrysler mode
    bool jeep_chrysler;
};

#define NUM_GEARS 5
extern TCM_CORE_CONFIG VEHICLE_CONFIG;

#endif
