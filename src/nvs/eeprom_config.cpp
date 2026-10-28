#include "eeprom_config.h"

// Fill in for your car. Everything not set here is 0 (not used by MinTCU yet).
TCM_CORE_CONFIG VEHICLE_CONFIG = {
    .deprecated_is_large_nag = 0,
    .diff_ratio = 0,
    .wheel_circumference = 0,
    .is_four_matic = 0,
    .transfer_case_high_ratio = 0,
    .transfer_case_low_ratio = 0,
    .default_profile = 0,
    .red_line_rpm_diesel = 0,
    .red_line_rpm_petrol = 0,
    .engine_type = 0,
    // CAN frames the engine ECU sends (Same numbers as UN52): 1 = EGS51, 2 = EGS52
    // W210 EDC15C6 from an EGS51 car (722.634) = 1
    .egs_can_type = 1,
    .shifter_style = 0, // EWM
    // 1 = dedicated output speed sensor on Pin::io_pin (Hall sensor)
    .io_0_usage = 1,
    // Pulses per revolution of the output speed sensor. 0 = not set, the output sensor is then
    // not used (Same as UN52). SET THIS for your sensor / tone wheel!
    .input_sensor_pulses_per_rev = 0,
    .output_pulse_width_per_kmh = 0,
    .gen_mosfet_purpose = 0,
    .throttlevalve_maxopeningangle = 0,
    .c_eng = 0,
    .engine_drag_torque = 0,
    .jeep_chrysler = false,
};
