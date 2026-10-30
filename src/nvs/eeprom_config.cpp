#include "eeprom_config.h"

// UN52's defaults (nvs/eeprom_config.cpp, used when no SCN config is stored), except where it says MinTCU.
// Fill in for your car.
TCM_CORE_CONFIG VEHICLE_CONFIG = {
    .deprecated_is_large_nag = 0,
    .diff_ratio = 1000,
    .wheel_circumference = 2850,
    .is_four_matic = 0,
    .transfer_case_high_ratio = 1000,
    .transfer_case_low_ratio = 1000,
    .default_profile = 0, // Standard
    .red_line_rpm_diesel = 4500, // Safe for diesels, petrol-heads can change this!
    .red_line_rpm_petrol = 6000,
    .engine_type = 0, // 0 = diesel, 1 = petrol
    // MinTCU: CAN frames the engine ECU sends (Same numbers as UN52): 1 = EGS51, 2 = EGS52
    // The W210 EDC15C6 sends its torque in MS_312/MS_212 (EGS52 frames), not MS_310 (Found from the
    // EDC15C6 CAN frame map, not checked on Daniel's car yet). If the log shows failsafe 2 (no torque)
    // with the engine running, try 1
    .egs_can_type = 2,
    .shifter_style = 0, // EWM
    // MinTCU: 1 = dedicated output speed sensor on Pin::io_pin (Hall sensor)
    .io_0_usage = 1,
    // MinTCU: pulses per revolution of the output speed sensor. 0 = no sensor fitted, the output
    // speed is then estimated from the input speed and the gear ratio. SET THIS for your sensor / tone wheel!
    .input_sensor_pulses_per_rev = 0,
    .output_pulse_width_per_kmh = 1,
    .gen_mosfet_purpose = 0,
    .throttlevalve_maxopeningangle = 255, // 89.25°
    .c_eng = 1000,
    .engine_drag_torque = 400, // 40Nm
    .jeep_chrysler = false,
};
