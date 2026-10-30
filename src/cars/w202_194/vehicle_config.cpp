#include "../../nvs/eeprom_config.h"

// W202 C220 CDI, chassis 202.194: CR1 EDC15, 722.614 or 722.699, original ABS on the CAN bus.
// UN52's defaults (nvs/eeprom_config.cpp, used when no SCN config is stored), except where it says MinTCU.

// MinTCU: the diff ratio comes from the car's SCN coding (x1000, eg 3.070 = 3070). It turns the rear wheel
// speeds on CAN into the output shaft speed, so a wrong value makes every shift decision wrong.
// It is not in the UN52 calibration database, so it is left for the coding of this car:
// build with -D W202_194_DIFF_RATIO=xxxx, or put the number here.
#ifndef W202_194_DIFF_RATIO
#error "Set W202_194_DIFF_RATIO (diff ratio x1000 from the 202.194 coding) in src/cars/w202_194/vehicle_config.cpp"
#endif

TCM_CORE_CONFIG VEHICLE_CONFIG = {
    .deprecated_is_large_nag = 0,
    .diff_ratio = W202_194_DIFF_RATIO,
    .wheel_circumference = 2850,
    .is_four_matic = 0,
    .transfer_case_high_ratio = 1000,
    .transfer_case_low_ratio = 1000,
    .default_profile = 0, // Standard
    .red_line_rpm_diesel = 4500, // Safe for diesels, petrol-heads can change this!
    .red_line_rpm_petrol = 6000,
    .engine_type = 0, // 0 = diesel, 1 = petrol
    // MinTCU: CAN frames the engine ECU sends (Same numbers as UN52): 1 = EGS51, 2 = EGS52
    // The CR1 EDC15 sends MS_210, MS_308, MS_310 and MS_608 and no MS_212/MS_312 (CAN table in its
    // calibration blocks at +0x1CC4, found in the flash, not checked on the car yet)
    .egs_can_type = 1,
    .shifter_style = 0, // EWM
    // MinTCU: 0 = no output speed sensor, the output speed comes from the rear wheel speeds in the
    // ABS frame BS_208 (Same as UN52). If BS_208 is missing it is estimated from the input speed
    // and the gear ratio, like a car without the sensor
    .io_0_usage = 0,
    // MinTCU: pulses per revolution of the output speed sensor, 0 = no sensor fitted
    .input_sensor_pulses_per_rev = 0,
    .output_pulse_width_per_kmh = 1,
    .gen_mosfet_purpose = 0,
    .throttlevalve_maxopeningangle = 255, // 89.25°
    .c_eng = 1000,
    .engine_drag_torque = 400, // 40Nm
    .jeep_chrysler = false,
};
