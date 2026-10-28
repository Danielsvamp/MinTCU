#ifndef __MODULE_SETTINGS_H
#define __MODULE_SETTINGS_H

#include <stdint.h>

/*
    Slimmed down copy of UN52's module settings. Only what the pressure path needs.
    No EEPROM yet, these are compiled in.
*/

// Solenoid settings
typedef struct {
    // Below this battery voltage the MPC/SPC solenoids are switched off (0mA = max pressure)
    //
    // UNIT: mV
    uint16_t min_batt_cc_mv;
    // Above this battery voltage the MPC/SPC solenoids are switched off (0mA = max pressure)
    //
    // UNIT: mV
    uint16_t max_batt_cc_mv;
    // The temperature coefficient of the solenoid wiring and coils (copper)
    //
    // UNIT: % per C
    float cc_temp_coefficient_wires;
    // MPC and SPC solenoids resistance at cc_reference_temp. Measure YOUR solenoid + harness
    // resistance with a meter at the TCU plug and put it here!
    //
    // UNIT: Ohm
    float cc_reference_resistance;
    // MPC and SPC solenoids resistance reference temperature
    //
    // UNIT: C
    float cc_reference_temp;
    // Voltage drop of the flyback diode across the solenoid (used in the PWM duty calculation)
    // ~400 for a schottky, ~700 for a normal diode
    //
    // UNIT: mV
    uint16_t cc_flyback_diode_drop_mv;
    // Voltage drop of the MOSFET + wiring when ON
    //
    // UNIT: mV
    uint16_t cc_driver_drop_mv;
} __attribute__ ((packed)) SOL_MODULE_SETTINGS;

const SOL_MODULE_SETTINGS SOL_DEFAULT_SETTINGS = {
    9000,   // min_batt_cc_mv (same as UN52)
    16000,  // max_batt_cc_mv
    0.393,  // cc_temp_coefficient_wires (same as UN52)
    5.3,    // cc_reference_resistance (same as UN52)
    25,     // cc_reference_temp (same as UN52)
    700,    // cc_flyback_diode_drop_mv
    100,    // cc_driver_drop_mv
};

// Pressure manager settings
typedef struct {
    // DEBUG: Friction coefficient when stationary (In gear)
    uint8_t stationary_coefficient;
    // DEBUG: Friction coefficient when releasing a clutch
    uint8_t releasing_coefficient;
    // DEBUG: Friction coefficient when applying a clutch (Cold)
    uint8_t applying_coefficient_cold;
    // DEBUG: Friction coefficient when applying a clutch (Hot)
    uint8_t applying_coefficient_hot;
} __attribute__ ((packed)) PRM_MODULE_SETTINGS;

// Same as UN52 defaults
const PRM_MODULE_SETTINGS PRM_DEFAULT_SETTINGS = {
    100, // stationary_coefficient
    120, // releasing_coefficient
    185, // applying_coefficient_cold
    140, // applying_coefficient_hot
};

extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;

#endif
