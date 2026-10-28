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
    // 1N5822 schottky: about 400mV at 1A (datasheet forward voltage curve)
    //
    // UNIT: mV
    uint16_t cc_flyback_diode_drop_mv;
    // Voltage drop of the MOSFET when ON. IRLB3034: Rds(on) max 2.0mOhm at Vgs 4.5V, so ~2mV at 1A.
    // The harness resistance belongs in cc_reference_resistance (measure it at the TCU plug)
    //
    // UNIT: mV
    uint16_t cc_driver_drop_mv;
    // Reference voltage for the shift solenoid hold PWM (Same as UN52, the hold PWM is scaled by
    // cc_vref_solenoid / battery voltage)
    //
    // UNIT: mV
    uint16_t cc_vref_solenoid;
} __attribute__ ((packed)) SOL_MODULE_SETTINGS;

const SOL_MODULE_SETTINGS SOL_DEFAULT_SETTINGS = {
    9000,   // min_batt_cc_mv (same as UN52)
    16000,  // max_batt_cc_mv
    0.393,  // cc_temp_coefficient_wires (same as UN52)
    5.3,    // cc_reference_resistance (same as UN52)
    25,     // cc_reference_temp (same as UN52)
    400,    // cc_flyback_diode_drop_mv (1N5822)
    2,      // cc_driver_drop_mv (IRLB3034)
    12000,  // cc_vref_solenoid (same as UN52)
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

// Garage shift settings (Same fields and defaults as UN52)
typedef struct {
    // Threshold output speed where the gearbox
    // will force hydraulic N when trying to shift into R
    // to prevent damage to the gearbox. Once below this
    // output shaft speed, the gearbox will allow the garage
    // shift to proceed
    uint16_t output_rpm_force_n_rev;
    // Threshold output speed where the gearbox
    // will force hydraulic N when trying to shift from N to D
    // to prevent damage to the gearbox. Once below this
    // output shaft speed, the gearbox will allow the N-D
    // shift to proceed
    uint16_t output_rpm_force_n;
    // Modulating pressure during applying phase
    uint16_t p_mod_applying;
    // Modulating pressure during synchronization phase
    uint16_t p_mod_sync;
    // Shift pressure ramp (In mBar/20ms)
    uint16_t spc_ramp_apply;
    // Clutch speed threshold to determine if the
    // garage shift was completed OK
    uint16_t completed_rpm;
    // Clutch speed threshold to proceed from apply to synchronization,
    // when not applying the accelerator
    uint16_t sync_rpm_idle;
    // Clutch speed threshold to proceed from apply to synchronization,
    // when applying the accelerator
    uint16_t sync_rpm_with_accel;
} __attribute__ ((packed)) GAR_MODULE_SETTINGS;

const GAR_MODULE_SETTINGS GAR_DEFAULT_SETTINGS = {
    50,   // output_rpm_force_n_rev
    1000, // output_rpm_force_n
    3000, // p_mod_applying
    5000, // p_mod_sync
    10,   // spc_ramp_apply
    20,   // completed_rpm
    90,   // sync_rpm_idle
    350,  // sync_rpm_with_accel
};

extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern GAR_MODULE_SETTINGS GAR_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;

#endif
