#ifndef __MODULE_SETTINGS_H
#define __MODULE_SETTINGS_H

#include <stdint.h>
#include <tcu_maths.h>

/*
    Slimmed down copy of UN52's module settings. Only what MinTCU uses.
    No EEPROM yet, these are compiled in. SBS, ADP, REL and CRS are copied verbatim from UN52.
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

// Copied verbatim from UN52 nvs/module_settings.h
// Shift program basic settings
typedef struct {
    // DEBUG - Show an 'F' marker in the gear display when the TCU
    // detects a flare condition
    bool f_shown_if_flare;
    // DEBUG - Show '^' or 'v' in the gear display when the shift
    // thread is active
    bool debug_show_up_down_arrows_in_r;
    // Enable torque requests for 1-2
    bool en_trq_req_1_2;
    // Enable torque requests for 2-3
    bool en_trq_req_2_3;
    // Enable torque requests for 3-4
    bool en_trq_req_3_4;
    // Enable torque requests for 4-5
    bool en_trq_req_4_5;
    // Enable torque requests for 2-1
    bool en_trq_req_2_1;
    // Enable torque requests for 3-2
    bool en_trq_req_3_2;
    // Enable torque requests for 4-3
    bool en_trq_req_4_3;
    // Enable torque requests for 5-4
    bool en_trq_req_5_4;
    // When in an automatic profile, this limit determines the offset from
    // Redline RPM before an upshift is forced, in order to stop the engine
    // actually hitting redline. If your engine stays at the limiter and after
    // a while the car finally upshifts, increase this threshold
    uint16_t redline_offset_auto_upshift;
    // If the gearbox upshifts while the kickdown switch is still held (Redline
    // upshift, or a manual upshift), kickdown will not request another downshift
    // until the lower gear would land at least this far below redline RPM, or until
    // the kickdown switch is released. This stops the gearbox bouncing between gears
    // at redline whilst still allowing kickdown again if the car slows down (EG: Uphill).
    //
    // UNIT: RPM
    uint16_t kickdown_rearm_rpm_offset;
} __attribute__ ((packed)) SBS_MODULE_SETTINGS;

const SBS_MODULE_SETTINGS SBS_DEFAULT_SETTINGS = {
    .f_shown_if_flare = false,
    .debug_show_up_down_arrows_in_r = false,
    .en_trq_req_1_2 = true,
    .en_trq_req_2_3 = true,
    .en_trq_req_3_4 = true,
    .en_trq_req_4_5 = true,

    .en_trq_req_2_1 = true,
    .en_trq_req_3_2 = true,
    .en_trq_req_4_3 = true,
    .en_trq_req_5_4 = true,
    .redline_offset_auto_upshift = 100,
    .kickdown_rearm_rpm_offset = 500

};

// Adaptation settings
typedef struct {
    // Minimum transmission oil temperature for adaptation
    //
    // UNIT: degrees C
    int16_t min_atf_temp;
    // Maximum transmission oil temperature for adaptation
    //
    // UNIT: degrees C
    int16_t max_atf_temp;
    // Minimum input speed for adaptation
    //
    // UNIT: RPM
    uint16_t min_input_rpm;
    // Maximum input speed for adaptation
    //
    // UNIT: RPM
    uint16_t max_input_rpm;
    // Prefill time/pressure adapt allowed for the K1 clutch
    bool prefill_adapt_k1;
    // Prefill time/pressure adapt allowed for the K2 clutch
    bool prefill_adapt_k2;
    // Prefill time/pressure adapt allowed for the K3 clutch
    bool prefill_adapt_k3;
    // Prefill time/pressure adapt allowed for the B1 brake
    bool prefill_adapt_b1;
    // Prefill time/pressure adapt allowed for the B2 brake
    bool prefill_adapt_b2;
    // The max pressure delta (+/-) allowed for prefill adaptation
    //
    // UNIT: mBar
    uint16_t prefill_max_pressure_delta;
    // The max time delta (+/-) allowed for prefill adaptation
    //
    // UNIT: milliseconds
    uint16_t prefill_max_time_delta;
    // Torque adaptation allowed for 1-2
    bool adapt_trq_1_2;
    // Torque adaptation allowed for 2-3
    bool adapt_trq_2_3;
    // Torque adaptation allowed for 3-4
    bool adapt_trq_3_4;
    // Torque adaptation allowed for 4-5
    bool adapt_trq_4_5;
    // Torque adaptation allowed for 2-1
    bool adapt_trq_2_1;
    // Torque adaptation allowed for 3-2
    bool adapt_trq_3_2;
    // Torque adaptation allowed for 4-3
    bool adapt_trq_4_3;
    // Torque adaptation allowed for 5-4
    bool adapt_trq_5_4;
    // Enable adaptation when manual shifting.
    // It is disabled by default as this can
    // cause shift latency
    bool adaptation_when_manual_shifting;

} __attribute__ ((packed)) ADP_MODULE_SETTINGS;

const ADP_MODULE_SETTINGS ADP_DEFAULT_SETTINGS = {
    .min_atf_temp = 60,
    .max_atf_temp = 110,
    .min_input_rpm = 1000,
    .max_input_rpm = 3000,
    .prefill_adapt_k1 = true,
    .prefill_adapt_k2 = true,
    .prefill_adapt_k3 = true,
    .prefill_adapt_b1 = true,
    .prefill_adapt_b2 = true,
    .prefill_max_pressure_delta = 200,
    .prefill_max_time_delta = 200,
    .adapt_trq_1_2 = false,
    .adapt_trq_2_3 = false,
    .adapt_trq_3_4 = false,
    .adapt_trq_4_5 = false,
    .adapt_trq_2_1 = false,
    .adapt_trq_3_2 = false,
    .adapt_trq_4_3 = false,
    .adapt_trq_5_4 = false,
    .adaptation_when_manual_shifting = false,
};


// Release shift settings
typedef struct {
    // Past this output shaft RPM, torque requests
    // will not be activated when upshifting
    uint16_t output_rpm_disable_trq_req;
    // Below this RPM, a clutch will be considered 'stationary'
    // which triggers the clutch syncronization phases
    // UNIT: RPM
    uint16_t clutch_stationary_rpm;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_upshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_upshift;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_downshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_downshift;
    // Mapping of pedal position to off clutch torque ramp release speed
    //
    // 'raw' values are pedal position (0-250 = 0-100%), 'new' values
    // are the output, in Nm/20ms reduction
    LinearInterpSetting torque_loss_speed_pedal_pos;
    // SPC ramp speed
    // UNIT: mBar/20ms
    uint8_t spc_ramp_speed;
    // SPC ramp multiplier in 'Manual' mode
    float spc_ramp_multi_m;
    // SPC ramp multiplier in 'Race' mode
    float spc_ramp_multi_r;
    // SPC offset based on input RPM. If your shifts are sliggish at higher
    // RPMs, then you can increase the output pressure here
    LinearInterpSetting adder_spc_rpm;
    // SPC offset based on pedal position for Race mode. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal_r;
    // SPC offset based on pedal position for Manual mode. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal_m;
    // SPC offset based on pedal position for all other modes. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal;
} __attribute__ ((packed)) REL_MODULE_SETTINGS;

const REL_MODULE_SETTINGS REL_DEFAULT_SETTINGS = {
    .output_rpm_disable_trq_req = 1500,
    .clutch_stationary_rpm = 130,
    .pid_p_val_upshift = -150,
    .pid_i_val_upshift = -5,
    .pid_p_val_downshift = 200,
    .pid_i_val_downshift = 5,
    .torque_loss_speed_pedal_pos = {
        .new_min = 0.1,
        .new_max = 0.5,
        .raw_min = 10,
        .raw_max = 250,
    },
    .spc_ramp_speed = 8,
    .spc_ramp_multi_m = 1.5,
    .spc_ramp_multi_r = 2.0,
    .adder_spc_rpm = {
        .new_min = 0,
        .new_max = 100,
        .raw_min = 2500,
        .raw_max = 5000,
    },
    .adder_spc_pedal_r = {
        .new_min = 0,
        .new_max = 500,
        .raw_min = 10,
        .raw_max = 250,
    },
    .adder_spc_pedal_m = {
        .new_min = 0,
        .new_max = 250,
        .raw_min = 10,
        .raw_max = 250,
    },
    .adder_spc_pedal = {
        .new_min = 0,
        .new_max = 100,
        .raw_min = 10,
        .raw_max = 250,
    }
};

// Crossover shift settings
typedef struct {
    // Below this RPM, a clutch will be considered 'stationary'
    // which triggers the clutch syncronization phases
    // UNIT: RPM
    uint16_t clutch_stationary_rpm;
    // Number of 20ms cycles for the overlap phase when at low torque (<= 2x Drag torque)
    // UNIT: cycles
    uint8_t overlap_cycles_low_trq;
    // Number of 20ms cycles for the overlap phase when at high torque (>= 10x Drag torque)
    // UNIT: cycles
    uint8_t overlap_cycles_high_trq;
    // Adder to overlap_cycles_low_trq for 1-2 at low torque
    // UNIT: cycles
    uint8_t overlap_cycles_low_trq_adder_1_2;
    // Adder to overlap_cycles_high_trq for 1-2 at low torque
    // UNIT: cycles
    uint8_t overlap_cycles_high_trq_adder_1_2;
    // Adder to overlap_cycles based on RPM. Increasing the output minimum can help
    // with harsh shifting at lower RPMs
    LinearInterpSetting overlap_cycles_adder_rpm;
    // Multiplier for the overlap cycles based on target shift speed
    LinearInterpSetting overlap_multi_shift_speed;
    // Torque adder factor for upshifting in normal profiles
    float adder_trq_multi_normal_up;
    // Torque adder factor for upshifting in manual profile
    float adder_trq_multi_manual_up;
    // Torque adder factor for upshifting in race profile
    float adder_trq_multi_race_up;
    // Torque adder factor for downshifting in normal profiles
    float adder_trq_multi_normal_dn;
    // Torque adder factor for downshifting in manual profile
    float adder_trq_multi_manual_dn;
    // Torque adder factor for downshifting in race profile
    float adder_trq_multi_race_dn;
    // Number of 20ms cycles for the torque sync phase when at low torque (<= 2x Drag torque)
    // UNIT: cycles
    uint8_t sync_cycles_low_trq;
    // Number of 20ms cycles for the torque sync phase when at high torque (>= 10x Drag torque)
    // UNIT: cycles
    uint8_t sync_cycles_high_trq;
    // Adder to sync_cycles_low_trq for 1-2 at low torque
    // UNIT: cycles
    uint8_t sync_cycles_low_trq_adder_1_2;
    // Adder to sync_cycles_high_trq for 1-2 at low torque
    // UNIT: cycles
    uint8_t sync_cycles_high_trq_adder_1_2;
    // Adder to sync_cycles based on RPM. Increasing the output minimum can help
    // with harsh shifting at lower RPMs
    LinearInterpSetting sync_cycles_adder_rpm;
    // Multiplier for the output sync cycles based on target shift speed
    LinearInterpSetting sync_multi_shift_speed;
    // Torque sync torque adder ramp value based on shift speed (Output is in Nm/20ms)
    LinearInterpSetting sync_trq_adder_speed;

    // Torque request multiplier based on pedal position (100% = 250)
    LinearInterpSetting trq_req_multi_pedal_pos;
    // Torque request multiplier based on input RPM
    LinearInterpSetting trq_req_multi_input_rpm;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_upshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_upshift;
    // Clutch inertia control PID algorithm 'D' value (downshifts)
    int16_t pid_d_val_upshift;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_downshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_downshift;
    // Clutch inertia control PID algorithm 'D' value (downshifts)
    int16_t pid_d_val_downshift;
} __attribute__ ((packed)) CRS_MODULE_SETTINGS;

const CRS_MODULE_SETTINGS CRS_DEFAULT_SETTINGS = {
    .clutch_stationary_rpm = 130,
    .overlap_cycles_low_trq = 10,
    .overlap_cycles_high_trq = 6,
    .overlap_cycles_low_trq_adder_1_2 = 3,
    .overlap_cycles_high_trq_adder_1_2 = 2,
    .overlap_cycles_adder_rpm = LinearInterpSetting {
        .new_min = 3,
        .new_max = 1,
        .raw_min = 1000,
        .raw_max = 4000
    },
    .overlap_multi_shift_speed = LinearInterpSetting {
        .new_min = 0.5,
        .new_max = 1.0,
        .raw_min = 100,
        .raw_max = 750
    },

    .adder_trq_multi_normal_up = 1.0,
    .adder_trq_multi_manual_up = 1.5,
    .adder_trq_multi_race_up = 2.0,
    .adder_trq_multi_normal_dn = 1.0,
    .adder_trq_multi_manual_dn = 1.5,
    .adder_trq_multi_race_dn = 2.0,

    .sync_cycles_low_trq = 10,
    .sync_cycles_high_trq = 7,
    .sync_cycles_low_trq_adder_1_2 = 2,
    .sync_cycles_high_trq_adder_1_2 = 1,
    .sync_cycles_adder_rpm = LinearInterpSetting {
        .new_min = 0,
        .new_max = 2,
        .raw_min = 1000,
        .raw_max = 4000
    },
    .sync_multi_shift_speed = LinearInterpSetting {
        .new_min = 0.5,
        .new_max = 1.0,
        .raw_min = 100,
        .raw_max = 750
    },
    .sync_trq_adder_speed = LinearInterpSetting {
        .new_min = 0.25,
        .new_max = 1.0,
        .raw_min = 1000,
        .raw_max = 100
    },

    .trq_req_multi_pedal_pos = LinearInterpSetting {
        .new_min = 0,
        .new_max = 0.5,
        .raw_min = 25,
        .raw_max = 250
    },
    .trq_req_multi_input_rpm = LinearInterpSetting {
        .new_min = 1.0,
        .new_max = 1.5,
        .raw_min = 1500,
        .raw_max = 6000
    },
    .pid_p_val_upshift = 150,
    .pid_i_val_upshift = 15,
    .pid_d_val_upshift = 30,
    .pid_p_val_downshift = -150,
    .pid_i_val_downshift = -5,
    .pid_d_val_downshift = -15
};


extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern GAR_MODULE_SETTINGS GAR_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;
extern SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS;
extern ADP_MODULE_SETTINGS ADP_CURRENT_SETTINGS;
extern REL_MODULE_SETTINGS REL_CURRENT_SETTINGS;
extern CRS_MODULE_SETTINGS CRS_CURRENT_SETTINGS;

#endif
