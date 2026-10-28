#ifndef S_ALGO_H
#define S_ALGO_H

#include <stdint.h>
#include <Arduino.h> // abs() for any type, like UN52 (avr-libc abs() is 16 bit int)
#include "common_structs.h"
#include "nvs/eeprom_config.h"
#include "nvs/module_settings.h"
#include "pressure_manager.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "torque_converter.h"
#include "shift_flags.h"

/*
    Copied from UN52 shifting_algo/ (s_algo, shift_crossover, shift_release, shifting_algo_helpers).

    MinTCU changes:
    - int is 16 bit on the Mega2560 (32 bit on the ESP32 UN52 runs on), so every int is int32_t here,
      and calculations on 16 bit values that can go past 16 bits are done in int32_t or float.
    - No ESP_LOGx (UN52 logs over USB), so they are left out.
    - No adaptation (adaptation_mgr is always nullptr) and no TCC control (always open).
*/
#define ESP_LOGI(...)
#define ESP_LOGW(...)
#define ESP_LOGE(...)

struct TorqueRequstData {
    TorqueRequestControlType ty;
    TorqueRequestBounds bounds;
    float amount;
};

const uint8_t STEP_RES_CONTINUE = 0;
const uint8_t STEP_RES_NEXT = 0xFD;
const uint8_t STEP_RES_FAILURE = 0xFE; // Shift failed. Abort!!
const uint8_t STEP_RES_END_SHIFT = 0xFF;

enum class ShiftStyle {
    Crossover_Up,
    Crossover_Dn,
    Release_Up,
    Release_Dn,
};

struct ShiftInterfaceData {
    AbstractProfile* profile;
    int32_t MOD_MAX;
    int32_t SPC_MAX;
    uint32_t shift_flags;
    GearChange change;
    Clutch applying;
    Clutch releasing;
    GearboxGear curr_g;
    GearboxGear targ_g;
    CircuitInfo inf;
    uint16_t release_spring_on_clutch;
    uint16_t release_spring_off_clutch;
    PrefillData prefill_info;
    ShiftCharacteristics chars;
    ShiftClutchData* ptr_r_clutch_speeds;
    ShiftPressures* ptr_w_pressures;
    TorqueRequstData* ptr_w_trq_req;
    TorqueConverter* tcc;
    ShiftAdaptationSystem* adaptation_mgr;
    bool manual_shift;
    bool trq_req_en;
    float diff_ratio;
    uint16_t max_gb_trq;
    float ratio_old_gear;
    uint16_t n3_rpm;
};

class ShiftingAlgorithm {
public:
    ShiftingAlgorithm(ShiftInterfaceData* data);
    virtual ~ShiftingAlgorithm() = default;
    ShiftAlgoFeedback get_diag_feedback(uint8_t phase_id);

    /**
     * SPECIAL RETURN VALUES
     * STEP_RES_CONTINUE - Continue in the same phase
     * STEP_RES_END_SHIFT - Finish shift
     * STEP_RES_NEXT - Goto next phase
     * 1..253 - The next phase ID to run
     */
    virtual uint8_t step_internal(
        bool stationary,
        bool is_upshift
    ) = 0;

    uint8_t step(
        uint8_t phase_id,
        uint16_t abs_input_torque,
        bool stationary,
        bool is_upshift,
        uint16_t phase_elapsed,
        uint16_t total_elapsed,
        PressureManager* pm,
        SensorData* sd
    );

    void reset_all_subphase_data();
    virtual uint8_t max_shift_stage_id() = 0;

protected:
    bool upshifting = false;
    ShiftInterfaceData* sid;
    uint8_t subphase_mod = 0;
    uint16_t timer_mod = 0;
    uint8_t subphase_shift = 0;
    uint16_t timer_shift = 0;
    int16_t timer_emergency = -1;

    // EGS COMPATIBILITY
    int32_t centrifugal_force_on_clutch;
    int32_t centrifugal_force_off_clutch;

    // EGS compatibility vars (Makes it easier to translate original EGS assembly)
    int32_t p_apply_clutch = 0;
    // used for diagnostics
    int32_t trq_at_apply_clutch = 0;
    int32_t trq_at_release_clutch = 0;
    int32_t mpc_trq_reducer = 0;
    int32_t trq_adder = 0;
    uint16_t abs_input_trq;
    uint16_t abs_load_percentage;
    uint16_t input_rpm_trq_adapt_map;

    int32_t mod_sol_pressure;
    int32_t shift_sol_pressure;
    uint8_t phase_id;

    PressureManager* pm;
    SensorData* sd;

    short momentum_ctrl = 0;
    short momentum_ctrl_filtered = 0;
    short target_turbine_speed = 0;
    short momentum_start_output_rpm = 0;
    short correction_trq = 0;

    uint16_t filling_torque = 0;
    float freeing_trq = 0;

    uint16_t emergency_trq_val = 0;

    // Because EGS is weird, bleed and end of ctrl phases have same for either shift
    uint8_t phase_bleed(PressureManager* pm);
    uint8_t phase_maxp(SensorData* sd);
    uint8_t phase_end_ctrl();


    // EGS compatibility functions
    uint16_t calc_max_trq_on_clutch(uint16_t pressure, CoefficientTy coef);
    uint16_t calc_mod_min_abs_trq(int32_t p_shift);
    uint16_t calc_mod_with_filling_trq_and_freewheeling(int32_t p_shift);
    uint16_t calc_mod_with_filling_trq(int32_t p_shift);
    uint16_t calc_mpc_sol_shift_ps(int32_t p_shift, int32_t p_mod);
    void reset_for_next_phase();
    uint16_t correct_shift_shift_pressure(int16_t pressure);

    uint16_t set_p_apply_clutch_with_spring(int32_t p);

    short calc_correction_trq(ShiftStyle style, short momentum);
    int32_t momentum_pid[3] = { 0,0,0 };
    virtual uint16_t max_p_mod_pressure() = 0;
    virtual bool is_release_shift() = 0;
    uint16_t threshold_rpm = 0;
    uint16_t torque_req_val = 0;

    uint16_t calc_high_filling_p();
    uint16_t calc_low_filling_p();

    void adaptation_step();
    uint8_t adapt_p_map_idx();
    void mark_end_torque_adapt();

    bool trq_req_up_ramp = false;
    uint16_t torque_req_out = 0;
    bool trq_req_down_ramp = false;
    uint8_t trq_req_timer = 0;

    bool adaptation_conditions_ok = true;

    bool do_fill_time_adaptation = false;
    bool do_fill_pressure_adaptation = false;
    bool do_torque_adaptation = false;
    bool end_of_fill_time_adapt = false;

    uint8_t timer_p_adapt = 0;
    uint16_t adapting_trq_limit = 0;
    uint16_t adapting_turbine_spd = 0;
    int32_t adapting_p_adapt_trq = 0;
    int32_t adapting_p_adapt_pressure = 0;

    short rpm_adapt_off_clutch = 0;

    uint8_t fill_time_adaptation_stage = 0;
    uint8_t fill_pressure_adaptation_stage = 0;
    uint8_t torque_adaptation_stage = 0;
    uint16_t old_engine_rpm = 0;

    // Multiplied by 10x
    int32_t first_order_pump_trq_filter = 0;

    int32_t pid_sum = 0;
    uint16_t pid_count = 0;

    int16_t old_input_trq = 0;

    uint32_t trq_adp_load_percentage_sum = 0;
    int32_t trq_adapt_rpm = 0;
    int32_t trq_adapt_engine_load_percent = 0;
    bool trq_adapt_below_min_trq = false;

    bool first_run = true;
    bool torque_jumped = false;
    bool trq_adapt_completed = false;

    int32_t release_minimum_mod_reduction_trq = 0;
    bool n3_moving = false;
    float loss_torque = 0;
    int32_t pid_err_i = 0;
};

// Helper functions
namespace ShiftHelpers {
    float get_shift_intertia(uint8_t shift_idx);
    void calc_shift_flags(ShiftInterfaceData* sid, SensorData* sd, bool bleed_phase);

    uint8_t cycles_crossover_overlap(GearChange change, uint16_t abs_input_torque, uint16_t input_rpm);
    uint8_t cycles_crossover_overlap2(GearChange change, uint16_t abs_input_torque, uint16_t input_rpm);
    uint16_t total_time_crossover_shift(PressureManager* pm, ShiftAdaptationSystem* adp, GearChange change, uint16_t abs_input_torque, uint16_t input_rpm);
    uint16_t correct_shift_shift_pressure(PressureManager* pm, int16_t pressure, uint8_t map_idx);
}

#endif
