#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include <tcu_maths.h>
#include <lookupmap.h>
#include "sensors.h"
#include "lookuptable.h"
#include "stored_map.h"
#include <string.h>

typedef struct {
    uint16_t fill_cycles;
    uint16_t fill_pressure_on_clutch;
    uint16_t low_fill_pressure_on_clutch;
} PrefillData;

struct ShiftPressures {
    // At the applying clutch
    float on_clutch;
    // At the releasing clutch
    float off_clutch;
    // At the shift solenoid
    float shift_sol_req;
    // At the modulating solenoid
    float mod_sol_req;
};

enum class CoefficientTy {
    Static,
    Release,
    Sliding
};

class PressureManager {

public:
    void set_target_modulating_pressure(uint16_t targ);
    void set_target_shift_pressure(uint16_t targ);
    /**
     * @brief SPC off (0mA = maximum shift pressure)
     */
    void set_spc_p_max();
    uint16_t get_input_shift_pressure(void) const;
    uint16_t get_corrected_spc_pressure(void) const;
    uint16_t get_max_shift_pressure(uint8_t shift_idx);
    float calculate_centrifugal_force_for_clutch(Clutch clutch, uint16_t input, uint16_t rear_sun);
    /**
     * @brief Switches a shift solenoid (Y3/Y4/Y5) on or off
     */
    void set_shift_circuit(ShiftCircuit ss, bool enable);
    uint8_t get_active_shift_circuits(void) const;

    uint16_t get_max_solenoid_pressure();
    uint16_t get_spring_pressure(Clutch c);
    uint16_t get_calc_line_pressure(void) const;
    uint16_t get_calc_inlet_pressure(void) const;
    uint16_t get_input_modulating_pressure(void) const;
    uint16_t get_corrected_modulating_pressure(void) const;

    uint16_t calc_current_linear_sol(uint16_t p_targ, GearboxGear current_gear, GearChange change_state);

    /**
     * Friction coefficient for applying clutches (Sliding into place)
     */
    float sliding_coefficient() const;
     /**
     * Friction coefficient for releasing clutches (Releasing away)
     */
    float release_coefficient() const;
     /**
     * Friction coefficient for static clutches (Held in place)
     */
    float stationary_coefficient() const;

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    /**
     * @brief Get the shift data object for the requested gear change
     */
    void notify_shift_end();
    CircuitInfo get_basic_shift_data(GearChange shift_request);
    uint16_t p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty);
    int16_t p_clutch_with_coef_signed(GearboxGear gear, Clutch clutch, int16_t abs_torque_nm, CoefficientTy coef_ty);
    uint16_t find_pressure_holding_other_clutches_in_change(GearChange change, GearboxGear current_g, uint16_t abs_torque_nm);
    uint16_t find_freeing_torque(GearChange change, uint16_t motor_torque, uint16_t output_rpm);
    uint16_t find_decent_adder_torque(GearChange change, uint16_t abs_motor_torque, uint16_t output_rpm);
    uint16_t calc_max_torque_for_clutch(GearboxGear gear, Clutch clutch, uint16_t pressure, CoefficientTy coef_val);
    int32_t calc_max_torque_for_clutch_signed(GearboxGear gear, Clutch clutch, int32_t pressure, CoefficientTy coef_val);
    PrefillData make_fill_data(Clutch applying);
    void register_shift_pressure_data(ShiftPressures* p) {
        this->ptr_shift_pressures = p;
    }

    uint16_t find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic = false);

    /**
     * @brief Calculates the solenoid currents and passes them to the solenoids.
     */
    void update_pressures(GearboxGear current_gear, GearChange change_state);

    /**
     * @brief Set the target TCC pressure (Torque converter)
     *
     * @param targ Target TCC pressure to achieve in mBar
     */
    void set_target_tcc_pressure(uint16_t targ);
    uint16_t get_targ_tcc_pressure(void) const;
    /**
     * Returns the estimated PWM to send to the TCC solenoid
     */
    uint16_t get_tcc_solenoid_pwm_duty(uint16_t request_mbar) const;

    /**
     * @brief Failsafe: MPC and SPC at 0mA (maximum pressure) and TCC off while active. Not in UN52.
     */
    void set_failsafe(bool active);
    bool is_failsafe() const;
private:
    SensorData* sensor_data;

    // Shift pressure
    uint16_t target_shift_pressure = 0;
    bool shift_sol_en = false;
    // Modulating pressure
    uint16_t target_modulating_pressure = 0;
    uint16_t corrected_spc_pressure = 0;
    uint16_t corrected_mpc_pressure = 0;

    uint16_t calculated_working_pressure = 0;
    uint16_t calculated_inlet_pressure = 0;

    LookupMap* pressure_pwm_map;
    // TCC pressure
    uint16_t target_tcc_pressure = 0;
    StoredMap* tcc_pwm_map;
    uint16_t gb_max_torque;
    bool mpc_flushing = false;
    uint8_t mpc_flush_timer = 0; // uint8_t like UN52. Note: mpc_no_flush_time (30000) gets truncated to 48 here, same as UN52
    bool failsafe = false;
    uint8_t shift_circuit_flag = 0;
    uint8_t c_gear = 0;
    uint8_t t_gear = 0;
    ShiftPressures* ptr_shift_pressures = nullptr;
    // UN52 StoredMap (NVS backed). MinTCU has no NVS, these always hold UN52's default maps
    LookupAllocMap* fill_time_map;
    LookupAllocMap* fill_pressure_map;
    LookupAllocMap* fill_low_pressure_map;
    // 1-2, 2-3, 3-4, 4-5
    LookupByteMap* momentum_upshifts[4];
    // 2-1, 3-2, 4-3, 5-4
    LookupByteMap* momentum_downshifts[4];
    // 1-2, 2-3, 3-4, 4-5
    LookupByteMap* torque_adder_upshifts[4];
    // 2-1, 3-2, 4-3, 5-4
    LookupByteMap* torque_adder_downshifts[4];
};

extern PressureManager* pressure_manager;

#endif
