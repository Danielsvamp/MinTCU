#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include <tcu_maths.h>
#include <lookupmap.h>
#include "sensors.h"
#include "lookuptable.h"
#include <string.h>

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

    uint16_t p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty);

    uint16_t find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic = false);

    /**
     * @brief Calculates the solenoid currents and passes them to the solenoids.
     */
    void update_pressures(GearboxGear current_gear, GearChange change_state);

    /**
     * @brief Failsafe: MPC and SPC at 0mA (maximum pressure) while active. Not in UN52.
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
    uint16_t gb_max_torque;
    bool mpc_flushing = false;
    uint8_t mpc_flush_timer = 0; // uint8_t like UN52. Note: mpc_no_flush_time (30000) gets truncated to 48 here, same as UN52
    bool failsafe = false;
    uint8_t shift_circuit_flag = 0;
};

extern PressureManager* pressure_manager;

#endif
