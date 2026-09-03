#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include <tcu_maths.h>
#include <lookupmap.h>
//#include "profiles.h"
//#include "adaptation/shift_adaptation.h"
//#include "nvs/eeprom_config.h"
//#include "stored_map.h"
#include "sensors.h"
//#include "nvs/module_settings.h"
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
    uint8_t sliding_coefficient() const;
     /**
     * Friction coefficient for releasing clutches (Releasing away)
     */
    uint8_t release_coefficient() const;
     /**
     * Friction coefficient for static clutches (Held in place)
     */
    uint8_t stationary_coefficient() const;

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    uint16_t p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty);

    uint16_t find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic = false);

    void update_pressures(GearboxGear current_gear, GearChange change_state);
private:

    uint8_t shift_circuit_flag = 0;

    SensorData* sensor_data;
    
    // Shift pressure
    uint16_t target_shift_pressure = 0;
    bool shift_sol_en = false;
    // Modulating pressure
    uint16_t target_modulating_pressure = 0;
    // TCC pressure
    uint16_t corrected_mpc_pressure = 0;

    uint16_t calculated_working_pressure = 0;
    uint16_t calculated_inlet_pressure = 0;

    LookupMap* pressure_pwm_map;
    uint16_t gb_max_torque;
    uint8_t c_gear = 0;
    uint8_t t_gear = 0;
    bool init_ss_recovery = false;
    bool mpc_flushing = false;
    uint8_t mpc_flush_timer = 0;
    uint64_t last_ss_on_time = 0;
};

extern PressureManager* pressure_manager;

#endif