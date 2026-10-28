#include "pressure_manager.h"
#include <stdlib.h> //måst va jär fö ja veit int naa vannifrån abs() kombe från i un52
#include <tcu_maths.h>
#include "solenoids/solenoids.h"
#include "common_structs_ops.h"
#include "nvs/module_settings.h"
#include "egs_calibration/calibration_structs.h"
#include "maps.h"

/*
    Ported from UN52 pressure_manager.cpp (Everything except TCC control).

    NOTE: int is 16 bit on the Mega2560 (32 bit on the ESP32 UN52 runs on), so every
    calculation that can go above 32767 is done in int32_t here.
*/

PressureManager* pressure_manager = nullptr;

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->gb_max_torque = max_torque;

    this->pressure_pwm_map = new LookupRefMap((int16_t*)HYDR_PTR->pcs_map_x, 7, (int16_t*)HYDR_PTR->pcs_map_y, 4, (int16_t*)HYDR_PTR->pcs_map_z, 7*4);

    this->momentum_upshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_1_2_x, 3, SHIFT_ALGO_CFG_PTR->momentum_1_2_y, 2, SHIFT_ALGO_CFG_PTR->momentum_1_2_z, 3*2);
    this->momentum_upshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_2_3_x, 3, SHIFT_ALGO_CFG_PTR->momentum_2_3_y, 2, SHIFT_ALGO_CFG_PTR->momentum_2_3_z, 3*2);
    this->momentum_upshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_3_4_x, 3, SHIFT_ALGO_CFG_PTR->momentum_3_4_y, 2, SHIFT_ALGO_CFG_PTR->momentum_3_4_z, 3*2);
    this->momentum_upshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_4_5_x, 3, SHIFT_ALGO_CFG_PTR->momentum_4_5_y, 2, SHIFT_ALGO_CFG_PTR->momentum_4_5_z, 3*2);
    for (auto i = 0; i < 4; i++) {
        if (!this->momentum_upshifts[i]->is_allocated()) {
            delete this->momentum_upshifts[i];
            this->momentum_upshifts[i] = nullptr;
        }
    }

    this->momentum_downshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_2_1_x, 6, SHIFT_ALGO_CFG_PTR->momentum_2_1_y, 10, SHIFT_ALGO_CFG_PTR->momentum_2_1_z, 6*10);
    this->momentum_downshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_3_2_x, 6, SHIFT_ALGO_CFG_PTR->momentum_3_2_y, 10, SHIFT_ALGO_CFG_PTR->momentum_3_2_z, 6*10);
    this->momentum_downshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_4_3_x, 6, SHIFT_ALGO_CFG_PTR->momentum_4_3_y, 10, SHIFT_ALGO_CFG_PTR->momentum_4_3_z, 6*10);
    this->momentum_downshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->momentum_5_4_x, 6, SHIFT_ALGO_CFG_PTR->momentum_5_4_y, 10, SHIFT_ALGO_CFG_PTR->momentum_5_4_z, 6*10);
    for (auto i = 0; i < 4; i++) {
        if (!this->momentum_downshifts[i]->is_allocated()) {
            delete this->momentum_downshifts[i];
            this->momentum_downshifts[i] = nullptr;
        }
    }

    this->torque_adder_upshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_1_2_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_1_2_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_1_2_z, 8*6);
    this->torque_adder_upshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_2_3_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_2_3_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_2_3_z, 8*6);
    this->torque_adder_upshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_3_4_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_3_4_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_3_4_z, 8*6);
    this->torque_adder_upshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_4_5_x, 6, SHIFT_ALGO_CFG_PTR->trq_adder_4_5_y, 8, SHIFT_ALGO_CFG_PTR->trq_adder_4_5_z, 8*6);
    for (auto i = 0; i < 4; i++) {
        if (!this->torque_adder_upshifts[i]->is_allocated()) {
            delete this->torque_adder_upshifts[i];
            this->torque_adder_upshifts[i] = nullptr;
        }
    }

    this->torque_adder_downshifts[0] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_2_1_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_2_1_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_2_1_z, 3*4);
    this->torque_adder_downshifts[1] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_3_2_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_3_2_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_3_2_z, 3*4);
    this->torque_adder_downshifts[2] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_4_3_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_4_3_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_4_3_z, 3*4);
    this->torque_adder_downshifts[3] = new LookupByteMap(SHIFT_ALGO_CFG_PTR->trq_adder_5_4_x, 3, SHIFT_ALGO_CFG_PTR->trq_adder_5_4_y, 4, SHIFT_ALGO_CFG_PTR->trq_adder_5_4_z, 3*4);
    for (auto i = 0; i < 4; i++) {
        if (!this->torque_adder_downshifts[i]->is_allocated()) {
            delete this->torque_adder_downshifts[i];
            this->torque_adder_downshifts[i] = nullptr;
        }
    }

    /** Pressure fill time map **/
    const int16_t fill_t_x_headers[4] = {-20, 5, 25, 60};
    const int16_t fill_t_y_headers[5] = {
        (int16_t)Clutch::K1,
        (int16_t)Clutch::K2,
        (int16_t)Clutch::K3,
        (int16_t)Clutch::B1,
        (int16_t)Clutch::B2
    };
    this->fill_time_map = new LookupAllocMap(fill_t_x_headers, 4, fill_t_y_headers, 5, LARGE_NAG_FILL_TIME_MAP, FILL_TIME_MAP_SIZE);
    if (!this->fill_time_map->is_allocated()) {
        delete this->fill_time_map;
        this->fill_time_map = nullptr;
    }

    /** Pressure fill pressure map **/
    const int16_t fill_p_x_headers[1] = {1};
    const int16_t fill_p_y_headers[6] = {
        (int16_t)Clutch::K1,
        (int16_t)Clutch::K2,
        (int16_t)Clutch::K3,
        (int16_t)Clutch::B1,
        (int16_t)Clutch::B2,
        (int16_t)Clutch::B3
    };
    this->fill_pressure_map = new LookupAllocMap(fill_p_x_headers, 1, fill_p_y_headers, 6, NAG_FILL_PRESSURE_MAP, FILL_PRESSURE_MAP_SIZE);
    if (!this->fill_pressure_map->is_allocated()) {
        delete this->fill_pressure_map;
        this->fill_pressure_map = nullptr;
    }

    /** Pressure fill pressure map **/
    const int16_t fill_lp_x_headers[1] = {1};
    const int16_t fill_lp_y_headers[5] = {
        (int16_t)Clutch::K1,
        (int16_t)Clutch::K2,
        (int16_t)Clutch::K3,
        (int16_t)Clutch::B1,
        (int16_t)Clutch::B2
    };
    this->fill_low_pressure_map = new LookupAllocMap(fill_lp_x_headers, 1, fill_lp_y_headers, 5, NAG_FILL_LOW_PRESSURE_MAP, LOW_FILL_PRESSURE_MAP_SIZE);
    if (!this->fill_low_pressure_map->is_allocated()) {
        delete this->fill_low_pressure_map;
        this->fill_low_pressure_map = nullptr;
    }

    // Init MPC and SPC req pressures
    this->target_shift_pressure = this->get_max_solenoid_pressure();
    this->target_modulating_pressure = this->get_max_solenoid_pressure();
}

uint16_t PressureManager::calc_current_linear_sol(uint16_t p_targ, GearboxGear current_gear, GearChange change_state) {
    int32_t factor;
    int32_t extra_p = 0;
    if (GearChange::_IDLE == change_state) { // Not shifting
        // Not shifting
        if (GearboxGear::First == current_gear || GearboxGear::Reverse_First == current_gear) {
            factor = HYDR_PTR->p_multi_1;
        } else {
            factor = HYDR_PTR->p_multi_other;
        }
    } else {
        // Shifting
        uint16_t extra_p_interp_max;
        if (GearChange::_1_2 == change_state || GearChange::_2_1 == change_state) {
            factor = HYDR_PTR->p_multi_1;
            extra_p_interp_max = HYDR_PTR->extra_pressure_adder_r1_1;
        } else {
            factor = HYDR_PTR->p_multi_other;
            extra_p_interp_max = HYDR_PTR->extra_pressure_adder_other_gears;
        }
        extra_p = interpolate_float(sensor_data->engine_rpm, 0, extra_p_interp_max, HYDR_PTR->extra_pressure_pump_speed_min, HYDR_PTR->extra_pressure_pump_speed_max, InterpType::Linear);
    }

    int32_t line_pressure = ((int32_t)HYDR_PTR->lp_reg_spring_pressure + (int32_t)this->target_modulating_pressure)*1000L;
    int32_t wp;
    if (factor > 0) {
        wp = extra_p + (line_pressure / factor);
    } else {
        wp = extra_p;
    }
    if (wp <= 0) {
        wp = 0;
    }
    this->calculated_working_pressure = MIN(wp, (int32_t)UINT16_MAX);

    int32_t interpolated = interpolate_float(
        wp,
        HYDR_PTR->inlet_pressure_output_min,
        HYDR_PTR->inlet_pressure_output_max,
        HYDR_PTR->inlet_pressure_input_min,
        HYDR_PTR->inlet_pressure_input_max,
        InterpType::Linear
    );
    this->calculated_inlet_pressure = interpolated;

    int32_t inlet_factor = (int32_t)HYDR_PTR->shift_pressure_addr_percent * ((int32_t)HYDR_PTR->inlet_pressure_output_max - interpolated);
    inlet_factor /= 1000;
    uint16_t output_p = get_max_solenoid_pressure();
    if (p_targ < interpolated) {
        float with_inlet = p_targ + HYDR_PTR->inlet_pressure_offset;
        inlet_factor *= with_inlet;
        inlet_factor /= 1000;
        output_p = MIN((int32_t)p_targ + inlet_factor, (int32_t)UINT16_MAX);
    }
    return output_p;
}

void PressureManager::update_pressures(GearboxGear current_gear, GearChange change_state) {
    if (this->failsafe) {
        // 0mA = maximum pressure on the 722.6, same as an unpowered TCU
        this->corrected_spc_pressure = get_max_solenoid_pressure();
        this->corrected_mpc_pressure = get_max_solenoid_pressure();
        sol_spc->set_current_target(0);
        sol_mpc->set_current_target(0);
        return;
    }
    // -- Set solenoid currents --
    if (this->shift_sol_en) {
        this->corrected_spc_pressure = this->calc_current_linear_sol(this->target_shift_pressure, current_gear, change_state);
        sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp+50.0, 0));
    } else {
        this->corrected_spc_pressure = get_max_solenoid_pressure();
        sol_spc->set_current_target(0);
    }
    this->corrected_mpc_pressure = this->calc_current_linear_sol(this->target_modulating_pressure, current_gear, change_state);
    sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp+50.0, 1));
}

void PressureManager::set_failsafe(bool active) {
    this->failsafe = active;
    if (active) {
        // Shift solenoids off too (Limp: 2nd in D, R)
        this->set_shift_circuit(ShiftCircuit::sc_1_2, false);
        this->set_shift_circuit(ShiftCircuit::sc_2_3, false);
        this->set_shift_circuit(ShiftCircuit::sc_3_4, false);
        this->set_spc_p_max();
        // Next time we leave failsafe, start from max pressure and filter down (find_working_mpc_pressure)
        this->target_modulating_pressure = get_max_solenoid_pressure();
    }
}

bool PressureManager::is_failsafe() const {
    return this->failsafe;
}

uint16_t PressureManager::p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef;
    switch (coef_ty) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 100.F;
    }
    if (coef <= 0.F) {
        coef = 100.F; // Guard a zeroed PRM setting (coefficients are stored x100)
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    if (gear == GearboxGear::Reverse_Second && clutch == Clutch::B3) {
        // Special logic
        friction_val *= MECH_PTR->friction_map[(2*6)+4];
        friction_val /= MECH_PTR->friction_map[(1*6)+4];
    }

    float calc = ((float)abs_torque_nm * friction_val) / coef;
    return MIN(calc, (float)UINT16_MAX);
}

int16_t PressureManager::p_clutch_with_coef_signed(GearboxGear gear, Clutch clutch, int16_t torque_nm, CoefficientTy coef_ty) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef;
    switch (coef_ty) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 100.F;
    }
    if (coef <= 0.F) {
        coef = 100.F; // Guard a zeroed PRM setting (coefficients are stored x100)
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = ((float)torque_nm * friction_val) / coef;
    return MAX((float)INT16_MIN, MIN(calc, (float)INT16_MAX));
}

// Clutches that are held (Not moving) during shifts
const Clutch HOLDING_CLUTCHES[8][2] = {
    {Clutch::B2, Clutch::K3}, // 1-2 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 2-3 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 3-4 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 4-5 (K2 + K3)

    {Clutch::B2, Clutch::K3}, // 2-1 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 3-2 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 4-3 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 5-4 (K2 + K3)
};

uint16_t PressureManager::find_pressure_holding_other_clutches_in_change(GearChange change, GearboxGear current_g, uint16_t abs_torque_nm) {
    const uint8_t idx = fwd_gearchange_egs_map_lookup_idx(change);
    const Clutch* lookup = HOLDING_CLUTCHES[idx];
    uint16_t max = 0;
    for(int i = 0; i < 2; i++) {
        uint16_t calc = p_clutch_with_coef(current_g, lookup[i], abs_torque_nm, CoefficientTy::Static);
        if (calc > max) {
            max = calc;
        }
    }
    return max;
}

float PressureManager::sliding_coefficient() const {
    return interpolate_float(
        sensor_data->atf_temp,
        PRM_CURRENT_SETTINGS.applying_coefficient_cold,
        PRM_CURRENT_SETTINGS.applying_coefficient_hot,
        29,
        65,
        InterpType::Linear
    );
}

float PressureManager::release_coefficient() const {
    return (float)PRM_CURRENT_SETTINGS.releasing_coefficient;
}

float PressureManager::stationary_coefficient() const {
    return (float)PRM_CURRENT_SETTINGS.stationary_coefficient;
}

uint16_t PressureManager::find_decent_adder_torque(GearChange change, uint16_t abs_motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case GearChange::_1_2:
            map = this->torque_adder_upshifts[0];
            break;
        case GearChange::_2_3:
            map = this->torque_adder_upshifts[1];
            break;
        case GearChange::_3_4:
            map = this->torque_adder_upshifts[2];
            break;
        case GearChange::_4_5:
            map = this->torque_adder_upshifts[3];
            break;
        case GearChange::_2_1:
            map = this->torque_adder_downshifts[0];
            break;
        case GearChange::_3_2:
            map = this->torque_adder_downshifts[1];
            break;
        case GearChange::_4_3:
            map = this->torque_adder_downshifts[2];
            break;
        case GearChange::_5_4:
            map = this->torque_adder_downshifts[3];
            break;
        default:
            break;
    }
    if (nullptr == map) {
        return 0;
    } else {
        uint16_t ret = map->get_value((float)output_rpm/30.0, (float)abs_motor_torque/5.0);
        return ret*5;
    }
}

uint16_t PressureManager::find_freeing_torque(GearChange change, uint16_t motor_torque, uint16_t output_rpm) {
    LookupByteMap* map = nullptr;
    switch(change) {
        case GearChange::_1_2:
            map = this->momentum_upshifts[0];
            break;
        case GearChange::_2_3:
            map = this->momentum_upshifts[1];
            break;
        case GearChange::_3_4:
            map = this->momentum_upshifts[2];
            break;
        case GearChange::_4_5:
            map = this->momentum_upshifts[3];
            break;
        case GearChange::_2_1:
            map = this->momentum_downshifts[0];
            break;
        case GearChange::_3_2:
            map = this->momentum_downshifts[1];
            break;
        case GearChange::_4_3:
            map = this->momentum_downshifts[2];
            break;
        case GearChange::_5_4:
            map = this->momentum_downshifts[3];
            break;
        default:
            break;
    }
    if (nullptr == map) {
        return 0;
    } else {
        uint16_t ret = map->get_value((float)output_rpm/30.0, (float)motor_torque/5.0);
        return ret*5;
    }
}

uint16_t PressureManager::calc_max_torque_for_clutch(GearboxGear gear, Clutch clutch, uint16_t pressure, CoefficientTy coef_val) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef;
    switch (coef_val) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 100.F;
    }
    if (coef <= 0.F) {
        coef = 100.F; // Guard a zeroed PRM setting (coefficients are stored x100)
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    if (gear == GearboxGear::Reverse_Second && clutch == Clutch::B3) {
        // Special logic
        friction_val *= MECH_PTR->friction_map[(2*6)+4];
        friction_val /= MECH_PTR->friction_map[(1*6)+4];
    }
    if (friction_val <= 0.F) {
        return 0; // Clutch is not loaded in this gear - avoid dividing by zero
    }
    float calc = ((float)pressure * coef) / (float)friction_val;
    return MIN(calc, (float)UINT16_MAX);
}

int32_t PressureManager::calc_max_torque_for_clutch_signed(GearboxGear gear, Clutch clutch, int32_t pressure, CoefficientTy coef_val) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    float coef;
    switch (coef_val) {
        case CoefficientTy::Static:
            coef = this->stationary_coefficient();
            break;
        case CoefficientTy::Sliding:
            coef = this->sliding_coefficient();
            break;
        case CoefficientTy::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 100.F;
    }
    if (coef <= 0.F) {
        coef = 100.F; // Guard a zeroed PRM setting (coefficients are stored x100)
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    if (friction_val <= 0.F) {
        return 0; // Clutch is not loaded in this gear - avoid dividing by zero
    }
    float calc =  ((float)pressure * coef) / (float)friction_val;
    return calc;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic) {
    if (flush_logic) {
        if (0 != this->mpc_flush_timer) {
            this->mpc_flush_timer -= 1;
        }
    }
    uint8_t gear_idx = gear_to_idx_lookup(curr_g);
    uint16_t output = 0;
    uint8_t clutch_idx = MECH_PTR->strongest_loaded_clutch_idx[gear_idx];
    if (gear_idx == 0 || clutch_idx >= 6) {
        // N,P,SNV
        output = 0;
    } else {
        float ret = p_clutch_with_coef(curr_g, (Clutch)clutch_idx, abs(sensor_data->input_torque), CoefficientTy::Static);
        ret += (MECH_PTR->release_spring_pressure[clutch_idx] + HYDR_PTR->extra_p_not_shifting);
        if (curr_g == GearboxGear::First || curr_g == GearboxGear::Reverse_First) {
            ret *= (HYDR_PTR->p_multi_1 / 1000.0);
        } else {
            ret *= (HYDR_PTR->p_multi_other / 1000.0);
        }
        if (ret < HYDR_PTR->lp_reg_spring_pressure) {
            ret = 0;
        } else {
            ret -= HYDR_PTR->lp_reg_spring_pressure;
        }
        output = MIN(ret, (float)UINT16_MAX);
    }
    // Clamping pressures
    if (output > get_max_solenoid_pressure()) {
        output = get_max_solenoid_pressure();
    }
    // MPC pressure surge reduction
    // - Reduces the slow buildup of pressure when we are working
    //   below min MPC pressure
    if (
        flush_logic &&
        (this->target_modulating_pressure < HYDR_PTR->min_mpc_pressure) && // Last call was below min
        (0 == output) && // Current call is 0 pressure
        ((sensor_data->atf_temp+50) >= HYDR_PTR->mpc_flush_temp_threshold) && // +50 to convert between our temperature and EGS Cal
        (0 != HYDR_PTR->mpc_no_flush_time)// MPC Flushing is enabled for this box
    ) {
        if (!this->mpc_flushing) {
            if (0 == this->mpc_flush_timer) {
                this->mpc_flushing = true;
                this->mpc_flush_timer = HYDR_PTR->mpc_flush_time;
            }
        } else if (0 == this->mpc_flush_timer) {
            this->mpc_flushing = false;
            this->mpc_flush_timer = HYDR_PTR->mpc_no_flush_time;
        }
    } else {
        this->mpc_flushing = false;
        this->mpc_flush_timer = 0;
    }

    if (false == this->mpc_flushing) {
        output = MAX(output, HYDR_PTR->min_mpc_pressure);
    }
    if (output < this->target_modulating_pressure) {
        // Filter when decreasing pressure, instant rise in pressure
        output = first_order_filter(HYDR_PTR->filter_factor, output, this->target_modulating_pressure);
    }

    return output;
}

uint16_t PressureManager::get_max_solenoid_pressure() {
    return HYDR_PTR->pcs_map_x[6];
}

uint16_t PressureManager::get_spring_pressure(Clutch c) {
    return MECH_PTR->release_spring_pressure[(uint8_t)c];
}

uint16_t PressureManager::get_calc_line_pressure(void) const {
    return this->calculated_working_pressure;
}

uint16_t PressureManager::get_calc_inlet_pressure(void) const {
    return this->calculated_inlet_pressure;
}

uint16_t PressureManager::get_input_modulating_pressure(void) const {
    return this->target_modulating_pressure;
}

uint16_t PressureManager::get_corrected_modulating_pressure(void) const {
    return this->corrected_mpc_pressure;
}

void PressureManager::set_target_modulating_pressure(uint16_t targ) {
    uint16_t max_p = get_max_solenoid_pressure();
    if (targ > max_p) {
        targ = max_p;
    }
    this->target_modulating_pressure = targ;
}

void PressureManager::set_target_shift_pressure(uint16_t targ) {
    uint16_t max_p = get_max_solenoid_pressure();
    if (targ > max_p) {
        targ = max_p;
    }
    this->target_shift_pressure = targ;
    this->shift_sol_en = true;
}

void PressureManager::set_spc_p_max() {
    this->target_shift_pressure = get_max_solenoid_pressure();
    this->shift_sol_en = false;
}

uint16_t PressureManager::get_input_shift_pressure(void) const {
    return this->target_shift_pressure;
}

uint16_t PressureManager::get_corrected_spc_pressure(void) const {
    return this->corrected_spc_pressure;
}

uint16_t PressureManager::get_max_shift_pressure(uint8_t shift_idx) {
    uint32_t max_p = (uint32_t)(this->get_max_solenoid_pressure() - HYDR_PTR->shift_reg_spring_pressure) * HYDR_PTR->shift_spc_gain[shift_idx];
    max_p /= 1000; // shift_spc_gain is *1000;
    return max_p;
}

void PressureManager::notify_shift_end() {
    this->c_gear = 0;
    this->t_gear = 0;
    this->ptr_shift_pressures = nullptr;
}

// TODO pull this from calibration tables
const int C_C_FACTOR[8] = {15, 40, 100, 100, 100, 100, 100, 80};

CircuitInfo PressureManager::get_basic_shift_data(GearChange shift_request) {
    CircuitInfo sd;
    uint8_t lookup_valve_info = fwd_gearchange_egs_map_lookup_idx(shift_request);
    switch (shift_request) {
        case GearChange::_1_2:
            sd.targ_g = 2; sd.curr_g = 1;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        case GearChange::_2_3:
            sd.targ_g = 3; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case GearChange::_3_4:
            sd.targ_g = 4; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case GearChange::_4_5:
            sd.targ_g = 5; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            break;
        case GearChange::_5_4:
            sd.targ_g = 4; sd.curr_g = 5;
            sd.shift_circuit = ShiftCircuit::sc_4_5;
            break;
        case GearChange::_4_3:
            sd.targ_g = 3; sd.curr_g = 4;
            sd.shift_circuit = ShiftCircuit::sc_3_4;
            break;
        case GearChange::_3_2:
            sd.targ_g = 2; sd.curr_g = 3;
            sd.shift_circuit = ShiftCircuit::sc_2_3;
            break;
        case GearChange::_2_1:
            sd.targ_g = 1; sd.curr_g = 2;
            sd.shift_circuit = ShiftCircuit::sc_1_2;
            break;
        default:
            break;
    }
    sd.pressure_multi_mpc_int = HYDR_PTR->overlap_circuit_factor_mpc[lookup_valve_info];
    sd.pressure_multi_spc_int = HYDR_PTR->overlap_circuit_factor_spc[lookup_valve_info];
    sd.mpc_pressure_spring_reduction = HYDR_PTR->overlap_circuit_spring_pressure[lookup_valve_info];
    sd.centrifugal_factor_off_clutch_int = C_C_FACTOR[lookup_valve_info];
    // Shift start notify for pm internal algo
    this->c_gear = sd.curr_g;
    this->t_gear = sd.targ_g;
    return sd;
}

PrefillData PressureManager::make_fill_data(Clutch applying) {
    if (nullptr == this->fill_time_map || nullptr == fill_pressure_map || nullptr == fill_low_pressure_map) {
        return PrefillData {
            .fill_cycles = 25,
            .fill_pressure_on_clutch = 1500,
            .low_fill_pressure_on_clutch = 700,
        };
    } else {
        PrefillData ret =  PrefillData {
            .fill_cycles = (uint16_t)(fill_time_map->get_value(this->sensor_data->atf_temp, (uint8_t)applying)/20.0),
            .fill_pressure_on_clutch = (uint16_t)fill_pressure_map->get_value(1, (uint8_t)applying),
            .low_fill_pressure_on_clutch = (uint16_t)fill_low_pressure_map->get_value(1, (uint8_t)applying),
        };
        return ret;
    }
}

float PressureManager::calculate_centrifugal_force_for_clutch(Clutch clutch, uint16_t input, uint16_t rear_sun) {
    float speed = 0;
    uint8_t sel_idx = 0xFF;
    float ret = 0;
    switch (clutch) {
        case Clutch::K1:
            sel_idx = 0;
            speed = input;
            break;
        case Clutch::K2:
            sel_idx = 1;
            speed = input;
            break;
        case Clutch::K3:
            sel_idx = 2;
            speed = rear_sun;
            break;
        default:
            break;
    }
    if (sel_idx != 0xFF) {
        int32_t clutch_factor = MECH_PTR->atf_density_centrifugal_force_factor[sel_idx];
        if (clutch_factor != 0) {
            float drop = (MECH_PTR->atf_density_drop_per_c * (float)(sensor_data->atf_temp + 50))/100.0;
            float density_now = MECH_PTR->atf_density_minus_50c - drop;
            ret = (((speed*speed)/1000.0)*density_now) / clutch_factor;
            ret /= 10.0; // To convert to mbar
        }
    }
    return ret;
}

void PressureManager::set_shift_circuit(ShiftCircuit ss, bool enable) {
    OnOffSolenoid* manipulated = nullptr;
    if (ShiftCircuit::sc_1_2 == ss) {
        manipulated = sol_y3;
    } else if (ShiftCircuit::sc_2_3 == ss) {
        manipulated = sol_y5;
    } else if (ShiftCircuit::sc_3_4 == ss) {
        manipulated = sol_y4;
    } else if (ShiftCircuit::sc_4_5 == ss) {
        manipulated = sol_y3;
    } else { // No shift circuit (placeholder)
        this->t_gear = 0;
        this->c_gear = 0;
        return;
    }
    // Firstly, check if new value is 0 (Close the solenoid!)
    if (!enable) {
        manipulated->off();
        this->shift_circuit_flag &= ~(uint8_t)ss; //
    } else { // Check if current value is 0, if so, write full PWM
        manipulated->on();
        this->shift_circuit_flag |= (uint8_t)ss; //
    }
}

uint8_t PressureManager::get_active_shift_circuits(void) const {
    return this->shift_circuit_flag;
}
