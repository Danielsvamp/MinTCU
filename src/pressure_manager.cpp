#include "pressure_manager.h"
#include <stdlib.h> //måst va jär fö ja veit int naa vannifrån abs() kombe från i un52
#include <tcu_maths.h>
#include "solenoids/solenoids.h"
#include "common_structs_ops.h"
#include "nvs/module_settings.h"
#include "egs_calibration/calibration_structs.h"

/*
    Ported from UN52 pressure_manager.cpp (MPC hold pressure only, no shifting yet).

    NOTE: int is 16 bit on the Mega2560 (32 bit on the ESP32 UN52 runs on), so every
    calculation that can go above 32767 is done in int32_t here.
*/

PressureManager* pressure_manager = nullptr;

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {
    this->sensor_data = sensor_ptr;
    this->gb_max_torque = max_torque;

    this->pressure_pwm_map = new LookupRefMap((int16_t*)HYDR_PTR->pcs_map_x, 7, (int16_t*)HYDR_PTR->pcs_map_y, 4, (int16_t*)HYDR_PTR->pcs_map_z, 7*4);

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

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic) {
    if (flush_logic) {
        if (0 != this->mpc_flush_timer) {
            this->mpc_flush_timer -= 1;
        }
    }
    uint8_t gear_idx = gear_to_idx_lookup(curr_g);
    uint16_t output = 0;
    // MinTCU uses its own strongestClutchKL / releaseSpringPressureKL tables here,
    // UN52 uses strongest_loaded_clutch_idx / release_spring_pressure
    uint8_t clutch_idx = MECH_PTR->strongestClutchKL[gear_idx];
    if (gear_idx == 0 || clutch_idx >= 6) {
        // N,P,SNV
        output = 0;
    } else {
        float ret = p_clutch_with_coef(curr_g, (Clutch)clutch_idx, abs(sensor_data->input_torque), CoefficientTy::Static);
        ret += (MECH_PTR->releaseSpringPressureKL[clutch_idx] + HYDR_PTR->extra_p_not_shifting);
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
    this->target_modulating_pressure = targ;
}
