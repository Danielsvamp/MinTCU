#include "pressure_manager.h"
#include <stdlib.h> //måst va jär fö ja veit int naa vannifrån abs() kombe från i un52
#include <tcu_maths.h>
//#include "solenoids/solenoids.h"
//#include "maps.h"
#include "common_structs_ops.h"
//#include "nvs/module_settings.h"
//#include "nvs/device_mode.h"
//#include "nvs/all_keys.h"
#include "egs_calibration/calibration_structs.h"

uint8_t PressureManager::sliding_coefficient() const {
    return 140;
}
uint8_t PressureManager::release_coefficient() const {
    return 120;
}
uint8_t PressureManager::stationary_coefficient() const {
    return 100;
}

uint16_t PressureManager::get_max_solenoid_pressure() {
    return HYDR_PTR->pcs_map_x[6];
}

uint16_t PressureManager::calc_current_linear_sol(uint16_t p_targ, GearboxGear current_gear, GearChange change_state) {
    int factor;
    uint16_t extra_p = 0;
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

    int line_pressure = ((int)HYDR_PTR->lp_reg_spring_pressure + (int)this->target_modulating_pressure)*1000;
    int wp = extra_p + (line_pressure / factor);
    if (wp <= 0) {
        wp = 0;
    }
    this->calculated_working_pressure = wp;

    int interpolated = interpolate_float(
        wp,
        HYDR_PTR->inlet_pressure_output_min,
        HYDR_PTR->inlet_pressure_output_max,
        HYDR_PTR->inlet_pressure_input_min,
        HYDR_PTR->inlet_pressure_input_max,
        InterpType::Linear
    );
    this->calculated_inlet_pressure = interpolated;

    int inlet_factor = HYDR_PTR->shift_pressure_addr_percent * (HYDR_PTR->inlet_pressure_output_max - interpolated);
    inlet_factor /= 1000;
    uint16_t output_p = get_max_solenoid_pressure();
    if (p_targ < interpolated) {
        float with_inlet = p_targ + HYDR_PTR->inlet_pressure_offset;
        inlet_factor *= with_inlet;
        inlet_factor /= 1000;
        output_p = p_targ + inlet_factor;
    }
    return output_p;
}

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {

    this->target_modulating_pressure = this->get_max_solenoid_pressure();

    this->pressure_pwm_map = new LookupRefMap((int16_t*)HYDR_PTR->pcs_map_x, 7, (int16_t*)HYDR_PTR->pcs_map_y, 4, (int16_t*)HYDR_PTR->pcs_map_z, 7*4);
};

uint16_t PressureManager::p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t torque, CoefficientTy coefType) {
    uint8_t gearId = gear_to_idx_lookup(gear);
    
    uint8_t coef;
    switch (coefType) {
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
            coef = 100;
    }

    uint16_t friction = MECH_PTR->clutchFrictionKF[(gearId * 6) + (uint8_t)clutch];
    uint32_t calc = ((uint32_t)torque * (uint32_t)friction) / coef;
    return (uint16_t)calc;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic) {
    if (flush_logic) {
        if (0 != this->mpc_flush_timer) {
            this->mpc_flush_timer -= 1;
        }
    }
    uint8_t gearId = gear_to_idx_lookup(curr_g);
    uint16_t output = 0;
    uint8_t clutchId = MECH_PTR->strongestClutchKL[gearId];
    if (gearId == 0 || clutchId >= 6) { // N/P
        output = 0;
    } else {
        float ret = p_clutch_with_coef(curr_g, (Clutch)clutchId, abs(sensor_data->input_torque), CoefficientTy::Static);
        ret += (MECH_PTR->releaseSpringPressureKL[clutchId] + HYDR_PTR->extraPressureNotShifting);
        if (curr_g == GearboxGear::First || curr_g == GearboxGear::Reverse_First) {
            ret *= (HYDR_PTR->p_multi_1 / 1000.0);
        } else {
          ret *= (HYDR_PTR->p_multi_other / 1000.0);
        }

        if (ret < HYDR_PTR->lp_reg_spring_pressure) {
            ret = 0;
        } else {
            ret -= (HYDR_PTR->lp_reg_spring_pressure);
        }
        output = ret;
    }
    // Clamping pressures
    if (output > get_max_solenoid_pressure()) {
        output = get_max_solenoid_pressure();
    }
    /*
        MPC pressure surge reduction - Reduces the slow buildup of pressure when we are working below min MPC pressure
    */
    if  (flush_logic &&
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

void PressureManager::update_pressures(GearboxGear current_gear, GearChange change_state) {
    // This is my best guess at interpreting the assembly (Decompiler view messes a lot up with this function due to indirections)

    // -- Set solenoid currents --
    /* Uncomment when shifting is relevant!
    if (this->shift_sol_en) {
        this->corrected_spc_pressure = this->calc_current_linear_sol(this->target_shift_pressure, current_gear, change_state);
        sol_spc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_spc_pressure, sensor_data->atf_temp+50.0));
    } else {
        this->corrected_spc_pressure = getMaxPcsPressure();
        sol_spc->set_current_target(0);
    }
    
    this->corrected_spc_pressure = getMaxPcsPressure();
    sol_spc->set_current_target(0);
    */
   
    this->corrected_mpc_pressure = this->calc_current_linear_sol(this->target_modulating_pressure, current_gear, change_state);
    sol_mpc->set_current_target(this->pressure_pwm_map->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp+50.0));
   
    /* Uncomment when TCC is relevant!
    sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->target_tcc_pressure));
    */
}