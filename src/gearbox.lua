local currentGear = 2
local targetGear = 2



while true do
   
end


-- Crossover Shift start -- For upshifts under positive load and downshifts when coasting

-- PREPARE FOR SHIFT 
-- Phase 0 Bleed: SPC pressure ramps from "SPC_MAX" to "high fill pressure" over time "60 ms", MPC holds releasing clutch
-- Wait for timer

-- Shift solenoid ON here

-- Phase 1.1 Fill 1 (High fill): SPC pressure from "fill pressure map" for time "fill time hold map"
-- Phase 1.2 Fill 2 (Ramp to low fill): SPC ramps pressure to "low fill pressure" over time "low fill time varable (60 ms)"
-- Phase 1.3 Fill 3 (Low fill hold): SPC pressure "low fill pressure map" for time "low fill hold time map (100 ms)"
-- "KISSING POINT"

-- Phase 2 Overlap1: SPC ramps to "torque holding pressure per gear" (or "SPC_MAX"?), MPC reduced (quantity?) for releasing clutch to slip, TCC unlocked for damping
-- Wait for releasing clutch to slip (until? and how much?)
-- Phase 3 Overlap2: I don't know what I should do here, I won't have PID control or torque requesting yet

-- Phase 4 Max pressure: SPC ramps to "SPC_MAX" locks applying clutch
-- Shift solenoid OFF here

-- Phase 5 End control: SPC pressure still "SPC_MAX", MPC ramps to working pressure (MAX)
-- SHIFT COMPLETE

-- Crossover Shift end --


--[[    TODO

    Define "p_clutch_with_coef", Pressure from Torque
    Define parts for above function
    
    GearboxGear = Enum with possible gears, First, ..., Fifth, Reverse_First, Reverse_Second, P, N, SNA
    Clutch = Enum with possible clutches, K1, K2, K3, B1, B2, B3
    CoefficientTy = Enum with possible clutch states, Static, Release, Sliding
    Torque = A torque value in Nm
    gear_to_idx_lookup() = Returns a gear as uint8_t 0-7 (common_structs_ops.h and .cpp)
    x_coefficient() = Returns a float defined by calibration data (?)
    MECH_PTR = Mechanical calibration
    friction_map = Map from EGS calibration data
    
    
    
    
    
    
    Define "find_working_mpc_pressure"
    Define parts for above function
     
    uint16_t find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic = false);
    
    flush_logic = ?
    mpc_flush_timer = ?
    
    gear_idx = uint8_t gear
    strongest_loaded_clutch_idx = 
    
    
    
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
        output = ret;
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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    From UN52's pressure_manager.h:
    
    uint16_t p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, CoefficientTy coef_ty);
    
    GearboxGear = Enum with possible gears, First, ..., Fifth, Reverse_First, Reverse_Second, P, N, SNA
    Clutch = Enum with possible clutches, K1, K2, K3, B1, B2, B3
    CoefficientTy = Enum with possible clutch states, Static, Release, Sliding
    Torque = A torque value in Nm

    From UN52's pressure_manager.cpp:
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
            coef = 1.F;
    }
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = ((float)abs_torque_nm * friction_val) / coef;
    return calc;
}

    gear_to_idx_lookup() = Returns a gear as uint8_t 0-7 (common_structs_ops.h and .cpp)
    x_coefficient() = Returns a float defined by calibration data (?)
    MECH_PTR = Mechanical calibration
    friction_map = Map from EGS calibration data
    


--]]
















