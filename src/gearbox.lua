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
    x_coefficient() = Returns a value defined by calibration data


--]]
















