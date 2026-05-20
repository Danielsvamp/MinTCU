
// Nu skaja faktist fösök få na ådentli byrjan
// Ja ska ha MPC ti arbeit från kalibrering å sensore

struct ShiftInterfaceData {
    int MOD_MAX;
    int SPC_MAX;
    GearboxGear curr_g;
    GearboxGear targ_g;
};

protected:
    ShiftInterfaceData* sid;
};

uint8_t gear_to_idx_lookup(GearboxGear g) {
    uint8_t gear_idx = 0;
    switch(g) {
        case GearboxGear::First:
            gear_idx = 1;
            break;
        case GearboxGear::Second:
            gear_idx = 2;
            break;
        case GearboxGear::Third:
            gear_idx = 3;
            break;
        case GearboxGear::Fourth:
            gear_idx = 4;
            break;
        case GearboxGear::Fifth:
            gear_idx = 5;
            break;
        case GearboxGear::Reverse_First:
            gear_idx = 6;
            break;
        case GearboxGear::Reverse_Second:
            gear_idx = 7;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvailable:
        default:
            gear_idx = 0;
            break;
    }
    return gear_idx;
}

enum class Clutch {
    K1 = 0,
    K2 = 1,
    K3 = 2,
    B1 = 3,
    B2 = 4,
    B3 = 5
};

uint16_t PressureManager::p_clutch_with_coef(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm) {
    uint8_t gear_idx = gear_to_idx_lookup(gear);
    
    float coef;
    coef = 1.F;
    
    float friction_val = MECH_PTR->friction_map[(gear_idx*6)+(uint8_t)clutch];
    float calc = ((float)abs_torque_nm * friction_val) / coef;
    return calc;
}


// Example call
p_clutch_with_coef(sid->targ_g, sid->applying, this->abs_input_trq)




