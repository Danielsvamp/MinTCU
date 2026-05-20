
// Nu skaja faktist fösök få na ådentli byrjan
// Ja ska ha MPC ti arbeit från kalibrering å sensore

// Calibration
typedef struct {
    uint16_t friction_map[48];
} __attribute__((packed)) calibrationData;

calibrationData* calibrationPointer =  

struct ShiftInterfaceData {
    int MOD_MAX;
    int SPC_MAX;
    GearboxGear curr_g;
};

protected:
    ShiftInterfaceData* sid;
};

uint8_t gear_to_idx_lookup(GearboxGear g) {
    uint8_t gear_idx = 0;
    switch(g) {

        case GearboxGear::Second:
            gear_idx = 2;
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




