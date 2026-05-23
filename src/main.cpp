#include <Arduino.h>
#include <stdint.h>
#undef B1

// Nu skaja faktist fösök få na ådentli byrjan
// Ja ska ha MPC ti arbeit från kalibrering å sensore

//v Calibration v

//Per clutch friction map
uint16_t clutchFrictionKF[48];

enum class GearboxGear: uint8_t {
    Second = 2,
    Park = 8,
    Neutral = 9,
    ReverseFirst = 10,
    SignalNotAvailable = 0xFF
};


uint8_t getGearId(GearboxGear g) {
    uint8_t gearId = 0;
    switch(g) {
        case GearboxGear::Second:
            gearId = 2;
            break;
        case GearboxGear::ReverseFirst:
            gearId = 6;
            break;
        case GearboxGear::Park:
        case GearboxGear::Neutral:
        case GearboxGear::SignalNotAvailable:
        default:
            gearId = 0;
            break;
    }
    return gearId;
}

enum class Clutch {
    K1 = 0,
    K2 = 1,
    K3 = 2,
    B1 = 3,
    B2 = 4,
    B3 = 5
};


class PressureManager {

public: 

    uint16_t pFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque);
    
};
    
uint16_t PressureManager::pFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque) {
    uint8_t gearId = getGearId(gear);
    
    float coef;
    coef = 1.F;
    
    float friction = clutchFrictionKF[(gearId*6)+(uint8_t)clutch];
    float calc = ((float)torque * friction) / coef;
    return calc;
}

void setup() {
    // p_clutch_with_coef call

    PressureManager pm;
    uint16_t torque = 100;

    uint16_t pressure = pm.pFromTorque(GearboxGear::Second, Clutch::K1, torque);
    Serial.print(pressure);

    
}










