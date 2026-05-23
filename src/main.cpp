#include <Arduino.h>
#include <stdint.h>
#undef B1

// Nu skaja faktist fösök få na ådentli byrjan
// Ja ska ha MPC ti arbeit från kalibrering å sensore
// Allt ska lånas från UN52 så langt he ba gar, fö he funkkar

//v Calibration v

enum class clutchCoefType {
    Static,
    Release,
    Sliding
};

//Per clutch friction map
uint16_t clutchFrictionKF[48] = {
    //6, 8,

    //K1, K2, K3, B1, B2, B3,
    
    //0, 1, 2, 3, 4, 5, -1, -2

    4709,	0,	    0,	    3574,	0,  	0,      // 0
    0,	    0,	    3076,	2303,	2685,	0,      // 1
    1845,	0,	    1871,	0,	    1633,	0,      // 2
    0,	    1101,	0,	    0,	    1109,	0,      // 3
    958,	1673,	971,	0,	    0,  	0,      // 4
    0,	    1390,	807,	604,	0,	    0,      // 5
    0,	    0,	    3076,	2303,	0,	    3387,   // -1
    1845,	0,	    1871,	0,	    0,	    2060,   // -2
};

uint16_t pcsKF[48] = { // Pressure in mbar, ATF Temp in C, Current in mA(?)
    //7, 4,

    //50, 600, 1000, 2350, 5600, 6600, 7700,
    
    //25, 70, 110, 200, // These are raw values, true temperature is offset by -50

    1100,	1085,	954,	700,	450,	350,	200,
    1077,	925,	830,	675,	415,	320,	0,
    1000,	835,	780,	650,	400,	288,	0,
    975,	795,	745,	625,	370,	260,	0,
};

enum class GearboxGear: uint8_t {
    First = 1,
    Second = 2,
    Third = 3,
    Fourth = 4,
    Fifth = 5,
    Park = 8,
    Neutral = 9,
    ReverseFirst = 10,
    SignalNotAvailable = 0xFF
};

uint8_t getGearId(GearboxGear g) {
    uint8_t gearId = 0;
    switch(g) {
        case GearboxGear::First:
            gearId = 1;
            break;
        case GearboxGear::Second:
            gearId = 2;
            break;
        case GearboxGear::Third:
            gearId = 3;
            break;
        case GearboxGear::Fourth:
            gearId = 4;
            break;
        case GearboxGear::Fifth:
            gearId = 5;
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
    // Friction coefficient for applying clutches (Sliding into place)
    uint8_t sliding_coefficient() const;
    // Friction coefficient for releasing clutches (Releasing away)
    uint8_t release_coefficient() const;
    // Friction coefficient for static clutches (Held in place)
    uint8_t stationary_coefficient() const;
    
    uint32_t pFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque, clutchCoefType coefType);
    

    // Returns the estimated PWM to send to either SPC or MPC solenoid
    // Based on the requested pressure that is needed withint either pressure rail.
    uint16_t getSolenoidCurrent(uint16_t request_mbar) const;
};

uint8_t PressureManager::sliding_coefficient() const {
    return 140;
}
uint8_t PressureManager::release_coefficient() const {
    return 120;
}
uint8_t PressureManager::stationary_coefficient() const {
    return 100;
}

uint32_t PressureManager::pFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque, clutchCoefType coefType) {
    uint8_t gearId = getGearId(gear);
    
    uint8_t coef;
    switch (coefType) {
        case clutchCoefType::Static:
            coef = this->stationary_coefficient();
            break;
        case clutchCoefType::Sliding:
            coef = this->sliding_coefficient();
            break;
        case clutchCoefType::Release:
            coef = this->release_coefficient();
            break;
        default:
            coef = 1;
    }

    uint16_t friction = clutchFrictionKF[(gearId * 6) + (uint8_t)clutch];
    uint32_t calc = ((uint32_t)torque * (uint32_t)friction) / coef;
    return (uint16_t)calc;
}

uint16_t PressureManager::getSolenoidCurrent(uint16_t request_mbar) const {
    if (this->pressure_pwm_map == nullptr) {
        return 0; // 10% (Failsafe)
    }
    return this->pressure_pwm_map->get_value(request_mbar, this->sensor_data->atf_temp);
}













// v Arduino testing v

void setup() {
    Serial.begin(115200);
}

PressureManager pm;
uint16_t torque = 0;
GearboxGear gear = GearboxGear::Second;
Clutch clutch = Clutch::K3;

void loop_(){
    delay(500);

    if (torque >= 1000) {
        torque = 0;
    }   else {
        torque += 50;
    }
    // "p_clutch_with_coef" call
    //uint16_t pressure = pm.pFromTorque(gear, clutch, torque, clutchCoefType::Static);
    //Serial.print("Torque: ");
    //Serial.print(torque);
    //Serial.print(" Nm | Pressure: ");
    //Serial.print(pressure);
    //Serial.print(" | Clutch friction value: ");
    //Serial.print(clutchFrictionKF[(getGearId(gear) * 6) + (uint8_t)clutch]);
    //Serial.println();

    // "get_p_solenoid_current" call
    uint16_t current = pm.getSolenoidCurrent() const;
}








