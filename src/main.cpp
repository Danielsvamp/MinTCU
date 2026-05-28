#include <Arduino.h>
#include <stdint.h>
#undef B1


// Math things
#ifndef MAX
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

int32_t first_order_filter(uint8_t sample_count, int32_t new_val, int32_t last_val) {
    if (sample_count == 0xFF) {
        sample_count = 0xFE;
    }
    return (new_val + (sample_count*last_val)) / (sample_count + 1);
}

/*
// SHIFT PINS
const int shift23 = 22;
const int shift34 = 23;
const int shift12_45 = 24;

// PWM PINS
const int mpc = 11;
const int spc = 12;
const int tcc = 13;
*/

/*
    Nu skaja faktist fösök få na ådentli byrjan
    Ja ska ha MPC ti arbeit från kalibrering å sensore
    Allt ska lånas från UN52 så langt he ba gar, fö he funkkar
*/

//v Calibration v

enum class clutchCoefType {
    Static,
    Release,
    Sliding
};

// Mechanical calibration

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

uint16_t pcsKFx[7] = { // X axis of pcsKF. Pressure in mbar

    50, 600, 1000, 2350, 5600, 6600, 7700
};

uint16_t pcsKFy[4] = { // Y axis of pcsKF. These are raw values, true temperature is offset by -50

    25, 70, 110, 200
};

uint16_t pcsKFz[28] = { // Z of pcsKF. Current in mA(?)
    //7, 4,

    //50, 600, 1000, 2350, 5600, 6600, 7700
    
    //25, 70, 110, 200

    1100,	1085,	954,	700,	450,	350,	200,
    1077,	925,	830,	675,	415,	320,	0,
    1000,	835,	780,	650,	400,	288,	0,
    975,	795,	745,	625,	370,	260,	0,
};

uint8_t strongestClutchKL[8] = { // Axis - Gears 0, 1, 2, 3, 4, 5, -1, -2
    255, 2, 2, 1, 1, 1, 2, 2
};

uint16_t releaseSpringPressure[6] = { // Axis - Clutches K1-B3
    1270, 846, 1205, 1139, 1289, 488
};

// Hydraulic calibration
uint16_t extraPressureNotShifting = 1000;

uint16_t p_multi_1 = 1000;
uint16_t p_multi_other = 1000;

uint16_t lp_reg_spring_pressure = 1000;

uint16_t min_mpc_pressure = 500;

uint8_t mpc_flush_temp_threshold = 100;
uint16_t mpc_no_flush_time = 1000;
uint16_t mpc_flush_time = 1000;

uint8_t filter_factor;


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

    uint16_t find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic = false);

    uint16_t getMaxPcsPressure();

    uint16_t inputTorque = 0;
    uint16_t atfTemp = 0;

private:
    // Modulating pressure
    uint16_t target_modulating_pressure = 0;

    bool mpc_flushing = false;
    uint8_t mpc_flush_timer = 0;
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



uint16_t PressureManager::getMaxPcsPressure() {
    return pcsKFx[6];
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear curr_g, bool flush_logic) {
    if (flush_logic) {
        if (0 != this->mpc_flush_timer) {
            this->mpc_flush_timer -= 1;
        }
    }
    uint8_t gearId = getGearId(curr_g);
    uint16_t output = 0;
    uint8_t clutchId = strongestClutchKL[gearId];
    if (gearId == 0 || clutchId >= 6) {
        // N,P,SNV
        output = 0;
    } else {   
        float ret = pFromTorque(curr_g, (Clutch)clutchId, abs(inputTorque), clutchCoefType::Static);
        ret += (releaseSpringPressure[clutchId] + extraPressureNotShifting);
        if (curr_g == GearboxGear::First || curr_g == GearboxGear::ReverseFirst) {
            ret *= (p_multi_1 / 1000.0);
        } else {
            ret *= (p_multi_other / 1000.0);
        }
        if (ret < lp_reg_spring_pressure) {
            ret = 0;
        } else {
            ret -= lp_reg_spring_pressure;
        }
        output = ret;
    }
    // Clamping pressures
    if (output > getMaxPcsPressure()) {
        output = getMaxPcsPressure();
    }
    // MPC pressure surge reduction
    // - Reduces the slow buildup of pressure when we are working
    //   below min MPC pressure
    if (
        flush_logic &&
        (this->target_modulating_pressure < min_mpc_pressure) && // Last call was below min
        (0 == output) && // Current call is 0 pressure
        ((atfTemp+50) >= mpc_flush_temp_threshold) && // +50 to convert between our temperature and EGS Cal
        (0 != mpc_no_flush_time)// MPC Flushing is enabled for this box
    ) {
        if (!this->mpc_flushing) {
            if (0 == this->mpc_flush_timer) {
                this->mpc_flushing = true;
                this->mpc_flush_timer = mpc_flush_time;
            }
        } else if (0 == this->mpc_flush_timer) {
            this->mpc_flushing = false;
            this->mpc_flush_timer = mpc_no_flush_time;
        }
    } else {
        this->mpc_flushing = false;
        this->mpc_flush_timer = 0;
    }

    if (false == this->mpc_flushing) {
        output = MAX(output, min_mpc_pressure);
    }
    if (output < this->target_modulating_pressure) {
        // Filter when decreasing pressure, instant rise in pressure
        output = first_order_filter(filter_factor, output, this->target_modulating_pressure);
    }

    return output;
}







// v Arduino testing v

void setup() {
    Serial.begin(115200);
}

PressureManager pm;
uint16_t torque = 0;
GearboxGear gear = GearboxGear::Second;
Clutch clutch = Clutch::K3;



void loop() {

};






