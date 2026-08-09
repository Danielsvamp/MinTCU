/*
    Nu skaja faktist fösök få na ådentli byrjan
    Ja ska ha MPC ti arbeit från kalibrering å sensore
    Allt ska lånas från UN52 så langt he ba gar, fö he funkkar

    Now I'm actually gonna try to get a decent start
    I need MPC to work from calibration and sensors
    Everything must be borrowed from UN52 as far as possible, because that works
*/

#include <Arduino.h>
#include <stdint.h>
#include <common.h>
#include <maplookup.cpp>
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
    My arduino pinout saved here so I don't forget what's what

// SHIFT PINS
const int shift23 = 22;
const int shift34 = 23;
const int shift12_45 = 24;

// PWM PINS
const int mpc = 11;
const int spc = 12;
const int tcc = 13;
*/

//v Calibration v

enum class clutchCoefType {
    Static,
    Release,
    Sliding
};

// Mechanical calibration
struct MechanicalCalibration {
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

    //fix this
    uint16_t tccKF[35] = { // Z of pcsKF. Current in mA(?)
        //7, 4,

        //50, 600, 1000, 2350, 5600, 6600, 7700
        
        //25, 70, 110, 200

        0,	7680,	15360,	20480,	30720,	40960,	64535,
        0,	8960,	16640,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
    };

    // Strongest currently holding clutch per gear
    uint8_t strongestClutchKL[8] = { // Axis - Gears 0, 1, 2, 3, 4, 5, -1, -2
        255, 2, 2, 1, 1, 1, 2, 2
    };

    // Release spring pressure per clutch
    uint16_t releaseSpringPressureKL[6] = { // Axis - Clutches K1-B3
        1270, 846, 1205, 1139, 1289, 488
    };
};


// "Hydraulic calibration" from UN52 for A0305452032 (210.016 OM612 722.634)
struct HydraulicCalibration {
    uint16_t p_multi_1 = 431;
    uint16_t p_multi_other = 592;

    uint16_t lp_reg_spring_pressure = 1828;

    uint16_t min_mpc_pressure = 500;

    uint8_t filter_factor = 15;

    uint8_t mpc_flush_temp_threshold = 75;
    uint16_t mpc_no_flush_time = 30000;
    uint16_t mpc_flush_time = 50;

    uint16_t extraPressureNotShifting = 0;
    uint16_t extra_pressure_pump_speed_min = 0;
    uint16_t extra_pressure_pump_speed_max = 0;
    uint16_t extra_pressure_adder_r1_1 = 0;
    uint16_t extra_pressure_adder_other_gears = 0;
    uint16_t shift_pressure_addr_percent = 0;

    uint16_t inlet_pressure_offset = 0;
    uint16_t inlet_pressure_input_min = 0;
    uint16_t inlet_pressure_input_max = 0;
    uint16_t inlet_pressure_output_min = 0;
    uint16_t inlet_pressure_output_max = 0;

    // Pressure solenoid current map
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

};

// Calibration End




enum class GearboxGear: uint8_t {
    First = 1,
    Second = 2,
    Third = 3,
    Fourth = 4,
    Fifth = 5,
    Park = 8,
    Neutral = 9,
    Reverse_First = 10,
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
        case GearboxGear::Reverse_First:
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

enum class GearChange {
    _IDLE = 0,
    _1_2 = 1,
    _2_3 = 2,
    _3_4 = 3,
    _4_5 = 4,
    _2_1 = 5,
    _3_2 = 6,
    _4_3 = 7,
    _5_4 = 8,
};

class PressureManager {

public:

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    // Friction coefficient for applying clutches (Sliding into place)
    uint8_t sliding_coefficient() const;
    // Friction coefficient for releasing clutches (Releasing away)
    uint8_t release_coefficient() const;
    // Friction coefficient for static clutches (Held in place)
    uint8_t stationary_coefficient() const;
    
    uint16_t pClutchFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque, clutchCoefType coefType);
    

    // Returns the estimated PWM to send to either SPC or MPC solenoid
    // Based on the requested pressure that is needed withint either pressure rail.

    uint16_t find_working_mpc_pressure(GearboxGear currentGear, bool flush_logic = false);
    
    uint16_t getMaxPcsPressure();

    uint16_t calc_current_linear_sol(uint16_t p_targ, GearboxGear current_gear, GearChange change_state);

    void set_target_modulating_pressure(uint16_t targ);

    void update_pressures(GearboxGear current_gear, GearChange change_state);

    uint16_t inputTorque = 0;
    uint16_t atfTemp = 0;

    MechanicalCalibration mechCalib;
    HydraulicCalibration hydrCalib;
private:

    bool mpc_flushing = false;
    uint8_t mpc_flush_timer = 0;

    SensorData* sensor_data;

    // Shift pressure
    uint16_t target_shift_pressure = 0;
    bool shift_sol_en = false;
    // Modulating pressure
    uint16_t target_modulating_pressure = 0;
    // TCC pressure
    uint16_t target_tcc_pressure = 0;
    uint16_t corrected_spc_pressure = 0;
    uint16_t corrected_mpc_pressure = 0;

    uint16_t calculated_working_pressure = 0;
    uint16_t calculated_inlet_pressure = 0;

    LookupMap* pcsKF;

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

uint16_t PressureManager::getMaxPcsPressure() {
    return hydrCalib.pcsKFx[6];
}

uint16_t PressureManager::calc_current_linear_sol(uint16_t p_targ, GearboxGear current_gear, GearChange change_state) {
    int factor;
    uint16_t extra_p = 0;
    if (GearChange::_IDLE == change_state) { // Not shifting
        // Not shifting
        if (GearboxGear::First == current_gear || GearboxGear::Reverse_First == current_gear) {
            factor = hydrCalib.p_multi_1;
        } else {
            factor = hydrCalib.p_multi_other;
        }
    } else {
        // Shifting
        uint16_t extra_p_interp_max;
        if (GearChange::_1_2 == change_state || GearChange::_2_1 == change_state) {
            factor = hydrCalib.p_multi_1;
            extra_p_interp_max = hydrCalib.extra_pressure_adder_r1_1;
        } else {
            factor = hydrCalib.p_multi_other;
            extra_p_interp_max = hydrCalib.extra_pressure_adder_other_gears;
        }
        extra_p = interpolate_float(sensor_data->engine_rpm, 0, extra_p_interp_max, hydrCalib.extra_pressure_pump_speed_min, hydrCalib.extra_pressure_pump_speed_max, InterpType::Linear);
    }

    int line_pressure = ((int)hydrCalib.lp_reg_spring_pressure + (int)this->target_modulating_pressure)*1000;
    int wp = extra_p + (line_pressure / factor);
    if (wp <= 0) {
        wp = 0;
    }
    this->calculated_working_pressure = wp;

    int interpolated = interpolate_float(
        wp,
        hydrCalib.inlet_pressure_output_min,
        hydrCalib.inlet_pressure_output_max,
        hydrCalib.inlet_pressure_input_min,
        hydrCalib.inlet_pressure_input_max,
        InterpType::Linear
    );
    this->calculated_inlet_pressure = interpolated;

    int inlet_factor = hydrCalib.shift_pressure_addr_percent * (hydrCalib.inlet_pressure_output_max - interpolated);
    inlet_factor /= 1000;
    uint16_t output_p = getMaxPcsPressure();
    if (p_targ < interpolated) {
        float with_inlet = p_targ + hydrCalib.inlet_pressure_offset;
        inlet_factor *= with_inlet;
        inlet_factor /= 1000;
        output_p = p_targ + inlet_factor;
    }
    return output_p;
}

PressureManager::PressureManager(SensorData* sensor_ptr, uint16_t max_torque) {

    this->target_modulating_pressure = this->getMaxPcsPressure();

    this->pcsKF = new LookupRefMap((int16_t*)hydrCalib.pcsKFx, 7, (int16_t*)hydrCalib.pcsKFy, 4, (int16_t*)hydrCalib.pcsKFz, 7*4);
};

uint16_t PressureManager::pClutchFromTorque(GearboxGear gear, Clutch clutch, uint16_t torque, clutchCoefType coefType) {
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
            coef = 100;
    }

    uint16_t friction = mechCalib.clutchFrictionKF[(gearId * 6) + (uint8_t)clutch];
    uint32_t calc = ((uint32_t)torque * (uint32_t)friction) / coef;
    return (uint16_t)calc;
}

uint16_t PressureManager::find_working_mpc_pressure(GearboxGear currentGear, bool flush_logic) {
    if (flush_logic) {
        if (0 != this->mpc_flush_timer) {
            this->mpc_flush_timer -= 1;
        }
    }
    uint8_t gearId = getGearId(currentGear);
    uint16_t output = 0;
    uint8_t clutchId = mechCalib.strongestClutchKL[gearId];
    if (gearId == 0 || clutchId >= 6) { // N/P
        output = 0;
    } else {
        float ret = pClutchFromTorque(currentGear, (Clutch)clutchId, abs(inputTorque), clutchCoefType::Static);
        ret += (mechCalib.releaseSpringPressureKL[clutchId] + hydrCalib.extraPressureNotShifting);
        if (currentGear == GearboxGear::First || currentGear == GearboxGear::Reverse_First) {
            ret *= (hydrCalib.p_multi_1 / 1000.0);
        } else {
          ret *= (hydrCalib.p_multi_other / 1000.0);
        }

        if (ret < hydrCalib.lp_reg_spring_pressure) {
            ret = 0;
        } else {
            ret -= (hydrCalib.lp_reg_spring_pressure);
        }
        output = ret;
    }
    // Clamping pressures
    if (output > getMaxPcsPressure()) {
        output = getMaxPcsPressure();
    }
    /*
        MPC pressure surge reduction - Reduces the slow buildup of pressure when we are working below min MPC pressure
    */
    if  (flush_logic &&
        (this->target_modulating_pressure < hydrCalib.min_mpc_pressure) && // Last call was below min
        (0 == output) && // Current call is 0 pressure
        ((atfTemp+50) >= hydrCalib.mpc_flush_temp_threshold) && // +50 to convert between our temperature and EGS Cal
        (0 != hydrCalib.mpc_no_flush_time)// MPC Flushing is enabled for this box
    ) {
        if (!this->mpc_flushing) {
            if (0 == this->mpc_flush_timer) {
                this->mpc_flushing = true;
                this->mpc_flush_timer = hydrCalib.mpc_flush_time;
            }
        } else if (0 == this->mpc_flush_timer) {
            this->mpc_flushing = false;
            this->mpc_flush_timer = hydrCalib.mpc_no_flush_time;
        }
    } else {
        this->mpc_flushing = false;
        this->mpc_flush_timer = 0;
    }

    if (false == this->mpc_flushing) {
        output = MAX(output, hydrCalib.min_mpc_pressure);
    }
    if (output < this->target_modulating_pressure) {
        // Filter when decreasing pressure, instant rise in pressure
        output = first_order_filter(hydrCalib.filter_factor, output, this->target_modulating_pressure);
    }

    return output;
}

void PressureManager::update_pressures(GearboxGear current_gear, GearChange change_state) {
    // This is my best guess at interpreting the assembly (Decompiler view messes a lot up with this function due to indirections)

    // -- Set solenoid currents --
    /* Uncomment when shifting is relevant
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
    sol_mpc->set_current_target(this->pcsKF->get_value(this->corrected_mpc_pressure, sensor_data->atf_temp+50.0));
    
    /* Uncomment when TCC is relevant
    sol_tcc->set_duty(this->get_tcc_solenoid_pwm_duty(this->target_tcc_pressure));
    */
}





// v Arduino testing v

void setup() {
    Serial.begin(115200);
}

PressureManager pm;
GearboxGear gear = GearboxGear::Second;
Clutch clutch = Clutch::K3;



void loop() {

};






