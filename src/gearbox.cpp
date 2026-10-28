#include "gearbox.h"
#include "common_structs_ops.h"
#include <tcu_maths.h>
#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"
#include <stdlib.h> //manuellt hidlagt
#include <string.h>
#include <Arduino.h>

// Control loop period, same as UN52
#define GEARBOX_FRAME_MS 20
// Same as UN52: substitute a missing engine RPM for this many cycles, then treat the engine as stopped
#define ENGINE_RPM_MISSING_MAX_CYCLES 50 // 1 second
// Torque frames older than this put the gearbox into failsafe (0mA = max pressure)
#define TORQUE_EXPIRE_MS 200
// Shifter frames older than this put the gearbox into failsafe
#define SHIFTER_EXPIRE_MS 500
// Everything has to be OK for this many cycles before leaving failsafe
#define FAILSAFE_RECOVERY_CYCLES 50 // 1 second
// Used for the pcs map when the ATF temperature is not available. Hot = less current for the
// same pressure in the map, so this makes the pressure higher than wanted, not lower.
#define ATF_FAILSAFE_TEMP_C 120
// Print a log line every this many frames (5 = 10Hz)
#define LOG_EVERY_N_FRAMES 5

Gearbox::Gearbox()
{
    memset(&this->sensor_data, 0, sizeof(this->sensor_data));
    this->sensor_data.atf_temp = ATF_FAILSAFE_TEMP_C;
    this->sensor_data.tcc_trq_multiplier = 1.0;
    memset(&this->speed_sensors, 0, sizeof(this->speed_sensors));
    this->gearboxConfig.max_torque = 0;
    this->cal_ok = EGSCal::init_egs_calibration();
    this->sensors_ok = TCUIO::setup_io_layer();
    if (this->cal_ok) {
        // Same as UN52 (Ratios are checked to be non zero in init_egs_calibration)
        float r1 = ((float)(MECH_PTR->ratio_table[1])) / 1000.0;
        float r2 = ((float)(MECH_PTR->ratio_table[2])) / 1000.0;
        float r3 = ((float)(MECH_PTR->ratio_table[3])) / 1000.0;
        float r4 = ((float)(MECH_PTR->ratio_table[4])) / 1000.0;
        float r5 = ((float)(MECH_PTR->ratio_table[5])) / 1000.0;
        float rr1 = ((float)(MECH_PTR->ratio_table[6]) * -1) / 1000.0;
        float rr2 = ((float)(MECH_PTR->ratio_table[7]) * -1) / 1000.0;

        this->gearboxConfig.max_torque = 330;
        if (MECH_PTR->gb_ty == 0) {
            this->gearboxConfig.max_torque = 580;
        }
        const float ratios[7] = {r1, r2, r3, r4, r5, rr1, rr2};
        for (uint8_t i = 0; i < 7; i++) {
            this->gearboxConfig.bounds[i] = GearRatioInfo{
                .ratio_max_drift = ratios[i] * (float)1.1,
                .ratio = ratios[i],
                .ratio_min_drift = ratios[i] * (float)0.9,
            };
        }
        // IMPORTANT - Set the Ratio2/Ratio1 multiplier for the sensor RPM reading algorithm!
        TCUIO::set_2_1_ratio(r1 / r2);
    }

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    pressure_manager = this->pressure_mgr;
    // Start in failsafe until the engine is running and CAN is OK
    this->pressure_mgr->set_failsafe(true);
}

void Gearbox::update_gear_from_shifter() {
    // No shift solenoids are used yet, so the gearbox is hydraulically in its limp gear:
    // 2nd in D, reverse in R. Tell the pressure manager about that gear.
    // (Reverse_First gives slightly more pressure than Reverse_Second with this calibration)
    switch (this->shifter_pos) {
        case ShifterPosition::D:
        case ShifterPosition::N_D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            this->actual_gear = GearboxGear::Second;
            break;
        case ShifterPosition::R:
        case ShifterPosition::R_N:
        case ShifterPosition::P_R:
            this->actual_gear = GearboxGear::Reverse_First;
            break;
        case ShifterPosition::P:
            this->actual_gear = GearboxGear::Park;
            break;
        case ShifterPosition::N:
            this->actual_gear = GearboxGear::Neutral;
            break;
        default:
            this->actual_gear = GearboxGear::SignalNotAvailable;
            break;
    }
    this->target_gear = this->actual_gear;
}

bool Gearbox::is_stationary() {
    return this->sensor_data.output_rpm < 10;
}

// Same as UN52
bool Gearbox::process_speed_sensors()
{
    bool ok = true;
    bool conduct_sanity_check =
        (this->actual_gear == this->target_gear) && (                                                 // Same gear (Not shifting)
            (this->actual_gear == GearboxGear::Second) || // And in 2..
            (this->actual_gear == GearboxGear::Third) ||  // .. or 3 ..
            (this->actual_gear == GearboxGear::Fourth)    // .. or 4
            );
    uint16_t n2 = TCUIO::n2_rpm();
    uint16_t n3 = TCUIO::n3_rpm();
    uint16_t output = TCUIO::output_rpm();

    if (UINT16_MAX != n2 && UINT16_MAX != n3) {
        uint16_t turbine = TCUIO::calc_turbine_rpm(n2, n3);
        if (conduct_sanity_check) {
            if (abs((int32_t)n2 - (int32_t)n3) > 100) {
                ok = false;
            }
        }
        if (ok) {
            this->speed_sensors.turbine = turbine;
        }
        this->speed_sensors.n2 = n2;
        this->speed_sensors.n3 = n3;
    } else {
        ok = false;
    }

    if (UINT16_MAX != output) {
        speed_sensors.output = output;
    }
    else {
        ok = false; // Output RPM failed
    }

    return ok;
}

void Gearbox::controller_loop() {
    uint32_t start = GET_CLOCK_TIME();
    if (start - this->last_frame_time < GEARBOX_FRAME_MS) {
        return; // should run at 50 hz, unless something else takes too long
    }
    this->last_frame_time = start;

    TCUIO::update_io_layer();

    // Speed sensors. UN52 only copies input and output RPM when both are OK.
    // MinTCU can run without the output sensor, so the input RPM is used on its own
    bool speeds_valid = this->process_speed_sensors();
    this->sensor_data.input_rpm = this->speed_sensors.turbine;
    if (TCUIO::output_rpm() != UINT16_MAX) {
        this->sensor_data.output_rpm = this->speed_sensors.output;
    } else {
        this->sensor_data.output_rpm = 0;
    }
    if (speeds_valid && !this->is_stationary()) {
        // Store our ratio (Same as UN52)
        if (this->actual_gear == GearboxGear::Neutral || this->actual_gear == GearboxGear::Park || this->actual_gear == GearboxGear::SignalNotAvailable) {
            this->sensor_data.gear_ratio = 0.0;
        } else {
            this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
        }
    } else {
        // Stationary so no ratios
        this->sensor_data.gear_ratio = 0.0;
    }

    // Pedal (Same as UN52)
    uint8_t p_tmp = egs_can_hal->get_pedal_value(1000);
    if (p_tmp == 0xFF)
    {
        p_tmp = 250 / 4; // 25% as a fallback
    }
    this->pedal_last = p_tmp;
    this->sensor_data.pedal_pos = p_tmp;

    // Engine RPM (Same as UN52)
    int32_t tmp_rpm = egs_can_hal->get_engine_rpm(1000);
    if (tmp_rpm == UINT16_MAX)
    {
        // Substitute the last value for a short while, then treat the engine as stopped
        if (this->engine_rpm_missing_cycles < ENGINE_RPM_MISSING_MAX_CYCLES) {
            this->engine_rpm_missing_cycles += 1;
            tmp_rpm = this->sensor_data.engine_rpm; // Sub last value!
        } else {
            tmp_rpm = 0;
        }
    }
    else
    {
        this->engine_rpm_missing_cycles = 0;
    }
    this->sensor_data.engine_rpm = tmp_rpm;
    if (tmp_rpm > 400) {
        this->engine_running = true;
        this->engine_running_off_counter = 0;
    } else if (this->engine_running) {
        // UN52 uses the input shaft speed for this, we don't read N2/N3 yet
        this->engine_running_off_counter += 1;
        if (this->engine_running_off_counter >= 50) { // 1 second
            this->engine_running = false;
        }
    }

    // ATF temperature
    int16_t tmp_atf = TCUIO::atf_temperature();
    if (INT16_MAX != tmp_atf)
    {
        this->sensor_data.atf_temp = tmp_atf;
        this->atf_substituted = false;
    } else {
        this->sensor_data.atf_temp = ATF_FAILSAFE_TEMP_C;
        this->atf_substituted = true;
    }

    // Shifter
    this->shifter_pos = egs_can_hal->get_shifter_position(SHIFTER_EXPIRE_MS);
    this->update_gear_from_shifter();

    // Torques (Same as UN52)
    CanTorqueData trqs = egs_can_hal->get_torque_data(TORQUE_EXPIRE_MS);
    if (INT16_MAX != trqs.m_min) { sensor_data.min_torque = trqs.m_min; }
    if (INT16_MAX != trqs.m_max) { sensor_data.max_torque = trqs.m_max; }
    if (INT16_MAX != trqs.m_ind) { sensor_data.indicated_torque = trqs.m_ind; }
    if (INT16_MAX != trqs.m_converted_static) { sensor_data.converted_torque = trqs.m_converted_static; }
    if (INT16_MAX != trqs.m_converted_driver) {
        // Without N2/N3 input_rpm is 0, so this uses the stall torque multiplication (safe side)
        int16_t input_trq = InputTorqueModel::get_input_torque(
            sensor_data.engine_rpm,
            sensor_data.input_rpm,
            trqs.m_converted_driver
        );
        sensor_data.input_torque = input_trq;
        sensor_data.converted_driver_torque = trqs.m_converted_driver;
    }

    // Failsafe (Not in UN52, UN52 keeps the last torque value forever)
    // Anything we cannot trust -> 0mA on MPC/SPC = maximum line pressure, like an unpowered TCU
    uint8_t reasons = FS_NONE;
    if (!this->cal_ok) {
        reasons |= FS_NO_CAL;
    }
    if (!this->engine_running) {
        reasons |= FS_ENGINE_OFF;
    }
    if (INT16_MAX == trqs.m_converted_driver) {
        reasons |= FS_NO_TORQUE;
    }
    if (GearboxGear::SignalNotAvailable == this->actual_gear) {
        reasons |= FS_NO_SHIFTER;
    }
    if (FS_NONE != reasons) {
        this->failsafe_ok_cycles = 0;
    } else if (this->pressure_mgr->is_failsafe()) {
        if (this->failsafe_ok_cycles < FAILSAFE_RECOVERY_CYCLES) {
            this->failsafe_ok_cycles++;
            reasons |= FS_RECOVERING;
        }
    }
    this->failsafe_reasons = reasons;
    this->pressure_mgr->set_failsafe(FS_NONE != reasons);

    if (!this->pressure_mgr->is_failsafe() && !shifting) {
        this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, true);
        this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
    }

    pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
    // Open loop current -> PWM. Uses the real ATF reading (INT16_MAX if missing, the solenoid then assumes cold)
    uint16_t vbatt = TCUIO::battery_mv();
    sol_mpc->update(vbatt, tmp_atf);
    sol_spc->update(vbatt, tmp_atf);

    this->log_counter++;
    if (this->log_counter >= LOG_EVERY_N_FRAMES) {
        this->log_counter = 0;
        this->print_log();
    }
}

void Gearbox::print_log() {
    // CSV, header is printed in setup(). Keep this short, Serial.print blocks when the 64 byte buffer is full
    Serial.print(GET_CLOCK_TIME());
    Serial.print(',');
    Serial.print(this->sensor_data.engine_rpm);
    Serial.print(',');
    Serial.print(this->speed_sensors.n2);
    Serial.print(',');
    Serial.print(this->speed_sensors.n3);
    Serial.print(',');
    Serial.print(this->sensor_data.input_rpm);
    Serial.print(',');
    Serial.print(TCUIO::output_rpm() != UINT16_MAX ? this->sensor_data.output_rpm : -1);
    Serial.print(',');
    Serial.print(this->sensor_data.gear_ratio, 2);
    Serial.print(',');
    Serial.print(this->sensor_data.pedal_pos);
    Serial.print(',');
    Serial.print(this->sensor_data.converted_torque);
    Serial.print(',');
    Serial.print(this->sensor_data.converted_driver_torque);
    Serial.print(',');
    Serial.print(this->sensor_data.input_torque);
    Serial.print(',');
    Serial.print((uint8_t)this->shifter_pos);
    Serial.print(',');
    Serial.print((uint8_t)this->actual_gear);
    Serial.print(',');
    Serial.print(this->sensor_data.atf_temp);
    Serial.print(this->atf_substituted ? "s" : "");
    Serial.print(',');
    Serial.print(TCUIO::battery_mv());
    Serial.print(',');
    Serial.print(this->pressure_mgr->get_input_modulating_pressure());
    Serial.print(',');
    Serial.print(this->pressure_mgr->get_corrected_modulating_pressure());
    Serial.print(',');
    Serial.print(sol_mpc->get_current_target());
    Serial.print(',');
    Serial.print(sol_mpc->get_current_estimate());
    Serial.print(',');
    Serial.print(sol_mpc->get_duty_permille());
    Serial.print(',');
    Serial.print(this->failsafe_reasons, HEX);
    Serial.print(',');
    Serial.println(egs_can_hal->get_rx_overflow_count());
}

Gearbox* gearbox = nullptr;
