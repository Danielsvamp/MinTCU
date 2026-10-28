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
    this->gearboxConfig.max_torque = 0;
    this->cal_ok = EGSCal::init_egs_calibration();

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

void Gearbox::controller_loop() {
    uint32_t start = GET_CLOCK_TIME();
    if (start - this->last_frame_time < GEARBOX_FRAME_MS) {
        return; // should run at 50 hz, unless something else takes too long
    }
    this->last_frame_time = start;

    TCUIO::update_io_layer();

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
        // input_rpm is 0 until N2/N3 are read, so this uses the stall torque multiplication (safe side)
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
