#include "gearbox.h"
#include "common_structs_ops.h"
#include <tcu_maths.h>
#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"
#include <stdlib.h> //manuellt hidlagt

Gearbox::Gearbox(Shifter* shifter) : shifter(shifter)
{
    this->sensor_data = SensorData{
        .input_rpm = 0,
        .engine_rpm = 0,
        .output_rpm = 0,
        .pedal_pos = 0,
        .pedal_pos_smoothed = 0,
        .atf_temp = 0,
        .input_torque = 0,
        .converted_torque = 0,
        .converted_driver_torque = 0,
        .indicated_torque = 0,
        .max_torque = 0,
        .min_torque = 0,
        .last_shift_time = 0,
        .gear_ratio = 0.0F,
        .kickdown_pressed = false,
        .brake_pressed = false,
    };

    float r1 = ((float)(MECH_PTR->ratio_table[1])) / 1000.0;
    float r2 = ((float)(MECH_PTR->ratio_table[2])) / 1000.0;
    float r3 = ((float)(MECH_PTR->ratio_table[3])) / 1000.0;
    float r4 = ((float)(MECH_PTR->ratio_table[4])) / 1000.0;
    float r5 = ((float)(MECH_PTR->ratio_table[5])) / 1000.0;
    float rr1 = ((float)(MECH_PTR->ratio_table[6]) * -1) / 1000.0;
    float rr2 = ((float)(MECH_PTR->ratio_table[7]) * -1) / 1000.0;

    // IMPORTANT - Set the Ratio2/Ratio1 multiplier for the sensor RPM reading algorithm!
    TCUIO::set_2_1_ratio(r1 / r2);

    this->pressure_mgr = new PressureManager(&this->sensor_data, this->gearboxConfig.max_torque);
    pressure_manager = this->pressure_mgr;
    
    // Wait for solenoid routine to complete
}

void Gearbox::controller_loop() {
	static uint32_t last_start = 0;
    uint32_t start = GET_CLOCK_TIME();
    if (start - last_start < 20) {return;} // should run at 50 hz, unless something else takes too long
    last_start = start;
	
    TCUIO::update_io_layer();

    // Set sensors Motor temperature (Always ran)
    int16_t coolant_temp = egs_can_hal->get_engine_coolant_temp(50);

    uint8_t p_tmp = egs_can_hal->get_pedal_value(1000);
    this->pedal_last = this->sensor_data.pedal_pos;
    if (p_tmp != 0xFF)
    {
        this->sensor_data.pedal_pos = p_tmp;
    }
    else {
        p_tmp = 250 / 4; // 25% as a fallback
    }
    
    this->sensor_data.pedal_pos_smoothed = linear_interp_with_percentage(80, p_tmp, this->sensor_data.pedal_pos_smoothed);

    if (GET_CLOCK_TIME() - start > 100) {
        // Update every 100ms, not every EGS cycle, values multiplied by 10
        // to get them in terms of 1 second (1s/100ms = 10)
        if (this->pedal_delta) {
            this->pedal_delta->update(this->sensor_data.pedal_pos * 10);
        }
        if (this->input_rpm_delta) {
            this->input_rpm_delta->update(this->sensor_data.input_rpm * 10);
        }
        this->last_delta_time = start;
    }

    int tmp_rpm = 0;
    tmp_rpm = egs_can_hal->get_engine_rpm(1000);
    if (tmp_rpm == UINT16_MAX)
    {
        tmp_rpm = this->sensor_data.engine_rpm; // Sub last value!
    }
    this->cached_engine_rpm = first_order_filter(3, tmp_rpm * 100, this->cached_engine_rpm);
    this->sensor_data.engine_rpm = this->cached_engine_rpm / 100;
    
    // Update solenoids, only if engine RPM is OK
    if (tmp_rpm > 400)
    {
        if (!shifting)
        {
            this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, true);
            this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
        }
    }

    // update atf temp
    int16_t tmp_atf = TCUIO::atf_temperature();
    if (INT16_MAX != tmp_atf)
    {
        this->sensor_data.atf_temp = tmp_atf;
    }
    
    egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
    egs_can_hal->set_shifter_position(this->shifter_pos);
    egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
    egs_can_hal->set_tcc_trq_multiplier(this->sensor_data.tcc_trq_multiplier);
    
    CanTorqueData trqs = egs_can_hal->get_torque_data(100);
    // CALC TORQUES
    if (INT16_MAX != trqs.m_min) { sensor_data.min_torque = trqs.m_min; }
    if (INT16_MAX != trqs.m_max) { sensor_data.max_torque = trqs.m_max; }
    if (INT16_MAX != trqs.m_ind) { sensor_data.indicated_torque = trqs.m_ind; }
    if (INT16_MAX != trqs.m_converted_static) { sensor_data.converted_torque = trqs.m_converted_static; }
    if (INT16_MAX != trqs.m_converted_driver) {
        int input_trq = InputTorqueModel::get_input_torque(
            sensor_data.engine_rpm,
            sensor_data.input_rpm,
            trqs.m_converted_driver
        );
        sensor_data.input_torque = input_trq;
        sensor_data.converted_driver_torque = trqs.m_converted_driver;
    }

    pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
}



Gearbox* gearbox = nullptr;
