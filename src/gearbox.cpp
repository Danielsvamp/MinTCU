#include "gearbox.h"
#include "common_structs_ops.h"
//#include "nvs/eeprom_config.h"
//#include "adv_opts.h"
#include <tcu_maths.h>
//#include "speaker.h"
#include "clock.hpp"
//#include "nvs/device_mode.h"
#include "egs_calibration/calibration_structs.h"
//#include "shifting_algo/s_algo.h"
//#include "shifting_algo/shift_crossover.h"
//#include "shifting_algo/shift_release.h"
#include "tcu_io/tcu_io.hpp"
#include <stdlib.h> //manuellt hidlagt

#define SBS SBS_CURRENT_SETTINGS

const uint8_t AVG_SAMPLES_500MS = 500 / 20;

// ONLY FOR FORWARD GEARS!
int calc_input_rpm_from_req_gear(const int output_rpm, const GearboxGear req_gear, const GearboxConfiguration* gb_config)
{
    int calculated = output_rpm;
    switch (req_gear)
    {
    case GearboxGear::First:
        calculated *= gb_config->bounds[0].ratio;
        break;
    case GearboxGear::Second:
        calculated *= gb_config->bounds[1].ratio;
        break;
    case GearboxGear::Third:
        calculated *= gb_config->bounds[2].ratio;
        break;
    case GearboxGear::Fourth:
        calculated *= gb_config->bounds[3].ratio;
        break;
    case GearboxGear::Fifth:
        calculated *= gb_config->bounds[4].ratio;
        break;
    default:
        break;
    }
    return calculated;
}

Gearbox::Gearbox(Shifter* shifter) : shifter(shifter), kickdown(), brake_pedal()
{
    //this->current_profile = nullptr;
    //egs_can_hal->set_drive_profile(GearboxProfile::Underscore); // Uninitialized
    //this->profile_mutex = portMUX_INITIALIZER_UNLOCKED;
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
    //this->tcc = new TorqueConverter(this->gearboxConfig.max_torque);
    //this->shift_adapter = new ShiftAdaptationSystem();
    pressure_manager = this->pressure_mgr;
    //adaptation_manager = this->shift_adapter;
    // Wait for solenoid routine to complete
}

#define SHIFT_DELAY_MS 20     // 20ms steps
#define NUM_SCD_ENTRIES 100 / SHIFT_DELAY_MS // 100ms moving average window

void Gearbox::controller_loop()
{
    ShifterPosition last_position = ShifterPosition::SignalNotAvailable;
    uint32_t expire_check = GET_CLOCK_TIME() + 100; // 100ms

    while (1)
    {
        uint32_t start = GET_CLOCK_TIME();
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

        sensor_data.brake_pressed = brake_pedal.is_brake_pedal_pressed(egs_can_hal, 250);
        sensor_data.kickdown_pressed = kickdown.is_kickdown_newly_pressed(egs_can_hal, 250);
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
        uint8_t pll = TCUIO::parking_lock();
        if (UINT8_MAX != pll)
       
        if (this->sensor_data.engine_rpm > 100)
        {
            if (speeds_valid && is_fwd_gear(this->actual_gear))
            {
                // Check our range restict (Only for TRRS)
                switch (egs_can_hal->get_shifter_position(250)) { // Don't use shifter_pos, as that only registers D. Query raw selector pos
                case ShifterPosition::FOUR:
                    this->restrict_target = GearboxGear::Fourth;
                    break;
                case ShifterPosition::THREE:
                    this->restrict_target = GearboxGear::Third;
                    break;
                case ShifterPosition::TWO:
                    this->restrict_target = GearboxGear::Second;
                    break;
                case ShifterPosition::ONE:
                    this->restrict_target = GearboxGear::First;
                    break;
                default:
                    this->restrict_target = GearboxGear::Fifth;
                    break;
                }
                // Seek up the restriction target if the RPM is too high for the current gear!
                // Seek up to Fifth
                while (this->restrict_target != GearboxGear::Fifth && calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, this->restrict_target, &this->gearboxConfig) > this->redline_rpm)
                {
                    this->restrict_target = next_gear(this->restrict_target);
                }

                // In gear, not shifting, and no ratio mismatch
                if (!shifting && this->actual_gear == this->target_gear && gear_disagree_count == 0)
                {
                    // Enter critical ISR section
                    portENTER_CRITICAL(&this->profile_mutex);
                    AbstractProfile* p = this->current_profile;
                    // Exit critical
                    portEXIT_CRITICAL(&this->profile_mutex);
                    // Check if profile is loaded
                    if (p != nullptr)
                    {
                        p->update(&this->sensor_data);
                        // Ask the current drive profile if it thinks, given the current
                        // data, if the car should up/downshift
                        if (this->restrict_target > this->actual_gear && p->should_upshift(this->actual_gear, &this->sensor_data))
                        {
                            this->ask_upshift = true; // Upshift takes priority
                            this->manual_shift = false;
                        }
                        else if (this->restrict_target < this->actual_gear || p->should_downshift(this->actual_gear, &this->sensor_data)) {
                            this->ask_downshift = true; // Downshift is secondary
                            this->manual_shift = false;
                        }
                    }
                    if (this->ask_upshift && this->actual_gear < GearboxGear::Fifth)
                    {
                        // Check RPMs
                        GearboxGear next = next_gear(this->actual_gear);
                        // Second gear shift defaults to OK as we can safely start in second (For C/W mode)
                        if (next == GearboxGear::Second || calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, next, &this->gearboxConfig) > 900)
                        {
                            this->target_gear = next;
                        }
                    }
                    else if ((this->ask_downshift || sensor_data.kickdown_pressed) && this->actual_gear > GearboxGear::First)
                    {
                        // Check RPMs
                        GearboxGear prev = prev_gear(this->actual_gear);
                        if (calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev, &this->gearboxConfig) < this->redline_rpm - 500)
                        {
                            this->target_gear = prev;
                        }
                    }
                }
                // Request processed. Cancel the requests. Put this outside here so that if there is a ratio mismatch, paddles are ignored
                this->ask_downshift = false;
                this->ask_upshift = false;
                this->shift_req_was_manual = this->manual_shift;
                this->manual_shift = false;

                if (is_fwd_gear(this->target_gear))
                {
                    if (this->tcc != nullptr)
                    {
                        this->tcc->update(this->actual_gear, this->target_gear, this->pressure_mgr, this->current_profile, &this->sensor_data);
                        egs_can_hal->set_clutch_status(this->tcc->get_clutch_state());
                    }
                }
            }
            else { // Cannot read, or not in foward gear!
                this->tcc_percent = 0;
                this->pressure_mgr->set_target_tcc_pressure(0);
                egs_can_hal->set_clutch_status(TccClutchStatus::Open);
                // sol_tcc->write_pwm_12_bit(0);
            }
            // Not shifting, but target has changed! Spawn a shift thread!
            if (this->target_gear != this->actual_gear && !this->shifting)
            {
                xTaskCreatePinnedToCore(Gearbox::start_shift_thread, "Shift handler", 8192, this, 10, &this->shift_task, 1);
            }
        }
        else if (!shifting)
        {
            sol_mpc->set_current_target(0);
            sol_spc->set_current_target(0);
            sol_tcc->set_duty(0);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
        }

        int16_t tmp_atf = TCUIO::atf_temperature();
        if (INT16_MAX != tmp_atf)
        {
            this->sensor_data.atf_temp = tmp_atf;
        }
        else
        {
            if (!temp_cal)
            {
                temp_cal = true;
                temp_at_test = tmp_atf;
                if (temp_at_test != 25)
                {
                    resistance_mpc = resistance_mpc + (resistance_mpc * (((25.0 - (float)temp_at_test) * 0.393) / 100.0));
                    resistance_spc = resistance_spc + (resistance_spc * (((25.0 - (float)temp_at_test) * 0.393) / 100.0));
                }
                ESP_LOGI("GB", "Calibrated solenoids at %d C. Adjusted for 25C: SPC %.2f MPC %.2f", tmp_atf, resistance_spc, resistance_mpc);
            }
            // SPC and MPC can cause voltage swing on the ATF line, so disable
            // monitoring when shifting gears!
            if (!shifting)
            {
                this->sensor_data.atf_temp = tmp_atf;
            }
        }
        egs_can_hal->set_gearbox_temperature(this->sensor_data.atf_temp);
        egs_can_hal->set_shifter_position(this->shifter_pos);
        egs_can_hal->set_input_shaft_speed(this->sensor_data.input_rpm);
        egs_can_hal->set_tcc_trq_multiplier(this->sensor_data.tcc_trq_multiplier);
        if (this->aborting)
        {
            egs_can_hal->set_abort_shift(true);
        }
        else
        {
            egs_can_hal->set_target_gear(this->target_gear);
        }
        egs_can_hal->set_actual_gear(this->actual_gear);
        egs_can_hal->set_wheel_torque(0); // Nm

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
        sensor_data.pump_torque = InputTorqueModel::get_pump_torque(sensor_data.engine_rpm, sensor_data.input_rpm);

        if (this->shifting && is_controllable_gear(this->target_gear) && !is_controllable_gear(this->actual_gear)) {
            if (INT16_MAX != sensor_data.pump_torque) {
                sensor_data.input_torque = sensor_data.pump_torque * sensor_data.tcc_trq_multiplier;
            }
        }

        // Wheel torque
        
        if (this->sensor_data.gear_ratio == 0)
        {
            // Fallback ratio for when gear ratio is actually 0
            float f;
            switch (this->target_gear)
            {
            case GearboxGear::First:
                f = gearboxConfig.ratios[0];
                break;
            case GearboxGear::Second:
                f = gearboxConfig.ratios[1];
                break;
            case GearboxGear::Third:
                f = gearboxConfig.ratios[2];
                break;
            case GearboxGear::Fourth:
                f = gearboxConfig.ratios[3];
                break;
            case GearboxGear::Reverse_First:
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Reverse_Second:
                f = gearboxConfig.ratios[4] * -1;
                break;
            case GearboxGear::Park:
            case GearboxGear::SignalNotAvailable:
            case GearboxGear::Neutral:
            default:
                f = 0.0;
                break;
            }
            egs_can_hal->set_wheel_torque_multi_factor(f);
        }
        else
        {
            egs_can_hal->set_wheel_torque_multi_factor(this->sensor_data.gear_ratio);
        }

        // ESP_LOG_LEVEL(ESP_LOG_INFO, "GEARBOX", "Torque: MIN: %3d, MAX: %3d, STAT: %3d", min_torque, max_torque, static_torque);
        //  Show debug symbols on IC
        float ratio_from_c_gear = ratio_absolute(this->actual_gear, &this->gearboxConfig);
        float ratio_from_t_gear = ratio_absolute(this->target_gear, &this->gearboxConfig);
        float tcc_multipler = InputTorqueModel::get_input_torque_factor(sensor_data.engine_rpm, sensor_data.input_rpm);
        this->sensor_data.tcc_trq_multiplier = tcc_multipler;
        float torque_ratio = 0; // Implausible
        if (
            ratio_from_c_gear != 0 && // Valid ratio
            sensor_data.engine_rpm != 0 // Engine is turning
            ) {
            torque_ratio = ratio_from_c_gear;
            if (ratio_from_t_gear > ratio_from_c_gear) {
                torque_ratio = ratio_from_t_gear;
            }
            torque_ratio *= tcc_multipler;
            torque_ratio *= diff_ratio_f;
            if (torque_ratio < 1) {
                torque_ratio = 1; // HOW!? (diff ratio is always > 2.0)
            }
        }
        else if (sensor_data.engine_rpm == 0) {
            torque_ratio = -1; // Cannot calculate
        }
        egs_can_hal->set_wheel_torque_multi_factor(torque_ratio);
        if (this->show_upshift && this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::RequestGearAgain);
        }
        else if (this->show_upshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Upshift);
        }
        else if (this->show_downshift)
        {
            egs_can_hal->set_display_msg(GearboxMessage::Downshift);
        }
        else
        {
            egs_can_hal->set_display_msg(GearboxMessage::None);
        }

        // Lastly, set display gear
        portENTER_CRITICAL(&this->profile_mutex);
        if (this->current_profile != nullptr)
        {
            egs_can_hal->set_drive_profile(this->current_profile->get_profile());
            if (this->flaring && SBS.f_shown_if_flare)
            {
                // Takes president
                egs_can_hal->set_display_msg(GearboxMessage::None);
                egs_can_hal->set_display_gear(GearboxDisplayGear::Failure, false);
            }
            else
            {
                if (this->current_profile == race && this->fwd_gear_shift && SBS.debug_show_up_down_arrows_in_r) {
                    egs_can_hal->set_display_msg(this->is_upshift ? GearboxMessage::Upshift : GearboxMessage::Downshift);
                }
                else if ((this->current_profile == manual || this->current_profile == race) &&
                    sensor_data.engine_rpm > this->redline_rpm - 1000
                    ) {
                    egs_can_hal->set_display_msg(GearboxMessage::Upshift);
                }
                else {
                    egs_can_hal->set_display_msg(GearboxMessage::None);
                }
                egs_can_hal->set_display_gear(this->current_profile->get_display_gear(this->target_gear, this->actual_gear), this->current_profile == manual);
            }
        }
        portEXIT_CRITICAL(&this->profile_mutex);
        pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
        uint32_t time = GET_CLOCK_TIME() - start;
        if (time < 20) {
            vTaskDelay((20 - time) / portTICK_PERIOD_MS); // 50 updates/sec!
        }
    }
}

bool Gearbox::process_speed_sensors()
{
    bool ok = true;
    bool conduct_sanity_check = gear_disagree_count == 0 &&
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
            if (abs(n2 - n3) > 100) {
                ok = false;
            }
        }
        if (ok) {
            this->speed_sensors.turbine = turbine;
        }
        this->speed_sensors.n2 = n2;
        this->speed_sensors.n3 = n3;
    }

    if (UINT16_MAX != output) {
        speed_sensors.output = output;
    }
    else {
        ok = false; // Output RPM failed
    }

    return ok;
}

Gearbox* gearbox = nullptr;
