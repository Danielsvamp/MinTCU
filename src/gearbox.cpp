#include "gearbox.h"
#include "common_structs_ops.h"
#include <tcu_maths.h>
#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"
#include "nvs/module_settings.h"
#include "shifting_algo/shifting_algo_helpers.h"
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

// Same as UN52 gearbox.cpp
// ONLY FOR FORWARD GEARS!
int32_t calc_input_rpm_from_req_gear(const int32_t output_rpm, const GearboxGear req_gear, const GearboxConfiguration* gb_config)
{
    float calculated = output_rpm;
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
    return (int32_t)calculated;
}

bool is_controllable_gear(GearboxGear g)
{
    bool controllable = true;
    if (g == GearboxGear::Park || g == GearboxGear::Neutral || g == GearboxGear::SignalNotAvailable) {
        controllable = false;
    }
    return controllable;
}

bool is_fwd_gear(GearboxGear g)
{
    bool is_fwd = false;
    if ((uint8_t)g >= 1 && (uint8_t)g <= 5) {
        is_fwd = true;
    }
    return is_fwd;
}

// Nothing upshifts or downshifts yet, so D is always 2nd (UN52 last_motion_gear 'No pressure default'),
// the same gear the 722.6 uses in limp mode.
// R is Reverse_First (UN52 would pick Reverse_Second after 2nd). Reverse_First needs more pressure
// with this calibration, so the pressure is on the safe side whichever reverse the gearbox is in.
GearboxGear Gearbox::requested_gear_from_shifter() {
    GearboxGear ret = this->target_gear;
    switch (this->shifter_pos) {
        case ShifterPosition::D:
        case ShifterPosition::N_D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            ret = this->last_motion_gear;
            break;
        case ShifterPosition::R:
            ret = GearboxGear::Reverse_First;
            break;
        case ShifterPosition::P:
            ret = GearboxGear::Park;
            break;
        case ShifterPosition::N:
            ret = GearboxGear::Neutral;
            break;
        case ShifterPosition::P_R:
        case ShifterPosition::R_N:
            // Between positions, keep what we had (Same as UN52, it ignores these)
            if (GearboxGear::SignalNotAvailable == ret) {
                ret = (ShifterPosition::P_R == this->shifter_pos) ? GearboxGear::Park : GearboxGear::Neutral;
            }
            break;
        default:
            ret = GearboxGear::SignalNotAvailable;
            break;
    }
    return ret;
}

// Like UN52 shift_thread(), for the gears MinTCU can do (Garage shifts in and out of D/R)
void Gearbox::update_gear(GearboxGear requested) {
    if (this->pressure_mgr->is_failsafe()) {
        // No garage shifts in failsafe. The shift solenoids are off and MPC/SPC are at 0mA (max pressure),
        // so the gearbox engages whatever the selector says, like limp mode.
        if (this->shifting) {
            this->gar_result = 2;
            this->end_garage_shift();
        }
        this->actual_gear = requested;
        this->target_gear = requested;
        return;
    }
    if (this->shifting) {
        this->step_garage_shift();
        return;
    }
    if (requested == this->actual_gear) {
        this->target_gear = requested;
        return;
    }
    this->target_gear = requested;
    if (!is_controllable_gear(this->actual_gear) && !is_controllable_gear(requested)) {
        // No need to shift
        this->actual_gear = requested;
    } else if (is_controllable_gear(this->actual_gear) != is_controllable_gear(requested)) {
        // This would be a garage shift, either in or out
        if (is_controllable_gear(requested)) {
            // The garage shift needs the input speed (N2/N3). With the engine running in P/N the input
            // shaft spins with the engine, so if it reads (almost) 0 the sensors are not working.
            // Then engage like limp mode instead (SPC at max pressure), same as without garage shifting.
            if (this->sensor_data.input_rpm < 300) {
                this->gar_result = 3;
                this->actual_gear = requested;
            } else {
                this->start_garage_shift(requested);
                this->step_garage_shift();
            }
        } else {
            this->shift_to_n_p(requested);
        }
    } else if (is_fwd_gear(requested) != is_fwd_gear(this->actual_gear)) {
        // Shifter got stuck in R-D. Go to neutral, then garage shift next cycle (Same as UN52)
        this->actual_gear = GearboxGear::Neutral;
    } else {
        // Forward gear to forward gear (Upshift/downshift), not done yet
        this->target_gear = this->actual_gear;
    }
}

void Gearbox::shift_to_n_p(GearboxGear target) {
    // Shift to N/P from an in-gear gear (Same as UN52)
    if (this->actual_gear < GearboxGear::Third) {
        // So only for D1/D2 to N/P will this shift valve turn on
        this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
    }
    this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
    this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
    this->pressure_mgr->set_target_modulating_pressure(1500);
    this->pressure_mgr->set_target_shift_pressure(3000);
    this->actual_gear = target;
}

void Gearbox::start_garage_shift(GearboxGear target) {
    memset(&this->gar, 0, sizeof(this->gar));
    this->shifting = true;
    this->gar.target = target;
    this->gar.into_reverse = GearboxGear::Reverse_First == target || GearboxGear::Reverse_Second == target;
    this->gar.applying = Clutch::B2;
    if (this->gar.into_reverse) {
        this->gar.applying = Clutch::B3;
    } else if (GearboxGear::Fourth == target || GearboxGear::Fifth == target) {
        this->gar.applying = Clutch::K2;
    }
    this->gar.circuit = GearChange::_4_5;
    this->gar.spring_p = this->pressure_mgr->get_spring_pressure(this->gar.applying);
}

// One loop of the UN52 garage shift (gearbox.cpp shift_thread). All maths in int32_t (int is 16 bit on AVR)
void Gearbox::step_garage_shift() {
    GarageShift* g = &this->gar;
    GearboxGear target_gear = g->target;
    g->rpm_delta = abs((int32_t)sensor_data.input_rpm - calc_input_rpm_from_req_gear(sensor_data.output_rpm, target_gear, &this->gearboxConfig));
    int32_t rpm_delta = g->rpm_delta;
    int32_t sync_rpm_threshold = GAR_CURRENT_SETTINGS.sync_rpm_idle;
    if (sensor_data.pedal_pos > 10) {
        sync_rpm_threshold = GAR_CURRENT_SETTINGS.sync_rpm_with_accel;
    }
    int32_t p_1 = MIN(25, (1100L * sensor_data.pedal_pos) / 25);
    int32_t rpm_offset = MAX(0, (int32_t)sensor_data.engine_rpm - 800);
    int32_t dyn_adder = ((rpm_offset * 150) / 100) + p_1;
    uint8_t circuit_idx = ((uint8_t)g->circuit)-1;

    // K3 is never used here, so specifying 0 rear sun gear is OK
    g->centrifugal = pressure_mgr->calculate_centrifugal_force_for_clutch(g->applying, sensor_data.input_rpm, 0);
    if (g->timer_s > 0) {
        g->timer_s -= 1;
    }
    if (g->timer_m > 0) {
        g->timer_m -= 1;
    }
    // Safety, going into reverse, AND car is moving forward! - Enter neutral until stopped
    // (Needs the output speed sensor, output_rpm is 0 without it)
    if (0 == g->stage) {
        if ((g->into_reverse && sensor_data.output_rpm > GAR_CURRENT_SETTINGS.output_rpm_force_n) || sensor_data.output_rpm > GAR_CURRENT_SETTINGS.output_rpm_force_n) {
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, true);
            g->p_mod = 0;
            g->p_shift = 0;
        } else {
            if (!is_fwd_gear(target_gear) || target_gear == GearboxGear::Second || target_gear == GearboxGear::First) {
                g->circuit = GearChange::_1_2;
            }
            circuit_idx = ((uint8_t)g->circuit)-1;
            g->p_max_apply_clutch = pressure_manager->get_max_shift_pressure(circuit_idx);
        }
        g->stage += 1;
        g->substage = 0;
    } else if (g->stage == 1) {
        float div = 1.0;
        if (sensor_data.engine_rpm > 0) {
            div = (float)sensor_data.input_rpm / (float)sensor_data.engine_rpm;
            if (div > 1.5) {
                g->jump_to_pid = false;
            } else {
                g->jump_to_pid = true;
            }
        }

        if (g->substage == 0) {
            if (!is_fwd_gear(target_gear) || target_gear == GearboxGear::Second || target_gear == GearboxGear::First) {
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
            } else {
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, true);
            }
            g->timer_s = interpolate_float(sensor_data.atf_temp, 25, 11, -5, 20, InterpType::Linear);
            g->substage = 1;
        }
        if (g->substage == 1) {
            g->p_mod = GAR_CURRENT_SETTINGS.p_mod_applying;
            int32_t p = 0;
            if (g->applying == Clutch::K2) {
                p = 1200; // K2RAMP
            } else {
                p = interpolate_float(sensor_data.atf_temp, 4000, 1200, -35, 25, InterpType::Linear);
            }
            p = MAX(0, p + (int32_t)g->spring_p - (int32_t)g->centrifugal);
            g->p_apply_clutch = p + dyn_adder;
            g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
            if (0 == g->timer_s) {
                g->substage = 2;
                g->timer_s = interpolate_float(sensor_data.atf_temp, 50, 10, -20, 90, InterpType::Linear);
            }
        } else if (g->substage == 2) {
            g->p_mod = GAR_CURRENT_SETTINGS.p_mod_applying;
            if (0 == g->timer_s || (div <= 7.5 && sensor_data.atf_temp > -20)) {
                if (sensor_data.pedal_pos > 10) {
                    g->timer_s = 11;
                } else {
                    g->timer_s = 80;
                }
                g->substage = 3;
            }
        } else if (g->substage == 3) {
            if (0 == g->timer_s || g->jump_to_pid) {
                if (Clutch::B2 == g->applying) {
                    g->timer_s = 25;
                } else if (Clutch::K2 == g->applying) {
                    g->timer_s = 10;
                } else {
                    g->timer_s = 25;
                }
                g->substage = 4;
            } else {
                g->p_mod = GAR_CURRENT_SETTINGS.p_mod_applying;
                int32_t p = 0;
                if (g->applying == Clutch::K2) {
                    p = 7500; // K2RAMP2
                } else {
                    if (g->applying == Clutch::B2) {
                        p = 6600;
                    } else {
                        p = 4500;
                    }
                    p = interpolate_float(sensor_data.atf_temp, 8000, p, -35, 25, InterpType::Linear);
                }
                p = MAX(0, p + (int32_t)g->spring_p - (int32_t)g->centrifugal);
                g->p_apply_clutch = linear_ramp_with_timer(g->p_apply_clutch, p + dyn_adder, g->timer_s);
                g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
                if (rpm_delta < sync_rpm_threshold) {
                    g->timer_s = 9;
                    if (sensor_data.output_rpm < 60) {
                        g->timer_m = g->timer_s + interpolate_float(sensor_data.atf_temp, 40, 7, -30, 20, InterpType::Linear);
                    } else {
                        g->timer_m = g->timer_s + 80;
                    }
                    g->substage = 7;
                }
            }
        } else if (g->substage == 4) {
            if (0 == g->timer_s || g->jump_to_pid) {
                g->timer_s = 80; // TODO
                g->timer_m = 0;
                g->substage = 5;
            } else {
                g->p_mod = 3000;
                int32_t p = 0;
                if (g->applying == Clutch::K2) {
                    p = 7500; // K2RAMP2
                } else {
                    if (g->applying == Clutch::B2) {
                        p = 6600;
                    } else {
                        p = 4500;
                    }
                    p = interpolate_float(sensor_data.atf_temp, 8000, p, -35, 25, InterpType::Linear);
                }
                p = MAX(0, p + (int32_t)g->spring_p - (int32_t)g->centrifugal);
                g->p_apply_clutch = p + dyn_adder;
                g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
                if (rpm_delta < sync_rpm_threshold) {
                    g->timer_s = 9;
                    if (sensor_data.output_rpm < 60) {
                        g->timer_m = g->timer_s + interpolate_float(sensor_data.atf_temp, 40, 7, -30, 20, InterpType::Linear);
                    } else {
                        g->timer_m = g->timer_s + 80;
                    }
                    g->substage = 7;
                }
            }
        } else if (g->substage == 5) {
            g->p_mod = GAR_CURRENT_SETTINGS.p_mod_applying;
            g->p_apply_clutch += GAR_CURRENT_SETTINGS.spc_ramp_apply;
            g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
            if (0 == g->timer_s || g->p_shift > pressure_manager->get_max_solenoid_pressure() - 1500 || rpm_delta < sync_rpm_threshold) {
                g->timer_s = 9;
                if (sensor_data.output_rpm < 60) {
                    g->timer_m = g->timer_s + interpolate_float(sensor_data.atf_temp, 50, 15, -30, 20, InterpType::Linear);
                } else {
                    g->timer_m = g->timer_s + 80;
                }
                g->substage = 7;
            }
        } else if (g->substage == 6 || g->substage == 7) {
            bool done = false;
            if (0 == g->timer_s) {
                g->p_shift = pressure_manager->get_max_solenoid_pressure();
                g->p_apply_clutch = g->p_max_apply_clutch;
                g->p_mod = GAR_CURRENT_SETTINGS.p_mod_sync;
                if (0 == g->timer_m) {
                    done = true;
                }
            } else {
                g->p_apply_clutch = linear_ramp_with_timer(g->p_apply_clutch, g->p_max_apply_clutch, g->timer_s);
                g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
                g->p_mod = linear_ramp_with_timer(g->p_mod, GAR_CURRENT_SETTINGS.p_mod_sync, g->timer_s);
            }

            if (done) {
                g->timer_s = 20;
                g->substage = 8;
            }
        } else if (g->substage == 8) {
            // Check for completion
            if (rpm_delta <= GAR_CURRENT_SETTINGS.completed_rpm) {
                // Sync is OK!
                g->substage = 0;
                g->stage = 2;
            } else if (0 == g->timer_s) {
                // Did not complete OK
                g->stage = 3;
            }
        } else {
            // Stage 9 - Reset stage 1
            if (0 == g->timer_s) {
                g->substage = 0;
            }
        }
    } else if (g->stage == 2) {
        // Exit!
        g->p_mod = GAR_CURRENT_SETTINGS.p_mod_sync + 100;
        g->p_apply_clutch = g->p_max_apply_clutch;
        g->p_shift = ShiftHelpers::correct_shift_shift_pressure(pressure_manager, g->p_apply_clutch, circuit_idx);
        g->completed_ok = true;
    } else if (g->stage == 3) {
        // Failure to shift, try again!
        if (0 == g->substage) {
            // Init
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, true);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, true);
            this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
            g->timer_s = interpolate_float(sensor_data.atf_temp, 150, 40, -20, 30, InterpType::Linear);
            g->substage += 1;
        } else {
            g->p_mod = 0;
            g->p_apply_clutch = 0;
            g->p_shift = 0;
            if (0 == g->timer_s) {
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
                this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
                g->stage = 1;
                g->substage = 0;
            }
        }
    }
    pressure_mgr->set_target_modulating_pressure(g->p_mod);
    pressure_mgr->set_target_shift_pressure(g->p_shift);

    if (g->completed_ok) {
        // Garage shift completed OK
        this->gar_result = 1;
        this->actual_gear = target_gear;
        this->end_garage_shift();
    } else if (this->shifter_pos == ShifterPosition::N || this->shifter_pos == ShifterPosition::P) {
        // Garage shift aborted
        this->gar_result = 2;
        this->actual_gear = this->shifter_pos == ShifterPosition::P ? GearboxGear::Park : GearboxGear::Neutral;
        this->target_gear = this->actual_gear;
        this->end_garage_shift();
        pressure_mgr->set_target_shift_pressure(4000);
    }
}

void Gearbox::end_garage_shift() {
    // Same as the end of the UN52 garage shift
    pressure_mgr->set_spc_p_max();
    this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_3_4, false);
    this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_2_3, false);
    this->pressure_mgr->set_shift_circuit(ShiftCircuit::sc_1_2, false);
    this->shifting = false;
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
    GearboxGear requested = this->requested_gear_from_shifter();

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
    if (GearboxGear::SignalNotAvailable == requested) {
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

    // Gear / garage shift
    this->update_gear(requested);

    if (this->shifting) {
        // The garage shift sets MPC and SPC itself (Same as UN52, it uses the target gear)
        pressure_mgr->update_pressures(this->gar.target, this->gar.circuit);
    } else {
        if (!this->pressure_mgr->is_failsafe()) {
            this->mpc_working = pressure_mgr->find_working_mpc_pressure(this->actual_gear, true);
            this->pressure_mgr->set_target_modulating_pressure(this->mpc_working);
        }
        pressure_mgr->update_pressures(this->actual_gear, GearChange::_IDLE);
    }
    // Open loop current -> PWM. Uses the real ATF reading (INT16_MAX if missing, the solenoid then assumes cold)
    uint16_t vbatt = TCUIO::battery_mv();
    sol_mpc->update(vbatt, tmp_atf);
    sol_spc->update(vbatt, tmp_atf);
    Solenoids::update_shift_solenoids(vbatt);

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
    Serial.print(TCUIO::output_rpm() != UINT16_MAX ? (int32_t)this->sensor_data.output_rpm : (int32_t)-1);
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
    Serial.print(sol_spc->get_current_target());
    Serial.print(',');
    Serial.print(this->pressure_mgr->get_active_shift_circuits(), HEX);
    Serial.print(',');
    if (this->shifting) {
        Serial.print('S');
        Serial.print(this->gar.stage);
        Serial.print('.');
        Serial.print(this->gar.substage);
    } else {
        Serial.print(this->gar_result);
    }
    Serial.print(',');
    Serial.print(this->failsafe_reasons, HEX);
    Serial.print(',');
    Serial.println(egs_can_hal->get_rx_overflow_count());
}

Gearbox* gearbox = nullptr;
