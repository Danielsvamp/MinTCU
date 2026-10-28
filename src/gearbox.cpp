#include "gearbox.h"
#include "common_structs_ops.h"
#include <tcu_maths.h>
#include "clock.hpp"
#include "egs_calibration/calibration_structs.h"
#include "tcu_io/tcu_io.hpp"
#include "nvs/module_settings.h"
#include "shifting_algo/s_algo.h"
#include "shifting_algo/shift_crossover.h"
#include "shifting_algo/shift_release.h"
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
// Same as UN52 gearbox.cpp
#define SHIFT_DELAY_MS 20     // 20ms steps

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
    this->tcc = new TorqueConverter();
    // MinTCU only has the Manual profile (+/- on the selector)
    Profiles::init_profiles(VEHICLE_CONFIG.engine_type == 0);
    this->current_profile = manual;
    memset(&this->fwd, 0, sizeof(this->fwd));

    // Same as UN52
    if (VEHICLE_CONFIG.engine_type == 1)
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_petrol;
    }
    else
    {
        this->redline_rpm = VEHICLE_CONFIG.red_line_rpm_diesel;
    }
    if (this->redline_rpm < 4000)
    {
        this->redline_rpm = 4000; // just in case
    }
    this->diff_ratio_f = (float)VEHICLE_CONFIG.diff_ratio / 1000.0;
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

// Same as UN52 gearbox.cpp
GearboxGear gear_from_idx(uint8_t idx)
{
    // Only for drivable gears. P/R/SNV is never used
    GearboxGear ret = GearboxGear::SignalNotAvailable;
    if (idx >= 1 && idx <= 5) {
        ret = (GearboxGear)(idx); // Direct cast for gears 1-5
    }
    else if (idx == 6) {
        ret = GearboxGear::Reverse_First;
    }
    else if (idx == 7) {
        ret = GearboxGear::Reverse_Second;
    }
    return ret;
}

GearboxGear next_gear(GearboxGear g)
{
    GearboxGear next = g;
    uint8_t idx = (uint8_t)g;
    if (idx >= 1 && idx < 5) { // 1-4
        next = (GearboxGear)(idx + 1);
    }
    return next;
}

GearboxGear prev_gear(GearboxGear g)
{
    GearboxGear prev = g;
    uint8_t idx = (uint8_t)g;
    if (idx > 1 && idx <= 5) { // 2-5
        prev = (GearboxGear)(idx - 1);
    }
    return prev;
}

float ratio_absolute(GearboxGear g, GearboxConfiguration* cfg) {
    switch (g) {
    case GearboxGear::First:
        return cfg->bounds[0].ratio;
    case GearboxGear::Second:
        return cfg->bounds[1].ratio;
    case GearboxGear::Third:
        return cfg->bounds[2].ratio;
    case GearboxGear::Fourth:
        return cfg->bounds[3].ratio;
    case GearboxGear::Fifth:
        return cfg->bounds[4].ratio;
    case GearboxGear::Reverse_First:
        return abs(cfg->bounds[5].ratio);
    case GearboxGear::Reverse_Second:
        return abs(cfg->bounds[6].ratio);
    default:
        return 0;
    }
}

void Gearbox::inc_gear_request()
{
    this->ask_upshift = true;
    this->ask_downshift = false;
    this->manual_shift = true;
}

void Gearbox::dec_gear_request()
{
    this->ask_upshift = false;
    this->ask_downshift = true;
    this->manual_shift = true;
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

// Same as UN52 controller_loop. Going into D uses last_motion_gear (2nd after the engine was off, UN52's
// 'No pressure default' and the gear the 722.6 uses in limp mode). Already in a forward gear, D keeps
// the gear, + / - change it (process_gear_requests).
// R is Reverse_First (UN52 would pick Reverse_Second after 2nd). Reverse_First needs more pressure
// with this calibration, so the pressure is on the safe side whichever reverse the gearbox is in.
GearboxGear Gearbox::requested_gear_from_shifter() {
    GearboxGear ret = this->target_gear;
    switch (this->shifter_pos) {
        case ShifterPosition::D:
        case ShifterPosition::N_D:
        case ShifterPosition::PLUS:
        case ShifterPosition::MINUS:
            // Some shift levers can be semi broken
            // and can switch between ND and D randomly whilst in motion
            // handle that case here by checking current fwd gear first
            // If current gear is also fwd, ignore!
            if (!is_fwd_gear(this->actual_gear) && !is_fwd_gear(this->target_gear)) {
                ret = this->last_motion_gear;
            }
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

// Same as UN52 controller_loop: + / - and the profile's up/downshift checks (Manual profile: only
// downshifts when almost stopped), the forced upshift before redline, and the RPM checks.
// Only in a forward gear, not shifting, and with no gear ratio mismatch.
void Gearbox::process_gear_requests() {
    if (this->engine_running && this->speeds_valid && is_fwd_gear(this->actual_gear))
    {
        // In gear, not shifting, and no ratio mismatch
        if (!this->shifting && this->actual_gear == this->target_gear && gear_disagree_count == 0)
        {
            AbstractProfile* p = this->current_profile;
            // Check if profile is loaded
            if (p != nullptr)
            {
                // Ask the current drive profile if it thinks, given the current
                // data, if the car should up/downshift
                // (No TRRS range restriction, restrict_target is always Fifth with the EWM)
                if (p->should_upshift(this->actual_gear, &this->sensor_data))
                {
                    this->ask_upshift = true; // Upshift takes priority
                    this->manual_shift = false;
                }
                else if (p->should_downshift(this->actual_gear, &this->sensor_data)) {
                    this->ask_downshift = true; // Downshift is secondary
                    this->manual_shift = false;
                }
            }
            if (manual == this->current_profile &&
                this->actual_gear != GearboxGear::Fifth && // Already checked if in FWD gear
                !this->ask_upshift &&
                this->engine_spd_flt > this->engine_spd_flt_prev
            ) {
                GearChange change = GearChange::_1_2;
                if (this->actual_gear == GearboxGear::Second) {
                    change = GearChange::_2_3;
                } else if (this->actual_gear == GearboxGear::Third) {
                    change = GearChange::_3_4;
                } else if (this->actual_gear == GearboxGear::Fourth) {
                    change = GearChange::_4_5;
                }
                int32_t cycles_to_shift = ShiftHelpers::total_time_crossover_shift(this->pressure_mgr, nullptr, change, abs(sensor_data.input_torque), sensor_data.input_rpm);
                int32_t delta_est = ((int32_t)this->engine_spd_flt - (int32_t)this->engine_spd_flt_prev) / 10; // Per 20ms cycle
                int32_t est_rpm_when_shifting = (this->engine_spd_flt/10) + (delta_est * cycles_to_shift);
                if (
                    est_rpm_when_shifting > (int32_t)this->redline_rpm - (int32_t)SBS_CURRENT_SETTINGS.redline_offset_auto_upshift &&
                    sensor_data.pedal_pos > 50
                ) {
                    this->ask_upshift = true;
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
                int32_t prev_rpm = calc_input_rpm_from_req_gear(this->sensor_data.output_rpm, prev, &this->gearboxConfig);
                bool can_downshift = prev_rpm <= (int32_t)this->redline_rpm - 100;
                if (can_downshift)
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
    }
}

// Like UN52 shift_thread()
void Gearbox::update_gear(GearboxGear requested) {
    if (this->pressure_mgr->is_failsafe()) {
        // No shifting in failsafe. The shift solenoids are off and MPC/SPC are at 0mA (max pressure).
        // The shift valves stay where they are, so a forward gear stays engaged. Selecting P/N/R engages
        // whatever the selector says, like limp mode.
        if (this->shifting) {
            this->abort_shift();
            requested = this->requested_gear_from_shifter();
        }
        this->target_gear = requested;
        this->actual_gear = requested;
        return;
    }
    if (this->shifting) {
        if (this->fwd_shifting) {
            this->step_fwd_shift();
        } else {
            this->step_garage_shift();
        }
        return;
    }
    // Forward gear requests (+ / -) are already in target_gear
    if (!(is_fwd_gear(requested) && is_fwd_gear(this->actual_gear))) {
        this->target_gear = requested;
    }
    if (this->target_gear == this->actual_gear) {
        return;
    }
    requested = this->target_gear;
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
    } else if (is_fwd_gear(requested) && is_fwd_gear(this->actual_gear)) {
        // Forward shift logic (Same as UN52, one gear at a time)
        GearChange pgc = GearChange::_IDLE;
        if (requested > this->actual_gear) {
            // Upshifting
            if (requested == GearboxGear::Second) {
                pgc = GearChange::_1_2;
            } else if (requested == GearboxGear::Third) {
                pgc = GearChange::_2_3;
            } else if (requested == GearboxGear::Fourth) {
                pgc = GearChange::_3_4;
            } else if (requested == GearboxGear::Fifth) {
                pgc = GearChange::_4_5;
            }
            this->is_upshift = true;
        } else {
            // Downshifting
            if (requested == GearboxGear::First) {
                pgc = GearChange::_2_1;
            } else if (requested == GearboxGear::Second) {
                pgc = GearChange::_3_2;
            } else if (requested == GearboxGear::Third) {
                pgc = GearChange::_4_3;
            } else if (requested == GearboxGear::Fourth) {
                pgc = GearChange::_5_4;
            }
            this->is_upshift = false;
        }
        if (GearChange::_IDLE == pgc || abs((int16_t)requested - (int16_t)this->actual_gear) != 1) {
            // WTF (More than one gear, never requested by process_gear_requests)
            this->target_gear = this->actual_gear;
        } else {
            this->start_fwd_shift(pgc);
        }
    } else {
        // Ignoring request to change between reverse gears
        this->target_gear = this->actual_gear;
    }
}

void Gearbox::abort_shift() {
    if (this->fwd_shifting) {
        this->fwd_result = 3;
        this->end_fwd_shift(false);
    } else {
        this->gar_result = 2;
        this->end_garage_shift();
    }
}

// First part of UN52 Gearbox::elapse_shift (Everything before the shift loop)
void Gearbox::start_fwd_shift(GearChange req_lookup) {
    // This is important for all EGS compatibility
    uint8_t egs_map_idx_lookup = fwd_gearchange_egs_map_lookup_idx(req_lookup);
    AbstractProfile* profile = this->current_profile;
    if (nullptr == profile || 0xFF == egs_map_idx_lookup) {
        this->target_gear = this->actual_gear;
        return;
    }
    CircuitInfo sd = pressure_mgr->get_basic_shift_data(req_lookup);
    sd.map_idx = egs_map_idx_lookup;
    if (this->last_shift_circuit == sd.shift_circuit) { // Same shift solenoid
        // UN52 waits here, MinTCU tries again next cycle (The normal MPC control keeps running)
        if (GET_CLOCK_TIME() - sensor_data.last_shift_time < 500) {
            return;
        }
    }
    ShiftCharacteristics chars = profile->get_shift_characteristics(req_lookup, &this->sensor_data);
    chars.target_shift_time = MAX(100, chars.target_shift_time);
    this->last_shift_circuit = sd.shift_circuit;
    this->shift_idx = req_lookup;

    FwdShift* f = &this->fwd;
    memset(f, 0, sizeof(FwdShift));
    f->sd = sd;

    pressure_manager->register_shift_pressure_data(&f->p_now);

    f->now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, req_lookup, this->gearboxConfig.bounds);
    Clutch applying = get_clutch_to_apply(req_lookup);
    Clutch releasing = get_clutch_to_release(req_lookup);
    PrefillData prefill_data = pressure_mgr->make_fill_data(applying);

    int32_t MOD_MAX = this->pressure_mgr->get_max_solenoid_pressure();
    int32_t SPC_MAX = pressure_manager->get_max_shift_pressure(egs_map_idx_lookup);

    f->trd = TorqueRequstData {
        .ty = TorqueRequestControlType::None,
        .bounds = TorqueRequestBounds::LessThan,
        .amount = 0
    };

    bool en_trq_req = true;
    if (GearChange::_1_2 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_1_2;
    } else if (GearChange::_2_3 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_2_3;
    } else if (GearChange::_3_4 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_3_4;
    } else if (GearChange::_4_5 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_4_5;
    } else if (GearChange::_2_1 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_2_1;
    } else if (GearChange::_3_2 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_3_2;
    } else if (GearChange::_4_3 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_4_3;
    } else if (GearChange::_5_4 == req_lookup) {
        en_trq_req = SBS_CURRENT_SETTINGS.en_trq_req_5_4;
    }
    // MinTCU only listens on the CAN bus, so the engine never gets a torque request. Tell the shift
    // algorithms, then they make up for the torque that is not taken away (UN52 does the same when
    // torque requests are switched off in the settings)
    en_trq_req = false;

    f->sid = ShiftInterfaceData {
        .profile = profile,
        .MOD_MAX = MOD_MAX,
        .SPC_MAX = SPC_MAX,
        .shift_flags = 0,
        .change = req_lookup,
        .applying = applying,
        .releasing = releasing,
        .curr_g = this->actual_gear,
        .targ_g = this->target_gear,
        .inf = sd,
        .release_spring_on_clutch = pressure_manager->get_spring_pressure(applying),
        .release_spring_off_clutch = pressure_manager->get_spring_pressure(releasing),
        .prefill_info = prefill_data,
        .chars = chars,
        .ptr_r_clutch_speeds = &f->now_cs,
        .ptr_w_pressures = &f->p_now,
        .ptr_w_trq_req = &f->trd,
        .tcc = this->tcc,
        .adaptation_mgr = nullptr, // MinTCU has no adaptation
        .manual_shift = this->shift_req_was_manual,
        .trq_req_en = en_trq_req,
        .diff_ratio = this->diff_ratio_f,
        .max_gb_trq = this->gearboxConfig.max_torque,
        // -1 since bounds table starts at 1st gear, but gear_to_idx_lookup returns 0 as N, not D1
        .ratio_old_gear = this->gearboxConfig.bounds[gear_to_idx_lookup(this->actual_gear)-1].ratio,
        .n3_rpm = this->speed_sensors.n3
    };
    // To set the flag values initially
    ShiftHelpers::calc_shift_flags(&f->sid, &this->sensor_data, true);

    float threshold_torque = VEHICLE_CONFIG.engine_drag_torque/10.0;
    if (is_upshift) {
        f->release = sensor_data.converted_driver_torque <= -threshold_torque/2;
    }
    else {
        bool is_release = true;
        // UN52 uses Gearbox::manual_shift here (Cleared before the shift starts, so always false)
        if (
            ((sensor_data.converted_driver_torque <= threshold_torque && ((f->sid.shift_flags & SHIFT_FLAG_COAST) == 0 || (f->sid.chars.extra_speed && this->shift_req_was_manual))) ||
                (
                    (f->sid.shift_flags & SHIFT_FLAG_COAST_54_43) != 0
                )
            )
        ) {
            is_release = false;
        }
        f->release = is_release;
    }
    if (f->release) {
        f->algo = new ReleasingShift(&f->sid);
    } else {
        f->algo = new CrossoverShift(&f->sid);
    }
    f->algo_phase_id = 0;
    f->total_elapsed = 0;
    f->phase_elapsed = 0;
    this->shifting = true;
    this->fwd_shifting = true;
    this->step_fwd_shift();
}

// One loop of the UN52 elapse_shift shift loop
void Gearbox::step_fwd_shift() {
    FwdShift* f = &this->fwd;
    f->sid.n3_rpm = this->speed_sensors.n3;
    bool stationary_shift = this->is_stationary();
    // Shifter moved mid shift!
    if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
        this->fwd_result = 3;
        this->end_fwd_shift(false);
        return;
    }

    int32_t abs_input_torque = abs((int32_t)sensor_data.input_torque);
    f->now_cs = ClutchSpeedModel::get_shifting_clutch_speeds(this->speed_sensors, f->sid.change, this->gearboxConfig.bounds);

    // (UN52 sends the torque request here. MinTCU does not, see start_fwd_shift)

    // Algorithm has control
    uint8_t step_result = f->algo->step(
        f->algo_phase_id,
        abs_input_torque,
        stationary_shift,
        is_upshift,
        f->phase_elapsed,
        f->total_elapsed,
        this->pressure_mgr,
        &this->sensor_data
    );

    // Update pressures (update_pressures() is called by controller_loop)
    pressure_mgr->set_target_modulating_pressure(f->p_now.mod_sol_req);
    pressure_mgr->set_target_shift_pressure(f->p_now.shift_sol_req);

    if (step_result == 0) {
        // Continue
        f->phase_elapsed += SHIFT_DELAY_MS;
    }
    else if (step_result == STEP_RES_END_SHIFT) {
        // Shift completed OK!
        this->fwd_result = 1;
        this->end_fwd_shift(true);
        return;
    }
    else if (step_result == STEP_RES_FAILURE) {
        // Shift failure!
        this->fwd_result = 2;
        this->end_fwd_shift(false);
        return;
    }
    else {
        // Phase has completed, update our data
        f->phase_elapsed = 0;
        if (step_result == STEP_RES_NEXT) {
            f->algo_phase_id += 1;
        }
        else {
            f->algo_phase_id = step_result;
        }
    }
    f->total_elapsed += SHIFT_DELAY_MS;
}

// End of UN52 Gearbox::elapse_shift
void Gearbox::end_fwd_shift(bool result) {
    FwdShift* f = &this->fwd;
    if (result) { // Only set gear on conformation!
        this->actual_gear = gear_from_idx(f->sd.targ_g);
        this->last_motion_gear = this->actual_gear;
    }
    else {
        if (!is_shifter_in_valid_drive_pos(this->shifter_pos)) {
            // Shift failed due to selector moving
            this->target_gear = GearboxGear::Neutral;
            this->actual_gear = GearboxGear::Neutral;
        }
        else {
            // Shift failed!
            this->target_gear = this->actual_gear;
        }
    }
    pressure_manager->set_spc_p_max();
    pressure_manager->set_shift_circuit(f->sd.shift_circuit, false);
    pressure_manager->notify_shift_end();
    this->sensor_data.last_shift_time = GET_CLOCK_TIME();
    delete f->algo;
    f->algo = nullptr;
    this->shift_idx = GearChange::_IDLE;
    this->shifting = false;
    this->fwd_shifting = false;
    this->is_upshift = false;
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
        this->output_estimated = false;
    }
    else if (VEHICLE_CONFIG.input_sensor_pulses_per_rev == 0 && ok) {
        // MinTCU: no output speed sensor fitted. In gear nothing slips between the input and output
        // shaft, so output = input / gear ratio. During an up/downshift the ratio changes, so the
        // value carries on at the rate it was changing before the shift for up to 1 second, then it is
        // held (The shift algorithms need the output speed to see the clutches sync, holding it still
        // breaks that if the car speeds up or slows down during the shift).
        // In P/N it cannot be known, it is 0 (Same as before, the garage shift then expects the car to
        // be stopped)
        if (!this->fwd_shifting) {
            float ratio = ratio_absolute(this->actual_gear, &this->gearboxConfig);
            float est = 0;
            if (ratio > 0 && is_controllable_gear(this->actual_gear)) {
                est = (float)speed_sensors.turbine / ratio;
                // Change per cycle, filtered (N2/N3 move in steps)
                this->est_output_change = (this->est_output_change * 0.9f) + ((est - this->est_output) * 0.1f);
            } else {
                this->est_output_change = 0;
            }
            this->est_output = est;
        } else if (this->fwd.total_elapsed < 1000) {
            this->est_output = MAX(0.0f, this->est_output + this->est_output_change);
        }
        speed_sensors.output = this->est_output;
        this->output_estimated = true;
    }
    else {
        ok = false; // Output RPM failed
    }

    return ok;
}

// Same as UN52
bool Gearbox::calcGearFromRatio(bool is_reverse)
{
    float ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
    if (is_reverse)
    {
        ratio *= -1;
        for (uint8_t i = 0; i < 2; i++)
        { // Scan the 2 reverse gears (stored in bounds[5] and bounds[6])
            GearRatioInfo limits = gearboxConfig.bounds[i + 5];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                // est_gear_idx must match gear_from_idx(): 6 = Reverse_First, 7 = Reverse_Second
                this->est_gear_idx = i + 6;
                return true;
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 5; i++)
        { // Scan the 5 forwards gears
            GearRatioInfo limits = gearboxConfig.bounds[i];
            if (ratio >= limits.ratio_min_drift && ratio <= limits.ratio_max_drift)
            {
                this->est_gear_idx = i + 1;
                return true;
            }
        }
    }
    this->est_gear_idx = 0;
    return false;
}

// Same as UN52, in int32_t
void Gearbox::process_acceleration() {
    if (UINT16_MAX != sensor_data.output_rpm) {
        int32_t wheel_spd_now = (((float)sensor_data.output_rpm*100) / this->diff_ratio_f);
        // Rpm -> Rps = RPM/60
        // Rps -> Rp/cycle = Rps/50
        int32_t wheel_delta = ((int32_t)this->wheel_spd-(int32_t)this->wheel_spd_prev);
        int32_t wheel_accel_m = (wheel_delta * (int32_t)VEHICLE_CONFIG.wheel_circumference)/300; // mm/sec delta
        // Rotate values
        this->wheel_spd_prev = this->wheel_spd;
        this->wheel_spd = wheel_spd_now;
        if (sensor_data.output_rpm < 20) {
            wheel_accel_m = 0;
            acceleration_ms2 = 0;
        } else {
            acceleration_ms2 = first_order_filter(10, wheel_accel_m*100, this->acceleration_ms2);
        }
    } else {
        wheel_spd = 0;
        wheel_spd_prev = 0;
        acceleration_ms2 = 0;
    }
}

// Same as UN52, in int32_t
void Gearbox::process_motor_spd_filtered() {
    if (UINT16_MAX != sensor_data.engine_rpm) {
        this->engine_spd_flt_prev = this->engine_spd_flt;
        this->engine_spd_flt = first_order_filter(8, (int32_t)sensor_data.engine_rpm*10, engine_spd_flt);
    }
}

void Gearbox::controller_loop() {
    uint32_t start = GET_CLOCK_TIME();
    if (start - this->last_frame_time < GEARBOX_FRAME_MS) {
        return; // should run at 50 hz, unless something else takes too long
    }
    this->last_frame_time = start;

    TCUIO::update_io_layer();

    // Speed sensors (Same as UN52). MinTCU also copies the input RPM when the output RPM is missing,
    // the garage shift only needs the input RPM
    this->speeds_valid = this->process_speed_sensors();
    this->sensor_data.input_rpm = this->speed_sensors.turbine;
    if (this->speeds_valid)
    {
        this->sensor_data.output_rpm = speed_sensors.output;
        bool stationary = this->is_stationary();
        this->process_acceleration();
        this->sensor_data.acceleration_ms2 = this->acceleration_ms2/10;
        this->sensor_data.wheel_speed_mps = this->wheel_spd;
        if (!stationary)
        {
            // Store our ratio
            if (this->actual_gear == GearboxGear::Neutral || this->actual_gear == GearboxGear::Park || this->actual_gear == GearboxGear::SignalNotAvailable) {
                this->sensor_data.gear_ratio = 0.0;
            } else {
                this->sensor_data.gear_ratio = (float)this->sensor_data.input_rpm / (float)this->sensor_data.output_rpm;
            }
            this->sensor_data.targ_gear_ratio = ratio_absolute(this->actual_gear, &this->gearboxConfig);
        }
        else {
            // Stationary so no ratios
            this->sensor_data.gear_ratio = 0.0;
            this->sensor_data.targ_gear_ratio = 0.0;
        }
        // Gear check from the ratio. Not possible with the estimated output RPM, it always matches the gear
        if (!shifting && sensor_data.output_rpm > 100 && !this->output_estimated)
        {
            if (is_fwd_gear(this->actual_gear))
            {
                if (calcGearFromRatio(false) && this->est_gear_idx != 0)
                {
                    // Compare gears
                    GearboxGear estimate = gear_from_idx(this->est_gear_idx);
                    if (estimate != this->actual_gear)
                    {
                        gear_disagree_count++;
                        if (gear_disagree_count >= 50)
                        {
                            this->actual_gear = estimate; // DID NOT SHIFT!
                            this->target_gear = estimate;
                            this->last_motion_gear = estimate;
                            gear_disagree_count = 0;
                        }
                    }
                    else
                    {
                        gear_disagree_count = 0;
                    }
                }
            }
            else
            {
                gear_disagree_count = 0;
            }
        }
        else
        {
            gear_disagree_count = 0;
        }
    }
    else
    {
        if (!this->output_estimated && TCUIO::output_rpm() == UINT16_MAX) {
            this->sensor_data.output_rpm = 0;
        }
        this->sensor_data.gear_ratio = 0.0;
        gear_disagree_count = 0;
    }

    // Pedal (Same as UN52)
    uint8_t p_tmp = egs_can_hal->get_pedal_value(1000);
    if (p_tmp == 0xFF)
    {
        p_tmp = 250 / 4; // 25% as a fallback
    }
    int16_t pedal_delta = ((int16_t)p_tmp - (int16_t)this->pedal_last)*50; // Per second
    this->sensor_data.pedal_delta_per_second = first_order_filter(3, (float)pedal_delta/2.5, this->sensor_data.pedal_delta_per_second);
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
        // UN52 uses input RPM == 0 for this (2.5 seconds). The converter drags the turbine at a
        // standstill in gear, so MinTCU uses the engine RPM
        this->engine_running_off_counter += 1;
        if (this->engine_running_off_counter >= 50) { // 1 second
            this->engine_running = false;
            // Same as UN52 when the engine stops
            if (this->shifting) {
                this->abort_shift();
            }
            this->last_motion_gear = GearboxGear::Second; // No pressure default
            this->actual_gear = GearboxGear::Neutral;
            this->target_gear = GearboxGear::Neutral;
        }
    }

    this->process_motor_spd_filtered();

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
    if (this->shifter_pos != this->shifter_pos_last)
    {
        // Same as UN52 main.cpp: Same position is ignored, process last request of the user
        switch (this->shifter_pos)
        {
        case ShifterPosition::PLUS:
            this->inc_gear_request();
            break;
        case ShifterPosition::MINUS:
            this->dec_gear_request();
            break;
        default:
            break;
        }
        this->shifter_pos_last = this->shifter_pos;
    }
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

    // + / - and gear changes
    if (this->pressure_mgr->is_failsafe()) {
        // Not in UN52: a + / - from before or during failsafe must not shift later
        this->ask_upshift = false;
        this->ask_downshift = false;
        this->manual_shift = false;
    } else {
        this->process_gear_requests();
    }
    this->update_gear(requested);

    if (this->fwd_shifting) {
        // The shift algorithm sets MPC and SPC (Same as UN52 elapse_shift)
        pressure_mgr->update_pressures(this->fwd.sid.curr_g, this->fwd.sid.change);
    } else if (this->shifting) {
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
    if (TCUIO::output_rpm() != UINT16_MAX || this->output_estimated) {
        Serial.print(this->sensor_data.output_rpm);
        Serial.print(this->output_estimated ? "e" : ""); // e = estimated (No output speed sensor)
    } else {
        Serial.print(-1);
    }
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
    if (this->shifting && !this->fwd_shifting) {
        Serial.print('S');
        Serial.print(this->gar.stage);
        Serial.print('.');
        Serial.print(this->gar.substage);
    } else {
        Serial.print(this->gar_result);
    }
    Serial.print(',');
    if (this->fwd_shifting) {
        // C = crossover, R = release, then the gear change (1 = 1-2 ... 4 = 4-5, 5 = 2-1 ... 8 = 5-4) and the phase
        Serial.print(this->fwd.release ? 'R' : 'C');
        Serial.print((uint8_t)this->fwd.sid.change);
        Serial.print('.');
        Serial.print(this->fwd.algo_phase_id);
    } else {
        Serial.print(this->fwd_result);
    }
    Serial.print(',');
    Serial.print(this->failsafe_reasons, HEX);
    Serial.print(',');
    Serial.println(egs_can_hal->get_rx_overflow_count());
}

Gearbox* gearbox = nullptr;
