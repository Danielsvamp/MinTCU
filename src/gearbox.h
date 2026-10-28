// Here we go, gearbox controller code! Lets go!

#ifndef GEARBOX_H
#define GEARBOX_H

#include <stdint.h>
#include "canbus/can_hal.h"
#include "solenoids/solenoids.h"
#include "sensors.h"
#include "common_structs.h"
#include "pressure_manager.h"
#include "models/input_torque.hpp"
#include "models/clutch_speed.hpp"
#include "shifter/shifter.h"
#include "shifter/shifter_ewm.h"
#include "profiles.h"
#include "torque_converter.h"
#include "shifting_algo/s_algo.h"

// Why the gearbox is in failsafe (bit flags, printed in the log)
enum FailsafeReason : uint8_t {
    FS_NONE = 0,
    FS_ENGINE_OFF = 1 << 0,      // Engine RPM missing or below 400
    FS_NO_TORQUE = 1 << 1,       // MS_312 / MS_212 torque frames missing
    FS_NO_SHIFTER = 1 << 2,      // EWM_230 missing or implausible
    FS_RECOVERING = 1 << 3,      // Everything is OK again, waiting before leaving failsafe
    FS_NO_CAL = 1 << 4,          // No (valid) calibration compiled in, see tools/egs_cal_gen.py
};

class Gearbox {
public:
    explicit Gearbox();
    SensorData sensor_data;

    bool shifting = false;

    PressureManager* pressure_mgr = nullptr;

    /**
     * @brief Call from loop(). Runs the gearbox logic every 20ms (50Hz), returns straight away otherwise
     */
    void controller_loop(void);

    // + / - on the selector (Same as UN52)
    void inc_gear_request();
    void dec_gear_request();
    void set_profile(AbstractProfile* prof);

private:
    GearboxGear requested_gear_from_shifter();
    void process_gear_requests();
    void update_gear(GearboxGear requested);
    void start_fwd_shift(GearChange change);
    void step_fwd_shift();
    void end_fwd_shift(bool result);
    void abort_shift();
    void start_garage_shift(GearboxGear target);
    void step_garage_shift();
    void end_garage_shift();
    void shift_to_n_p(GearboxGear target);
    bool process_speed_sensors();
    bool calcGearFromRatio(bool is_reverse);
    void process_acceleration();
    void process_motor_spd_filtered();
    bool is_stationary();
    void print_log();

    GearboxGear target_gear = GearboxGear::Neutral;
    GearboxGear actual_gear = GearboxGear::Neutral;

    uint8_t pedal_last = 0;
    int mpc_working = 0;
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvailable;
    GearboxConfiguration gearboxConfig;
    SpeedSensors speed_sensors;
    bool sensors_ok = false;
    bool speeds_valid = false;
    GearboxGear last_motion_gear = GearboxGear::Second;

    // UN52 runs the garage shift as a loop in its own task (Gearbox::shift_thread, 20ms per loop).
    // MinTCU has no tasks, so the loop variables live here and step_garage_shift() runs one loop
    // every gearbox cycle.
    struct GarageShift {
        GearboxGear target;
        bool into_reverse;
        Clutch applying;
        GearChange circuit;
        uint16_t centrifugal;
        uint16_t spring_p;
        uint16_t p_mod;
        uint16_t p_shift;
        uint16_t p_apply_clutch;
        uint16_t p_max_apply_clutch;
        uint8_t stage;
        uint8_t substage;
        uint8_t timer_s;
        uint8_t timer_m;
        bool completed_ok;
        bool jump_to_pid;
        int32_t rpm_delta;
    } gar;
    // Last garage shift: 0 = none yet, 1 = OK, 2 = aborted, 3 = skipped (no input speed, engaged like limp mode)
    uint8_t gar_result = 0;

    // UN52 runs up/downshifts in Gearbox::elapse_shift (shift thread, 20ms per loop).
    // Same as the garage shift, the loop variables live here and step_fwd_shift() runs one loop
    // every gearbox cycle.
    struct FwdShift {
        ShiftingAlgorithm* algo;
        ShiftInterfaceData sid;
        CircuitInfo sd;
        ShiftPressures p_now;
        ShiftClutchData now_cs;
        TorqueRequstData trd;
        uint8_t algo_phase_id;
        uint32_t total_elapsed;
        uint32_t phase_elapsed;
        bool release;
    } fwd;
    bool fwd_shifting = false;
    // Last up/downshift: 0 = none yet, 1 = OK, 2 = failed, 3 = aborted (selector moved / failsafe)
    uint8_t fwd_result = 0;
    GearChange shift_idx = GearChange::_IDLE;
    bool is_upshift = false;
    ShiftCircuit last_shift_circuit = ShiftCircuit::None;
    bool ask_upshift = false;
    bool ask_downshift = false;
    bool manual_shift = false;
    bool shift_req_was_manual = false;
    ShifterPosition shifter_pos_last = ShifterPosition::SignalNotAvailable;
    AbstractProfile* current_profile = nullptr;
    ShifterEwm* shifter = nullptr;
    // Set when an upshift happens with kickdown held. Kickdown cannot request a
    // downshift again until it is re-armed (See kickdown_rearm_rpm_offset)
    bool kickdown_rearm_pending = false;
    TorqueConverter* tcc = nullptr;
    uint16_t redline_rpm = 4000;
    float diff_ratio_f = 1.0;
    // MinTCU: no output speed sensor, sensor_data.output_rpm is calculated from the input speed
    // and the gear ratio (carried on at the rate from before the shift during a shift)
    bool output_estimated = false;
    float est_output = 0;
    float est_output_change = 0; // RPM per gearbox cycle
    uint8_t est_gear_idx = 0;
    uint8_t gear_disagree_count = 0;
    uint32_t wheel_spd = 0;
    uint32_t wheel_spd_prev = 0;
    int32_t acceleration_ms2 = 0;
    uint32_t engine_spd_flt = 0;
    uint32_t engine_spd_flt_prev = 0;

    bool engine_running = false;
    uint8_t engine_rpm_missing_cycles = 0;
    uint8_t engine_running_off_counter = 0;

    uint8_t failsafe_reasons = FS_ENGINE_OFF;
    uint8_t failsafe_ok_cycles = 0;
    bool atf_substituted = false;
    bool cal_ok = false;

    uint32_t last_frame_time = 0;
    uint8_t log_counter = 0;
};

extern Gearbox* gearbox;

#endif
