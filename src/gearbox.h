// Here we go, gearbox controller code! Lets go!

#ifndef GEARBOX_H
#define GEARBOX_H

#include <stdint.h>
#include "canbus/can_hal.h"
#include "solenoids/solenoids.h"
#include "sensors.h"
//#include "profiles.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "common_structs.h"
//#include "torque_converter.h"
#include "pressure_manager.h"
#include "models/input_torque.hpp"
//#include "adaptation/shift_adaptation.h"
//#include "models/clutch_speed.hpp"
#include "shifter/shifter.h"
//#include "inputcomponents/brakepedal.hpp"
//#include "inputcomponents/kickdownswitch.hpp"
#include "driver_dynamics/dynamics.h"
//#include "runtime_sensors/runtime_sensors.h"

struct PostShiftTorqueRamp {
    bool enabled;
    uint16_t start_nm;
    uint16_t time_to_exit;
};

class Gearbox {
public:
    explicit Gearbox(Shifter* shifter);
    SensorData sensor_data;

    bool shifting = false;
    
    PressureManager* pressure_mgr = nullptr;

    void controller_loop(void);

private:

    GearboxGear target_gear = GearboxGear::Park;
    GearboxGear actual_gear = GearboxGear::Park;
    GearboxGear last_fwd_gear = GearboxGear::Second;

    bool start_second = true; // By default

    [[noreturn]]
    static void start_controller_internal(void *_this) {
        static_cast<Gearbox*>(_this)->controller_loop();
    }
    uint16_t temp_raw = 0;
    uint8_t pedal_last = 0;
    uint16_t input_last = 0;
    bool ask_upshift = false;
    bool ask_downshift = false;
    bool manual_shift = false;
    bool shift_req_was_manual = false;
    bool is_upshift = false;
    bool fwd_gear_shift = false;
    float tcc_percent = 0.F;
    uint8_t est_gear_idx = 0;
    uint16_t curr_hold_pressure = 0;
    bool show_upshift = false;
    bool show_downshift = false;
    bool flaring = false;
    int gear_disagree_count = 0;
    unsigned long last_tcc_adjust_time = 0;
    int mpc_working = 0;
    bool diag_stop_control = false;
    Shifter* shifter = nullptr;
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvailable;
    GearboxConfiguration gearboxConfig;
    ShiftCircuit last_shift_circuit = ShiftCircuit::None;
    float diff_ratio_f =  1.0;
    GearChange shift_idx = GearChange::_IDLE;
    bool abort_shift = false;
    bool aborting = false;
    GearboxGear restrict_target = GearboxGear::Fifth;
    GearboxGear last_motion_gear = GearboxGear::Second;
    
    float pedal_average_ff = 0;
    
    DeltaTracker* input_rpm_delta = nullptr;
    DeltaTracker* pedal_delta = nullptr;
    uint32_t last_delta_time = 0;

    int req_static_torque_delta = 0;
    bool freeze_torque = false;

    int32_t cached_input_rpm = 0;
    int32_t cached_engine_rpm = 0;
    int32_t cached_output_rpm = 0;
};

extern Gearbox* gearbox;

#endif