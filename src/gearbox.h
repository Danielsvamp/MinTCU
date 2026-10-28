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
#include "shifter/shifter.h"

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

private:
    void update_gear_from_shifter();
    void print_log();

    GearboxGear target_gear = GearboxGear::Neutral;
    GearboxGear actual_gear = GearboxGear::Neutral;

    uint8_t pedal_last = 0;
    int mpc_working = 0;
    ShifterPosition shifter_pos = ShifterPosition::SignalNotAvailable;
    GearboxConfiguration gearboxConfig;

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
