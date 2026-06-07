#include <Arduino.h>
#include <stdint.h>

struct SensorData{
    /// Gearbox input RPM
    uint16_t input_rpm;
    /// Engine RPM
    uint16_t engine_rpm;
    /// Output shaft RPM
    uint16_t output_rpm;
    /// Accelerator pedal position. 0-250
    uint8_t pedal_pos;
    /// Accelerator pedal position. 0-250, smoothed to 500ms
    uint8_t pedal_pos_smoothed;
    /// Transmission oil temperature in Celcius
    int16_t atf_temp;
    // Input shaft torque
    int16_t input_torque;
    int16_t converted_torque;
    int16_t converted_driver_torque;
    int16_t indicated_torque;
    /// Engine torque limit maximum in Nm
    int16_t max_torque;
    /// Engine torque limit minimum in Nm
    int16_t min_torque;
    /// Last time the gearbox changed gear (in milliseconds)
    uint32_t last_shift_time;
    /// Current gearbox ratio
    float gear_ratio;
    /// Target gearbox ratio
    float targ_gear_ratio;
    float tcc_trq_multiplier;
    bool kickdown_pressed;
    bool brake_pressed;
};

