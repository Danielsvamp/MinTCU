#ifndef ON_OFF_SOLENOID_H
#define ON_OFF_SOLENOID_H

#include <stdint.h>

/**
 * Shift solenoid (Y3/Y4/Y5), ported from UN52 on_off_solenoid.cpp.
 * When switched on it gets full power for on_time_ms, then it is held at hold_pwm (out of 4096,
 * at cc_vref_solenoid battery voltage, scaled for the real battery voltage), same as UN52.
 *
 * Uses analogWrite(), so the pin must be a PWM pin that is not on Timer1 (Timer1 = MPC/SPC).
 */
class OnOffSolenoid {
public:
    OnOffSolenoid(const char* name, uint8_t pwm_pin, uint32_t on_time_ms, uint16_t hold_pwm);
    void on();
    void off();
    bool is_on() const { return this->state; }
    /**
     * @brief Call every gearbox cycle (20ms) with the battery voltage (UINT16_MAX if not available)
     */
    void update(uint16_t battery_mv);
private:
    const char* name;
    uint8_t pin;
    bool state = false;
    bool holding = false;
    uint32_t on_time_ms = 0;
    uint32_t target_on_time;
    uint16_t target_hold_pwm;
};

#endif
