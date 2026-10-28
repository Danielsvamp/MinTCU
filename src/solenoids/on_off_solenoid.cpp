#include "on_off_solenoid.h"
#include "nvs/module_settings.h"
#include "clock.hpp"
#include <tcu_maths.h>
#include <Arduino.h>

OnOffSolenoid::OnOffSolenoid(const char* name, uint8_t pwm_pin, uint32_t on_time_ms, uint16_t hold_pwm)
: name(name), pin(pwm_pin), target_on_time(on_time_ms), target_hold_pwm(hold_pwm) {
}

void OnOffSolenoid::update(uint16_t battery_mv) {
    // Same as UN52 OnOffSolenoid::__write_pwm, UN52 PWM is 12 bit (0-4095), analogWrite is 8 bit
    uint8_t duty = 0;
    if (this->state) {
        // Solenoid should be on!
        if (this->holding || GET_CLOCK_TIME() - this->on_time_ms >= this->target_on_time) {
            this->holding = true;
            float vref_compensation = 1.0;
            if (UINT16_MAX != battery_mv && 0 != battery_mv) {
                vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)battery_mv;
            }
            uint32_t calc_pwm = (uint32_t)((float)this->target_hold_pwm * vref_compensation);
            duty = MIN(255UL, calc_pwm / 16UL);
        } else {
            duty = 255;
        }
    }
    // analogWrite(pin, 0) / 255 switch the pin fully low / high
    analogWrite(this->pin, duty);
}

void OnOffSolenoid::on() {
    if (!this->state) {
        this->on_time_ms = GET_CLOCK_TIME();
        this->holding = false;
    }
    this->state = true;
}

void OnOffSolenoid::off() {
    this->on_time_ms = 0;
    this->state = false;
    this->holding = false;
}
