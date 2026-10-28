#include "cc_solenoid.h"
#include "solenoids.h"
#include "nvs/module_settings.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <tcu_maths.h>

ConstantCurrentSolenoid::ConstantCurrentSolenoid(const char* name, Timer1Channel channel)
    : name(name), channel(channel) {
    this->write_duty(0);
}

void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    this->current_target = target_ma;
}

uint16_t ConstantCurrentSolenoid::get_current_target() const {
    return this->current_target;
}

uint16_t ConstantCurrentSolenoid::get_current_estimate() const {
    return this->current_estimate;
}

uint16_t ConstantCurrentSolenoid::get_duty_permille() const {
    return ((uint32_t)this->duty_ticks * 1000UL) / SOLENOID_PWM_TOP;
}

uint16_t ConstantCurrentSolenoid::get_resistance_mohm() const {
    return this->resistance_mohm;
}

void ConstantCurrentSolenoid::off() {
    this->current_target = 0;
    this->current_estimate = 0;
    this->write_duty(0);
}

void ConstantCurrentSolenoid::update(uint16_t battery_mv, int16_t coil_temp_c) {
    if (
        UINT16_MAX == battery_mv ||
        battery_mv < SOL_CURRENT_SETTINGS.min_batt_cc_mv ||
        battery_mv > SOL_CURRENT_SETTINGS.max_batt_cc_mv ||
        0 == this->current_target
    ) {
        // Like UN52: no (plausible) battery voltage, solenoid off
        this->current_estimate = 0;
        this->write_duty(0);
        return;
    }
    // Unknown coil temperature: assume the reference (cold) temperature. Cold = lower resistance = less duty
    // = less current than wanted = MORE pressure, so this errs on the safe side
    float temp = (INT16_MAX == coil_temp_c) ? SOL_CURRENT_SETTINGS.cc_reference_temp : (float)coil_temp_c;
    float r = SOL_CURRENT_SETTINGS.cc_reference_resistance *
        (1.0 + ((SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires / 100.0) * (temp - SOL_CURRENT_SETTINGS.cc_reference_temp)));
    r = MAX(r, 1.0); // Never divide by something silly

    float v_diode = SOL_CURRENT_SETTINGS.cc_flyback_diode_drop_mv;
    float v_supply = (float)battery_mv - (float)SOL_CURRENT_SETTINGS.cc_driver_drop_mv;
    // mA * Ohm = mV
    float duty = (((float)this->current_target * r) + v_diode) / (v_supply + v_diode);
    duty = MAX(0.0, MIN(duty, 1.0));

    uint16_t ticks = duty * (float)SOLENOID_PWM_TOP;
    this->resistance_mohm = r * 1000.0;
    // Current the model expects for the duty we actually wrote (Lower than target if duty hit 100%)
    float est = ((duty * v_supply) - ((1.0 - duty) * v_diode)) / r;
    this->current_estimate = MAX(0.0, est);
    this->write_duty(ticks);
}

void ConstantCurrentSolenoid::write_duty(uint16_t ticks) {
    ticks = MIN(ticks, SOLENOID_PWM_TOP);
    this->duty_ticks = ticks;
    // 16 bit timer registers must be written with interrupts off (shared TEMP register)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // In fast PWM, OCR = 0 still gives a 1 clock spike every period. For 0% disconnect the
        // timer from the pin, so it is a plain LOW output (set in initPins)
        if (Timer1Channel::A == this->channel) {
            OCR1A = ticks;
            if (0 == ticks) { TCCR1A &= ~_BV(COM1A1); } else { TCCR1A |= _BV(COM1A1); }
        } else {
            OCR1B = ticks;
            if (0 == ticks) { TCCR1A &= ~_BV(COM1B1); } else { TCCR1A |= _BV(COM1B1); }
        }
    }
}
