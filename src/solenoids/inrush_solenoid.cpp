#include "inrush_solenoid.h"
#include <Arduino.h>
#include <util/atomic.h>

// AT 12.0V (Same as UN52)
const uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
// UN52 times are in 10MHz timer ticks (15000 = 1.5ms, 100000 = 10ms), MinTCU uses 100us ticks
const uint16_t INRUSH_TIME_TICKS = 15;
const uint16_t TOTAL_PERIOD_TICKS = 100; // Hydralic PWM is 100Hz

// Timer3 fast PWM, TOP = ICR3. 16MHz / (1 * (1599 + 1)) = 10kHz (UN52 TCC LEDC frequency)
#define TCC_PWM_TOP 1599u

static InrushControlSolenoid* isr_solenoid = nullptr;
static volatile uint16_t ticks_left = 0;

ISR(TIMER3_OVF_vect) {
    if (ticks_left > 1) {
        ticks_left -= 1;
    } else if (nullptr != isr_solenoid) {
        ticks_left = isr_solenoid->on_timer_interrupt();
    }
}

static void write_pwm(uint16_t pwm_4096) {
    if (0 == pwm_4096) {
        TCCR3A &= ~_BV(COM3A1); // Pin is LOW (OCR = 0 would still give a short pulse)
        OCR3A = 0;
    } else {
        OCR3A = (uint16_t)(((uint32_t)pwm_4096 * (TCC_PWM_TOP + 1)) / 4096UL);
        TCCR3A |= _BV(COM3A1);
    }
}

InrushControlSolenoid::InrushControlSolenoid(const char* name, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: name(name), target_hold_current(target_hold_current_ma) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR3B = 0;
        TCCR3A = 0;
        TCNT3 = 0;
        OCR3A = 0;
        ICR3 = TCC_PWM_TOP;
        // Mode 14 (Fast PWM, TOP = ICR3), output connected by write_pwm()
        TCCR3A = _BV(WGM31);
        TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);
        isr_solenoid = this;
        ticks_left = 0;
    }
    // Pin is already an output and LOW (initPins)
}

void InrushControlSolenoid::isr_disable() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TIMSK3 &= ~_BV(TOIE3);
        write_pwm(0);
        this->isr_disabled = true;
    }
}

void InrushControlSolenoid::isr_enable() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ticks_left = 0;
        this->phase_id = 2;
        TIMSK3 |= _BV(TOIE3);
        this->isr_disabled = false;
    }
}

bool InrushControlSolenoid::is_disabled() {
    return this->isr_disabled;
}

// Same as UN52 InrushControlSolenoid::on_timer_interrupt
uint16_t InrushControlSolenoid::on_timer_interrupt() {
    uint16_t ret = 0;
    uint16_t write = 0;
    // Special handling for Min/Max PWM
    if (this->pwm_raw < INRUSH_START_PWM) {
        write = 0;
        ret = TOTAL_PERIOD_TICKS;
    } else if (this->pwm_raw > INRUSH_SKIP_PWM) {
        write = this->calc_hold_pwm;
        ret = TOTAL_PERIOD_TICKS;
    } else {
        if (this->phase_id == 0) { // Off -> Inrush
            write = 4096;
            // Grab all values now
            this->inrush_time_this_cycle = this->inrush_time;
            this->hold_time_this_cycle = this->hold_time;
            this->off_time_this_cycle = this->off_time;
            ret = this->inrush_time_this_cycle;
            this->phase_id = this->hold_time_this_cycle == 0 ? 2 : 1;
        } else if (this->phase_id == 1) { // Inrush -> hold
            ret = this->hold_time_this_cycle;
            this->phase_id = 2;
            write = this->calc_hold_pwm;
        } else { // Hold -> Off
            write = 0;
            this->phase_id = 0;
            ret = this->off_time_this_cycle;
        }
    }
    write_pwm(write);
    // A phase of 0 ticks (MinTCU rounding) goes straight to the next phase on the next tick
    return ret == 0 ? 1 : ret;
}

// Same as UN52 InrushControlSolenoid::set_duty (Without the zener mode)
void InrushControlSolenoid::set_duty(uint16_t duty) {
    uint16_t period_on_time = (uint16_t)(((uint32_t)duty * (TOTAL_PERIOD_TICKS/2)) / 4096UL);
    uint16_t inrush, hold;
    if (period_on_time > INRUSH_TIME_TICKS) {
        hold = period_on_time - INRUSH_TIME_TICKS;
        inrush = INRUSH_TIME_TICKS;
    } else {
        inrush = period_on_time;
        hold = 0;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        this->pwm_raw = duty;
        this->inrush_time = inrush;
        this->hold_time = hold;
        this->off_time = TOTAL_PERIOD_TICKS - period_on_time;
    }
}
