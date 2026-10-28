#ifndef INRUSH_SOLENOID_H
#define INRUSH_SOLENOID_H

#include <stdint.h>

/**
 * TCC solenoid, ported from UN52 inrush_solenoid.cpp (the original mode, without the zener pin).
 *
 * UN52: the solenoid runs at 100Hz (10ms period). Every period it is switched fully on (inrush)
 * for up to 1.5ms, then held at the hold PWM (1024/4096 at 10kHz) for the rest of the on time, then
 * switched off. The on time is duty/4096 of half the period. Below 224/4096 the solenoid stays off,
 * above 3220/4096 it is held at the hold PWM all the time.
 *
 * MinTCU: Timer3 runs the 10kHz PWM on pin 5 (OC3A), and its overflow interrupt (every 100us)
 * runs the 100Hz phases. UN52 times the phases with a 10MHz timer, here they are rounded to 100us
 * (so the on time has 50 steps).
 */
class InrushControlSolenoid {
public:
    InrushControlSolenoid(const char* name, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms);
    // Duty is out of 4096 (Same as UN52)
    void set_duty(uint16_t duty);
    uint16_t get_pwm_raw() const { return this->pwm_raw; }
    void isr_disable();
    void isr_enable();
    bool is_disabled();
    // Called by the Timer3 overflow interrupt, returns the time until the next call (in 100us ticks)
    uint16_t on_timer_interrupt();
private:
    const char* name;
    volatile bool isr_disabled = true;
    // 0 - Inrush
    // 1 - Hold
    // 2 - Off
    uint8_t phase_id = 2; // So we start at 0 again
    uint16_t target_hold_current = 0;
    uint16_t calc_hold_pwm = 1024;
    volatile uint16_t pwm_raw = 0;
    // In 100us ticks
    volatile uint16_t inrush_time = 0;
    volatile uint16_t hold_time = 0;
    volatile uint16_t off_time = 100;
    uint16_t inrush_time_this_cycle = 0;
    uint16_t hold_time_this_cycle = 0;
    uint16_t off_time_this_cycle = 0;
};

#endif
