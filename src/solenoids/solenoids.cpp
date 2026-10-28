#include "solenoids.h"
#include "board_config.h"
#include <Arduino.h>
#include <util/atomic.h>

ConstantCurrentSolenoid* sol_mpc = nullptr;
ConstantCurrentSolenoid* sol_spc = nullptr;
OnOffSolenoid* sol_y3 = nullptr;
OnOffSolenoid* sol_y4 = nullptr;
OnOffSolenoid* sol_y5 = nullptr;
InrushControlSolenoid* sol_tcc = nullptr;

void Solenoids::init_all_solenoids() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Stop Timer1 while configuring it
        TCCR1B = 0;
        TCCR1A = 0;
        TCNT1 = 0;
        OCR1A = 0;
        OCR1B = 0;
        ICR1 = SOLENOID_PWM_TOP;
        // Mode 14 (Fast PWM, TOP = ICR1). The outputs (COM1A1 / COM1B1) are only connected
        // by ConstantCurrentSolenoid when the duty is above 0
        TCCR1A = _BV(WGM11);
        // No prescaler
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    }
    // Pins are already outputs and LOW (initPins)
    sol_mpc = new ConstantCurrentSolenoid("MPC", ConstantCurrentSolenoid::Timer1Channel::A);
    sol_spc = new ConstantCurrentSolenoid("SPC", ConstantCurrentSolenoid::Timer1Channel::B);
    // Same on time and hold PWM as UN52 (250ms full on, then 1524/4096 at 12V)
    sol_y3 = new OnOffSolenoid("Y3", toRaw(Pin::y3_pwm), 250, 1524);
    sol_y4 = new OnOffSolenoid("Y4", toRaw(Pin::y4_pwm), 250, 1524);
    sol_y5 = new OnOffSolenoid("Y5", toRaw(Pin::y5_pwm), 250, 1524);
    // Same as UN52 (100Hz, ~700mA for TCC solenoid when holding)
    sol_tcc = new InrushControlSolenoid("TCC", 100, 700, 10);
    sol_tcc->set_duty(0);
    sol_tcc->isr_enable();
}

void Solenoids::update_shift_solenoids(uint16_t battery_mv) {
    sol_y3->update(battery_mv);
    sol_y4->update(battery_mv);
    sol_y5->update(battery_mv);
}

void Solenoids::shift_solenoids_off() {
    sol_y3->off();
    sol_y4->off();
    sol_y5->off();
}
