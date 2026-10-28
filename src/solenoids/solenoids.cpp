#include "solenoids.h"
#include "board_config.h"
#include <Arduino.h>
#include <util/atomic.h>

ConstantCurrentSolenoid* sol_mpc = nullptr;
ConstantCurrentSolenoid* sol_spc = nullptr;

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
}
