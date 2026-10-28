#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdint.h>
#include "cc_solenoid.h"

// Timer1 fast PWM, TOP = ICR1. 16MHz / (1 * (15999 + 1)) = 1kHz, same frequency as UN52
#define SOLENOID_PWM_TOP 15999u

namespace Solenoids {
    /**
     * @brief Sets up Timer1 for 1kHz PWM on pins 11 (MPC) and 12 (SPC), both at 0% duty
     *
     * Y3, Y4, Y5 and TCC stay off (initPins), so the gearbox stays in its limp gear (2nd / R).
     */
    void init_all_solenoids(void);
}

extern ConstantCurrentSolenoid *sol_mpc;
extern ConstantCurrentSolenoid *sol_spc;

#endif // SOLENOID_H
