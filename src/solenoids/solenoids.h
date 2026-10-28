#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdint.h>
#include "cc_solenoid.h"
#include "on_off_solenoid.h"

// Timer1 fast PWM, TOP = ICR1. 16MHz / (1 * (15999 + 1)) = 1kHz, same frequency as UN52
#define SOLENOID_PWM_TOP 15999u

namespace Solenoids {
    /**
     * @brief Sets up Timer1 for 1kHz PWM on pins 11 (MPC) and 12 (SPC), both at 0% duty,
     * and the Y3/Y4/Y5 shift solenoids (off). TCC stays off.
     */
    void init_all_solenoids(void);
    /**
     * @brief Writes the shift solenoid PWM, call every gearbox cycle
     */
    void update_shift_solenoids(uint16_t battery_mv);
    /**
     * @brief All shift solenoids off (Limp: 2nd in D, R)
     */
    void shift_solenoids_off(void);
}

extern ConstantCurrentSolenoid *sol_mpc;
extern ConstantCurrentSolenoid *sol_spc;
extern OnOffSolenoid *sol_y3;
extern OnOffSolenoid *sol_y4;
extern OnOffSolenoid *sol_y5;

#endif // SOLENOID_H
