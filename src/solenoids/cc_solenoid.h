#ifndef __CONSTANT_CURRENT_SOLENOID_H_
#define __CONSTANT_CURRENT_SOLENOID_H_

#include <stdint.h>

/**
 * MPC / SPC solenoid driver.
 *
 * UN52 measures the solenoid current and runs a PID loop on it. MinTCU has no current sensing,
 * so this is OPEN LOOP: the PWM duty that should give the target current is calculated from
 * the battery voltage and the coil resistance at the current ATF temperature.
 *
 * With a flyback diode across the coil, the average coil voltage over one PWM period is
 *   V_avg = D * (V_batt - V_driver) - (1 - D) * V_diode
 * and the average current is V_avg / R(T). Solving for D gives the duty used here:
 *   D = (I * R(T) + V_diode) / (V_batt - V_driver + V_diode)
 *   R(T) = R_ref * (1 + coefficient * (T - T_ref))
 *
 * This is an estimate! Check it with an ammeter in series with the solenoid at least once.
 */
class ConstantCurrentSolenoid {
public:
    enum class Timer1Channel : uint8_t {
        A, // OC1A (Pin 11)
        B, // OC1B (Pin 12)
    };

    explicit ConstantCurrentSolenoid(const char* name, Timer1Channel channel);
    void set_current_target(uint16_t target_ma);
    uint16_t get_current_target() const;

    /**
     * @brief Calculates and writes the new PWM duty. Call every control loop.
     * @param battery_mv Solenoid supply voltage. UINT16_MAX if not available (Solenoid is switched off)
     * @param coil_temp_c Coil temperature (ATF temperature). INT16_MAX if not available
     */
    void update(uint16_t battery_mv, int16_t coil_temp_c);

    /**
     * @brief Forces the solenoid off (0mA = max pressure for MPC/SPC) until the next update with a target
     */
    void off();

    // Estimated current in mA (What the open loop model thinks is flowing)
    uint16_t get_current_estimate() const;
    // PWM duty, 0-1000 (0.1% steps)
    uint16_t get_duty_permille() const;
    // Coil resistance used for the last calculation, in mOhm
    uint16_t get_resistance_mohm() const;
private:
    void write_duty(uint16_t ticks);
    const char* name;
    Timer1Channel channel;
    uint16_t current_target = 0;
    uint16_t current_estimate = 0;
    uint16_t duty_ticks = 0;
    uint16_t resistance_mohm = 0;
};

#endif
