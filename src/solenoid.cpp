#include <Arduino.h>
#include <stdint.h>
#include <common.h>

// solenoids.cpp
void init_all_solenoids(){
    sol_mpc = new ConstantCurrentSolenoid("MPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 1);
    sol_spc = new ConstantCurrentSolenoid("SPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->spc_pwm, ledc_channel_t::LEDC_CHANNEL_4, ADC_CHANNEL_4, 1);
}

extern ConstantCurrentSolenoid *sol_mpc;
extern ConstantCurrentSolenoid *sol_spc;

// pwm_solenoid.h
class PwmSolenoid
{
public:
    /**
     * @brief Construct a new Solenoid
     * 
     * @param name Name of the solenoid (Avaliable from 722.6 PDFs)
     * @param pwm_pin GPIO Pin on the ESP32 to use for PWM control of the solenoid
     * @param frequency Default frequency of the PWM signal for the solenoid
     * @param channel LEDC Channel to use for controlling the solenoids PWM signal
     * @param read_channel The ADC 1 Channel used for current sense feedback
     * @param current_samples The number of samples from I2S for current measuring. It is assumed that each sample is ~2ms snapshot
     */
    PwmSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms);
};

// cc_solenoid.h
class ConstantCurrentSolenoid : public PwmSolenoid {
public:
    explicit ConstantCurrentSolenoid(const char* name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t phase_duration_ms);
    void set_current_target(uint16_t target_ma);
private:
    uint16_t current_target = 0;
};

// cc_solenoid.cpp
void ConstantCurrentSolenoid::set_current_target(uint16_t target_ma) {
    this->current_target = target_ma;
}