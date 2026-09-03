#include "solenoids.h"
//#include "esp_log.h"
//#include "esp_adc/adc_cali.h"
//#include "esp_adc/adc_cali_scheme.h"
#include "board_config.h"
#include "../sensors.h"
//#include "soc/i2s_periph.h"
#include "string.h"
//#include "esp_adc/adc_continuous.h"
//#include "esp_adc/adc_oneshot.h"
//#include "esp_check.h"
//#include "../nvs/module_settings.h"
#include "clock.hpp"
//#include "esp_timer.h"
#include "tcu_io/tcu_io.hpp"

ConstantCurrentSolenoid* sol_mpc = nullptr;

bool write_pwm = true;

void update_solenoids(void*) {
    int16_t atf_temp = 250;
    float vref_compensation = 1.0;
    float temp_compensation = 1.0;
    uint16_t vbatt = TCUIO::battery_mv();
    int16_t atf = TCUIO::atf_temperature();
    while (true) {
        vbatt = TCUIO::battery_mv();
        atf = TCUIO::atf_temperature();
        if (UINT16_MAX != vbatt) {
            voltage = vbatt;
            vref_compensation = (float)SOL_CURRENT_SETTINGS.cc_vref_solenoid / (float)voltage;
        }
        else {
            vref_compensation = 1.0;
        }
        if (INT16_MAX != atf) {
            atf_temp = atf * 10.0;
            temp_compensation = (((atf_temp - (SOL_CURRENT_SETTINGS.cc_reference_temp * 10.0)) / 10.0) * SOL_CURRENT_SETTINGS.cc_temp_coefficient_wires) / 10.0;
        }
        if (write_pwm) {
            // MOVED TO CURRENT READING TASK SO READINGS ARE SYNCED
        }
        vTaskDelay(1); // Max we can do at 1000hz
    }
}

float resistance_mpc = 5.0;
float resistance_spc = 5.0;
bool temp_cal = false;
int16_t temp_at_test = 25;

bool routine = false;
bool startup_ok = false;


uint16_t Solenoids::get_solenoid_voltage(void) {
    return voltage;
}

bool Solenoids::init_routine_completed(void) {
    return routine;
}

// bool Solenoids::startup_test_ok() {
//     return startup_ok;
// }

void Solenoids::init_all_solenoids()
{
    SolenoidSetup::init_adc();
    sol_mpc = new ConstantCurrentSolenoid("MPC", ledc_timer_t::LEDC_TIMER_0, pcb_gpio_matrix->mpc_pwm, ledc_channel_t::LEDC_CHANNEL_3, ADC_CHANNEL_6, 1);
}
