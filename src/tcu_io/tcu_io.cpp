//
// Created by ashcon on 9/2/24.
// Ported to the Mega2560. All 'value*100' maths is done in int32_t, int is only 16 bit on AVR!
//

#include "tcu_io.hpp"
#include "sensors.h"
#include <tcu_maths.h>

// How many 20ms cycles a sensor may be missing before it is reported as not available
#define ATF_MAX_ERROR_CYCLES 10   // 200ms
#define VBATT_MAX_ERROR_CYCLES 5  // 100ms

TCUIO::SmoothedSensor smoothed_sensor_atf_temp;
TCUIO::SmoothedSensor smoothed_sensor_vbatt;

TCUIO::OnePollSensor<uint8_t> onepoll_parking_lock;
TCUIO::OnePollSensor<int16_t> onepoll_motor_temperature;

SensorDataRaw raw_sensors;

void init_smoothed_sensor(TCUIO::SmoothedSensor* dest, uint8_t buffer_size, int32_t reset_value = 0) {
    // Start in the error state, so nothing is reported until a real value is seen
    dest->e_counter = 254;
    dest->sample_count = buffer_size;
    dest->last_value = reset_value*100;
}

template <typename T>
void init_onepoll(TCUIO::OnePollSensor<T>* dest, T reset_value) {
    dest->e_counter = 254;
    dest->current_value = reset_value;
}

// UN52 uses std::numeric_limits<T>::max() as the error value, which AVR does not have,
// so the error value is passed in instead
void add_to_smoothed_sensor(TCUIO::SmoothedSensor* dest, int32_t value, int32_t error_value, bool force_reset = false) {
    if (error_value == value) {
        // Error condition
        if (dest->e_counter < 254) {
            dest->e_counter++; // Increase cycle error counter
        }
    } else {
        if (0 != dest->e_counter || force_reset) { // There 'was' an error, but now the value is seen. Reset the value
            dest->last_value = value*100;
        } else {
            dest->last_value = first_order_filter(dest->sample_count, value*100, dest->last_value);
        }
        dest->e_counter = 0;
    }
}

template <typename T>
void add_to_onepoll_sensor(TCUIO::OnePollSensor<T>* dest, T value, T error_value) {
    if (error_value == value) {
        // Error condition
        if (dest->e_counter < 254) {
            dest->e_counter++; // Increase cycle error counter
        }
    } else {
        dest->current_value = value;
        dest->e_counter = 0;
    }
}

template <typename T>
inline T get_onepoll_sensor_val(TCUIO::OnePollSensor<T>* src, uint8_t ecounter_max, T error_value) {
    return (src->e_counter <= ecounter_max) ? src->current_value : error_value;
}

inline uint16_t get_smoothed_sensor_val_unsigned(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    uint16_t ret;
    if (src->e_counter <= ecounter_max) {
        ret = MAX(0, MIN((int32_t)UINT16_MAX - 1, src->last_value/100));
    } else {
        ret = UINT16_MAX;
    }
    return ret;
}

inline int16_t get_smoothed_sensor_val_signed(TCUIO::SmoothedSensor* src, uint8_t ecounter_max) {
    int16_t ret;
    if (src->e_counter <= ecounter_max) {
        ret = MAX((int32_t)INT16_MIN, MIN((int32_t)INT16_MAX - 1, src->last_value/100));
    } else {
        ret = INT16_MAX;
    }
    return ret;
}

void TCUIO::setup_io_layer() {
    init_smoothed_sensor(&smoothed_sensor_atf_temp, 12, 25); //250ms/20ms
    init_smoothed_sensor(&smoothed_sensor_vbatt, 12, 12000); // 250ms/20ms
    init_onepoll<uint8_t>(&onepoll_parking_lock, UINT8_MAX);
    init_onepoll<int16_t>(&onepoll_motor_temperature, INT16_MAX);
}

bool was_reading_from_engine = false;
void update_tft_sensor() {
    // Same as UN52:
    // parking_lock == UINT8_MAX -> True (Since PLL is not readable, just use engine temp)
    // parking_lock == 1 -> True (Since PLL is engaged, just use engine temp)
    // parking_lock == 0 -> False (We CAN use TFT temp!)
    bool atf_from_engine_temp = raw_sensors.parking_lock != 0;
    add_to_onepoll_sensor<uint8_t>(&onepoll_parking_lock, raw_sensors.parking_lock, UINT8_MAX);

    bool reset_average = was_reading_from_engine != atf_from_engine_temp; // State change
    int16_t temperature;
    if (atf_from_engine_temp) {
        // Use coolant temperature from CAN (INT16_MAX if missing)
        temperature = TCUIO::motor_temperature();
    } else {
        // Use TFT value
        temperature = raw_sensors.atf_temp_c;
    }
    // Temperature might be INT16_MAX (Something wrong with the signal)
    add_to_smoothed_sensor(&smoothed_sensor_atf_temp, temperature, INT16_MAX, reset_average);
    was_reading_from_engine = atf_from_engine_temp;
}

void update_can_values() {
    // Motor coolant temperature (Used as a sub for ATF temperature)
    int16_t coolant = INT16_MAX;
    if (nullptr != egs_can_hal) {
        coolant = egs_can_hal->get_engine_coolant_temp(1000);
    }
    add_to_onepoll_sensor<int16_t>(&onepoll_motor_temperature, coolant, INT16_MAX);
}

void TCUIO::update_io_layer() {
    Sensors::update(&raw_sensors); // 1. Update PCB Hal
    // Poll CAN Layer inputs
    update_can_values();
    // Now do battery voltage
    add_to_smoothed_sensor(&smoothed_sensor_vbatt, raw_sensors.battery_mv, UINT16_MAX);
    // To TFT and parking lock
    update_tft_sensor();
}

uint8_t TCUIO::parking_lock() { return get_onepoll_sensor_val<uint8_t>(&onepoll_parking_lock, 0, UINT8_MAX); }
// UN52 returns last_value here without checking the error counter, so a dead sensor keeps its last value forever
int16_t TCUIO::atf_temperature() { return get_smoothed_sensor_val_signed(&smoothed_sensor_atf_temp, ATF_MAX_ERROR_CYCLES); }
uint16_t TCUIO::battery_mv() { return get_smoothed_sensor_val_unsigned(&smoothed_sensor_vbatt, VBATT_MAX_ERROR_CYCLES); }
int16_t TCUIO::motor_temperature() { return get_onepoll_sensor_val<int16_t>(&onepoll_motor_temperature, 5, INT16_MAX); }
