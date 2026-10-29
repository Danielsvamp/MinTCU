#include "sensors.h"
#include "board_config.h"
#include <tcu_maths.h>
#include <tcu_maths_impl.h>
#include <util/atomic.h>
#include "nvs/eeprom_config.h"

#define N_SENSOR_PULSES_PER_REV 60 // N2 and N3 are 60 pulses per revolution
#define MAX_RPM_PCNT 10000

// UN52 uses the ESP32 pulse counter (PCNT) and counts both edges. The Mega has no free hardware
// counters on usable pins, so each sensor has an external interrupt on its rising edge, which
// counts the pulse and stores the time of the edge.
struct PcntMemData {
    bool init;
    uint32_t last_time_us; // Time of the last edge used in the previous calc_rpm call
    uint16_t pulses_rev;
    uint16_t last_rpm;
    volatile uint16_t count;
    volatile uint32_t edge_time_us;
};

PcntMemData mem_data_n2 = {};
PcntMemData mem_data_n3 = {};
PcntMemData mem_data_out = {};

bool output_rpm_ok = false;

static void isr_n2() { mem_data_n2.count++; mem_data_n2.edge_time_us = micros(); }
static void isr_n3() { mem_data_n3.count++; mem_data_n3.edge_time_us = micros(); }
static void isr_out() { mem_data_out.count++; mem_data_out.edge_time_us = micros(); }

// UN52 divides the pulses by the time between two calls (20ms). With few pulses per 20ms (Output
// sensor with a few pulses/rev) that jumps a lot, so this measures from edge to edge instead.
// With no pulse in this call the last value is kept, but never higher than a pulse arriving
// right now would give, so a stopping shaft goes down to 0.
uint16_t calc_rpm(PcntMemData* cb) {
    uint32_t val = 0;
    uint16_t pulses;
    uint32_t edge;
    uint32_t now = micros();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pulses = cb->count;
        cb->count = 0;
        edge = cb->edge_time_us;
    }
    uint32_t us_per_rev = (60UL * 1000UL * 1000UL) / cb->pulses_rev;
    if (0 != pulses) {
        uint32_t t = edge - cb->last_time_us;
        if (0 != t) {
            val = (us_per_rev * pulses) / t;
        } else {
            val = MAX_RPM_PCNT;
        }
        cb->last_time_us = edge;
    } else {
        uint32_t since_edge = now - cb->last_time_us;
        val = cb->last_rpm;
        if (0 != since_edge) {
            val = MIN(val, us_per_rev / since_edge);
        }
    }
    if (val < 60) {
        val = 0;
    }
    if (val > MAX_RPM_PCNT) {
        val = MAX_RPM_PCNT;
    }
    cb->last_rpm = val;
    return (uint16_t)val;
}

bool Sensors::using_dedicated_output_rpm() {
    return output_rpm_ok;
}

static void configure_pcnt(uint16_t pulses_per_rpm, Pin pin, PcntMemData* mem, void (*isr)(void)) {
    mem->count = 0;
    mem->last_rpm = 0;
    mem->last_time_us = micros();
    mem->pulses_rev = pulses_per_rpm; // Rising edges only
    attachInterrupt(digitalPinToInterrupt(toRaw(pin)), isr, RISING);
    mem->init = true;
}

bool Sensors::init_sensors(void) {
    bool ret = true;
    configure_pcnt(N_SENSOR_PULSES_PER_REV, Pin::n2_pin, &mem_data_n2, isr_n2);
    configure_pcnt(N_SENSOR_PULSES_PER_REV, Pin::n3_pin, &mem_data_n3, isr_n3);

    // Enable output RPM reading if needed
    if (VEHICLE_CONFIG.io_0_usage == 1) {
        if (VEHICLE_CONFIG.input_sensor_pulses_per_rev == 0) {
            // Cannot init output sensor with 0 pulses/rev specified
            ret = false;
        } else {
            configure_pcnt(VEHICLE_CONFIG.input_sensor_pulses_per_rev, Pin::io_pin, &mem_data_out, isr_out);
            output_rpm_ok = true;
        }
    }
    return ret;
}

const uint8_t NUM_TEMP_POINTS = 27u;

// Same table as UN52 board_config.h
// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
const int16_t TFT_RESISTANCE_TAB[2][NUM_TEMP_POINTS] TCU_FLASH = { // In flash
    // Resistance (Ohm)
    { 500,  525,  577,  632,  691,  754, 820,  889,  962, 1000, 1039, 1118, 1202, 1288, 1379, 1472, 1569, 1670, 1774, 1882, 1937, 1993, 2107, 2225, 2346, 2471, 2535},
    // Temperature (x10)
    {-550, -500, -400, -300, -200, -100,   0,  100,  200,  250,  300,  400,  500,  600,  700,  800,  900, 1000, 1100, 1200, 1250, 1300, 1400, 1500, 1600, 1700, 1750}
};

// Same as interpolate_linear_array (UN52) on TFT_RESISTANCE_TAB, reading the table from flash
static float interpolate_tft_flash(int16_t v) {
    int16_t x[NUM_TEMP_POINTS];
    uint8_t i = 0;
    for (i = 0; i < NUM_TEMP_POINTS; i++) {
        x[i] = (int16_t)pgm_read_word(&TFT_RESISTANCE_TAB[0][i]);
    }
    uint16_t idx_min = 0;
    uint16_t idx_max = 0;
    search_value(v, x, NUM_TEMP_POINTS, &idx_min, &idx_max);
    int16_t v1 = (int16_t)pgm_read_word(&TFT_RESISTANCE_TAB[1][idx_min]);
    int16_t v2 = (int16_t)pgm_read_word(&TFT_RESISTANCE_TAB[1][idx_max]);
    return interpolate((float)v1, (float)v2, x[idx_min], x[idx_max], v);
}

// A shorted sensor reads below this (Lowest table resistance is 500 Ohm)
#define ATF_MIN_PLAUSIBLE_OHM 400

static uint32_t adc_to_mv(int raw) {
    return ((uint32_t)raw * ADC_VREF_MV) / 1023UL;
}

void Sensors::update(SensorDataRaw* dest) {
    dest->rpm_n2 = UINT16_MAX;
    dest->rpm_n3 = UINT16_MAX;
    dest->rpm_out = UINT16_MAX;
    dest->battery_mv = UINT16_MAX;
    dest->atf_temp_c = INT16_MAX;
    dest->parking_lock = UINT8_MAX;

    // RPM Sensors
    if (mem_data_n2.init) {
        dest->rpm_n2 = calc_rpm(&mem_data_n2);
    }
    if (mem_data_n3.init) {
        dest->rpm_n3 = calc_rpm(&mem_data_n3);
    }
    if (output_rpm_ok) {
        dest->rpm_out = calc_rpm(&mem_data_out);
    }

    // Battery voltage. Vin = Vout(R1+R2)/R2
    uint32_t v = adc_to_mv(readPinAnalog(Pin::vsense_pin));
    v = (v * (VBATT_R1_OHM + VBATT_R2_OHM)) / VBATT_R2_OHM;
    if (v < UINT16_MAX) {
        dest->battery_mv = v;
    }

    // TFT/PL lock (Same logic as UN52 sensors.cpp)
    int adc_res = readPinAnalog(Pin::atf_pin);
    if (adc_res > ATF_PLL_OPEN_ADC) {
        dest->parking_lock = 1;
        dest->atf_temp_c = INT16_MAX;
    } else {
        dest->parking_lock = 0;
        uint32_t adc_voltage = adc_to_mv(adc_res);
        // Pull-up to Vref: R_sensor = Vout * R_pullup / (Vref - Vout)
        int32_t resistance = (adc_voltage * ATF_PULLUP_OHM) / (ADC_VREF_MV - adc_voltage);
        if (resistance >= ATF_MIN_PLAUSIBLE_OHM) {
            resistance = MIN(resistance, (int32_t)INT16_MAX);
            float out_x10 = interpolate_tft_flash((int16_t)resistance);
            dest->atf_temp_c = (int16_t)(out_x10 / 10.0);
        }
        // else: sensor shorted to ground, leave ATF as not available
    }
}
