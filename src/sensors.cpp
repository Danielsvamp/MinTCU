#include "sensors.h"
#include "board_config.h"
#include <tcu_maths.h>
#include <tcu_maths_impl.h>

const uint8_t NUM_TEMP_POINTS = 27u;

// Same table as UN52 board_config.h
// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
const int16_t TFT_RESISTANCE_TAB[2][NUM_TEMP_POINTS] = {
    // Resistance (Ohm)
    { 500,  525,  577,  632,  691,  754, 820,  889,  962, 1000, 1039, 1118, 1202, 1288, 1379, 1472, 1569, 1670, 1774, 1882, 1937, 1993, 2107, 2225, 2346, 2471, 2535},
    // Temperature (x10)
    {-550, -500, -400, -300, -200, -100,   0,  100,  200,  250,  300,  400,  500,  600,  700,  800,  900, 1000, 1100, 1200, 1250, 1300, 1400, 1500, 1600, 1700, 1750}
};

// A shorted sensor reads below this (Lowest table resistance is 500 Ohm)
#define ATF_MIN_PLAUSIBLE_OHM 400

static uint32_t adc_to_mv(int raw) {
    return ((uint32_t)raw * ADC_VREF_MV) / 1023UL;
}

void Sensors::update(SensorDataRaw* dest) {
    dest->battery_mv = UINT16_MAX;
    dest->atf_temp_c = INT16_MAX;
    dest->parking_lock = UINT8_MAX;

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
            float out_x10 = interpolate_linear_array((int16_t)resistance, NUM_TEMP_POINTS, TFT_RESISTANCE_TAB[0], TFT_RESISTANCE_TAB[1]);
            dest->atf_temp_c = (int16_t)(out_x10 / 10.0);
        }
        // else: sensor shorted to ground, leave ATF as not available
    }
}
