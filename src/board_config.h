#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

/*
    Mega2560 pinout. Check these against your board before flashing!

    MPC and SPC MUST stay on pins 11 and 12, they are driven by Timer1 (OC1A / OC1B)
    in solenoids.cpp at 1kHz with 16000 steps. analogWrite() is not used for them.
    Y3/Y4/Y5 must be on PWM pins that are not on Timer0 or Timer1 (analogWrite, ~490Hz).

    Every solenoid output needs a gate pull-down resistor (10k) on the MOSFET, so the
    solenoids are OFF while the Mega is in reset / bootloader. MPC and SPC OFF = 0mA = max pressure,
    shift solenoids OFF = gearbox stays in 2nd (D) or R. That is the 722.6 limp mode.
*/
enum class Pin : uint8_t {
    y3_pwm     = 6,  // Y3 (1-2/4-5) shift solenoid (Timer4 PWM, used for the hold PWM)
    y4_pwm     = 7,  // Y4 (3-4) shift solenoid (Timer4 PWM)
    y5_pwm     = 8,  // Y5 (2-3) shift solenoid (Timer4 PWM)
    mpc_pwm    = 11, // Modulating pressure solenoid (Timer1 OC1A)
    spc_pwm    = 12, // Shift pressure solenoid (Timer1 OC1B)
    tcc_pwm    = 5,  // Torque converter solenoid (Timer3 OC3A, see solenoids/inrush_solenoid.h)
    can_cs     = 53, // MCP2515 chip select
    can_int    = 2,  // MCP2515 INT pin (must be an interrupt pin: 2, 3, 18, 19, 20, 21)
    n2_pin     = 18, // N2 input speed sensor (INT3)
    n3_pin     = 19, // N3 input speed sensor (INT2)
    io_pin     = 3,  // Dedicated output speed sensor (INT5), used when VEHICLE_CONFIG.io_0_usage == 1
    vsense_pin = A1, // Battery voltage (solenoid supply) via divider
    atf_pin    = A0, // ATF temp sensor (TFT) and park/neutral lockout, with pull-up to 5V
};

// MCP2515 module crystal. Most cheap modules use 8MHz, some 16MHz. Check the can on the module!
#define CAN_CRYSTAL MCP_8MHZ

// ADC reference voltage (5V VCC on the Mega). Measure your 5V rail and put the real value here,
// battery voltage (and so solenoid current) is only as accurate as this
#define ADC_VREF_MV 5000

// Battery voltage divider: Vbatt -> R1 -> pin -> R2 -> GND (UN52 uses 100k/22k)
#define VBATT_R1_OHM 100000UL
#define VBATT_R2_OHM 22000UL

// ATF sensor: 5V -> pull-up resistor -> pin -> TFT sensor (KTY83, ~1k at 25C) -> GND
#define ATF_PULLUP_OHM 2000UL
// Above this ADC reading the TFT circuit is open, meaning the park/neutral switch is open (P or N)
#define ATF_PLL_OPEN_ADC 950

// N2, N3 and the output speed sensor must give a clean 0V / 5V square wave on their pins.
// The Mega pins are 5V max, so condition the sensor signal before it reaches them.

// Helper to safely convert Pin enum to raw uint8_t for native Arduino API calls
constexpr uint8_t toRaw(Pin pin) {
    return static_cast<uint8_t>(pin);
}

inline void setPinMode(Pin pin, uint8_t mode) {
    pinMode(toRaw(pin), mode);
}

inline void writePinDigital(Pin pin, uint8_t val) {
    digitalWrite(toRaw(pin), val);
}

inline int readPinDigital(Pin pin) {
    return digitalRead(toRaw(pin));
}

inline void writePinAnalog(Pin pin, int val) {
    analogWrite(toRaw(pin), val);
}

inline int readPinAnalog(Pin pin) {
    return analogRead(toRaw(pin));
}

// Function to handle hardware setup for all pins at startup
void initPins();

#endif
