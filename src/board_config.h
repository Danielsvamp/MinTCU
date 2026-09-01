#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

    // Pin numbers are placeholders
enum class Pin : uint8_t {
    vsense_pin = 25, // Battery voltage feedback
    atf_pin    = 27, // ATF temp sensor and lockout
    n3_pin     = 14, // N3 speed sensor
    n2_pin     = 26, // N2 speed sensor
    y3_pwm     = 23, // Y3 (1-2/4-5) shift solenoid (PWM output)
    y4_pwm     = 22, // Y4 (3-4) shift solenoid (PWM output)
    y5_pwm     = 19, // Y5 (2-3) shift solenoid (PWM output)
    mpc_pwm    = 21, // Modulating pressure solenoid (PWM output)
    spc_pwm    = 12, // Shift pressure solenoid (PWM output)
    tcc_pwm    = 13, // Torque converter solenoid (PWM output)
};

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