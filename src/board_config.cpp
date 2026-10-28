#include "board_config.h"

void initPins() {
    // All solenoids OFF first thing (limp mode: max pressure, 2nd gear / R)
    const Pin outputs[] = { Pin::y3_pwm, Pin::y4_pwm, Pin::y5_pwm, Pin::mpc_pwm, Pin::spc_pwm, Pin::tcc_pwm };
    for (Pin p : outputs) {
        writePinDigital(p, LOW);
        setPinMode(p, OUTPUT);
    }
    setPinMode(Pin::can_int, INPUT_PULLUP);
    setPinMode(Pin::vsense_pin, INPUT);
    setPinMode(Pin::atf_pin, INPUT);
}
