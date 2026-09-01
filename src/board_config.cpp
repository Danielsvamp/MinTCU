#include "board_config.h"

void initPins() {
    setPinMode(Pin::y3_pwm, OUTPUT);

    
    writePinDigital(Pin::y3_pwm, LOW);
}