#include "clock.hpp"
#include <arduino.h>

//millis should work here I think?

uint32_t GET_CLOCK_TIME() {
    return millis();
}
