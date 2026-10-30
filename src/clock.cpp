#include "clock.hpp"
#include <Arduino.h>

//millis should work here I think?

uint32_t GET_CLOCK_TIME() {
    return millis();
}
