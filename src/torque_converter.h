#ifndef TORQUE_CONVERTER_H
#define TORQUE_CONVERTER_H

#include <stdint.h>

/*
    MinTCU does not control the torque converter clutch yet, the TCC solenoid is never switched on,
    so the converter is always open (UN52 InternalTccState::Open).

    This is the part of UN52's TorqueConverter that the shift algorithms use (shift_start, shift_end
    and get_current_state), with the converter always open.
*/

enum class InternalTccState {
    Open = 0,
    Slipping = 1,
};

class TorqueConverter {
    public:
        void shift_start(bool upshift, bool release_shifting) {}
        void shift_end() {}
        uint8_t get_current_state() { return (uint8_t)InternalTccState::Open; }
};

#endif
