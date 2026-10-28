/**
 * CANBUS abstraction layer, slimmed down from UN52 for the Mega2560 + MCP2515.
 *
 * Only the getters that the pressure path needs are here. Every getter takes an
 * expire time, and returns the 'not available' value (UINT16_MAX, INT16_MAX, 0xFF, TORQUE_NDEF)
 * once the frame is older than that. The gearbox relies on this for its failsafes.
 */

#ifndef ABSTRACT_CAN_H
#define ABSTRACT_CAN_H

#include <stdint.h>
#include <limits.h>
#include "clock.hpp"
#include "can_defines.h"
#include "../shifter/shifter.h"

const CanTorqueData TORQUE_NDEF = {
    INT16_MAX, // m_min
    INT16_MAX, // m_max
    INT16_MAX, // m_ind
    INT16_MAX, // m_converted_static
    INT16_MAX, // m_converted_driver
};

class EgsBaseCan {
    public:
        /**
         * @brief Starts the CAN controller
         * @return true if the CAN controller was found and configured
         */
        virtual bool begin() { return false; }

        /**
         * @brief Moves received frames into the ECU objects. Call this as often as possible (Every loop())
         */
        virtual void update() {}

        /**
         * @brief true if any frame has been received within expire_time_ms
         */
        virtual bool bus_ok(const uint32_t expire_time_ms) { return false; }

        /**
         * @brief MANDITORY DATA - Returns the pedal position, on a scale of 0-250
         * @return Pedal position (0 = 0%, 250 = 100%), 0xFF if not available
         */
        virtual uint8_t get_pedal_value(const uint32_t expire_time_ms) {
            return 0xFF;
        }

        /**
         * @brief MANDITORY DATA - Returns a structure containing a bunch of important engine torque information
         */
        virtual CanTorqueData get_torque_data(const uint32_t expire_time_ms) {
            return TORQUE_NDEF;
        }

        /**
         * @brief OPTIONAL DATA - Returns the engines coolant temperature in Celcius
         */
        virtual int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) {
            return INT16_MAX;
        }

        /**
         * @brief MANDITORY DATA - Returns the engines speed in RPM
         */
        virtual uint16_t get_engine_rpm(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }

        /**
         * @brief MANDITORY DATA - Returns the position of the gear selector (EWM)
         */
        virtual ShifterPosition get_shifter_position(const uint32_t expire_time_ms) {
            return ShifterPosition::SignalNotAvailable;
        }

        /**
         * @brief Number of times the CAN controller ran out of RX buffers (Frames lost)
         */
        virtual uint16_t get_rx_overflow_count() { return 0; }
};

extern EgsBaseCan* egs_can_hal;

#endif
