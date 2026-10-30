/**
 * CANBUS abstraction layer, slimmed down from UN52 for the Mega2560 + MCP2515.
 *
 * Only the getters that the pressure path needs, and the setters for the EGS51 GS_218 frame, are here. Every getter takes an
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
         * @brief MANDITORY DATA (If no dedicated output shaft sensor) - Returns the rear right wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return rear right wheel data
         */
        virtual uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
        }

        /**
         * @brief MANDITORY DATA (If no dedicated output shaft sensor) - Returns the left right wheel speed (In half RPM increments)
         * @param expire_time_ms data expiration period
         * @return rear right left data
         */
        virtual uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) {
            return UINT16_MAX;
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
         * @brief OPTIONAL DATA (For kickdown only) - Returns if the kickdown switch is actuated
         * @param expire_time_ms data expiration period
         * @return Returns true if the kickdown switch is being pressed, false if released
         */
        virtual bool get_kickdown(const uint32_t expire_time_ms) {
            return false;
        }

        /**
         * This is intended for only the handling of the EWM CAN based shifters.
         * 
         * @param expire_time_ms data expiration period
         * @return True if the profile button is in the depressed position, or False if it is released
         */
        virtual bool get_profile_btn_press(const uint32_t expire_time_ms) {
            return false;
        }

        /**
         * @brief OPTIONAL DATA - Returns true if the shifter profile switch is in the top most position
         * This is intended for only the handling of the EWM CAN based shifters.
         * 
         * @param expire_time_ms data expiration period
         * @return True if the profile switch is in the top position
         */
        virtual ProfileSwitchPos get_profile_switch_pos(const uint32_t expire_time_ms) {
            return ProfileSwitchPos::SNV;
        }

        /**
         * @brief Engine request to open the TCC or let it slip (Same as UN52, not on EGS51.
         * Not ported for EGS52 yet)
         */
        virtual TccReqState get_engine_tcc_override_request(const uint32_t expire_time_ms) {
            return TccReqState::None;
        }

        /**
         * @brief Number of times the CAN controller ran out of RX buffers (Frames lost)
         */
        virtual uint16_t get_rx_overflow_count() { return 0; }

        /**
         * Setters (Same as UN52 can_hal.h, only the ones that go in the EGS51 GS_218 frame).
         * They only change the frame, update() sends it.
         */
        // Set the gearbox clutch position on CAN
        virtual void set_clutch_status(TccClutchStatus status){};
        // Set the actual gear of the gearbox
        virtual void set_actual_gear(GearboxGear actual){};
        // Set the target gear of the gearbox
        virtual void set_target_gear(GearboxGear target){};
        // Sets shifter position message
        virtual void set_shifter_position(ShifterPosition pos){};
        // Sets gearbox is OK
        virtual void set_gearbox_ok(bool is_ok){};
        // Sets torque request toggle
        virtual void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm){};
        // Tells the engine if we are shifting from P->R or N->D.
        // This is needed so the engine limits itself to 1K RPM, in order
        // to prevent any damage to the box!
        virtual void set_garage_shift_state(bool enable, bool to_d){};
        virtual void set_tcc_trq_multiplier(float multi){};

        /**
         * @brief MinTCU (not in UN52): true once torque requests really go out on the bus (The CAN
         * controller is out of listen-only mode). While false, the shift algorithms are told torque
         * requests are off, so they make up for the torque that is not taken away.
         */
        virtual bool can_request_torque() { return false; }
};

extern EgsBaseCan* egs_can_hal;

#endif
