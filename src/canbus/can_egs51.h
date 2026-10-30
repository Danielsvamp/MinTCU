#ifndef EGS51_CAN_H
#define EGS51_CAN_H

#include "can_mcp.h"
#include "../../lib/egs51_ecus/src/MS51.h"
// MinTCU: avr/io.h defines SE (The sleep enable bit number), which breaks the GS_218 SE field.
// Nothing in MinTCU uses the sleep modes
#ifdef SE
#undef SE
#endif
#include "../../lib/egs51_ecus/src/GS51.h"
#include "../../lib/egs52_ecus/src/EWM.h"
#include "../../lib/egs51_ecus/src/ESP51.h"

/**
 * EGS51 CAN layer (Engine ECU from an EGS51 car, Eg. W210 EDC15C6 + EWM shifter), ported from UN52 can_egs51.cpp.
 *
 * Accepted frames: MS_210, MS_308, MS_310, MS_608 (engine), EWM_230 (shifter, same frame as EGS52) and
 * BS_208 (ABS/ESP, rear wheel speeds for the output speed when there is no output speed sensor)
 * Sent frames: GS_218 (to the engine: gears, TCC state and the torque request), every 20ms, once the
 * car's frames have been received (See McpCan). UN52 also sends GS_418 (Display, cluster), MinTCU does not.
 */
class Egs51Can: public McpCan {
    public:
        explicit Egs51Can(uint8_t cs_pin, uint8_t int_pin);
        uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) override;
        uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) override;
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
        bool get_kickdown(const uint32_t expire_time_ms) override;
        bool get_profile_btn_press(const uint32_t expire_time_ms) override;
        ProfileSwitchPos get_profile_switch_pos(const uint32_t expire_time_ms) override;

        void set_clutch_status(TccClutchStatus status) override;
        void set_actual_gear(GearboxGear actual) override;
        void set_target_gear(GearboxGear target) override;
        void set_shifter_position(ShifterPosition pos) override;
        void set_gearbox_ok(bool is_ok) override;
        void set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) override;
        void set_garage_shift_state(bool enable, bool to_d) override;
        void set_tcc_trq_multiplier(float multi) override;

        /**
         * @brief The GS_218 frame as it goes on the bus (FEHLER counter not increased). MinTCU, for the log and tests
         */
        uint64_t get_gs218_raw() const { return this->gs218.raw; }
    protected:
        uint8_t import_frame(uint64_t data, uint16_t id, uint32_t now) override;
        void tx_frames() override;
    private:
        enum SeenBit : uint8_t {
            SEEN_MS_210 = 1 << 0,
            SEEN_MS_308 = 1 << 1,
            SEEN_MS_310 = 1 << 2,
            SEEN_MS_608 = 1 << 3,
            SEEN_EWM_230 = 1 << 4,
            SEEN_BS_208 = 1 << 5,
        };
        // Torque frames first (RXB0)
        static constexpr uint16_t FILTERS[MCP_NUM_FILTERS] = {
            MS_310_EGS51_CAN_ID, MS_210_EGS51_CAN_ID,
            MS_308_EGS51_CAN_ID, MS_608_EGS51_CAN_ID, EWM_230_EGS52_CAN_ID, BS_208_EGS51_CAN_ID
        };
        ECU_MS51 ms51;
        ECU_EWM ewm;
        ECU_ESP51 esp51;
        // Preserve last value, otherwise will be 0 (Same as UN52)
        int16_t drag_trq = 0;
        // CAN Frames to Tx
        GS_218_EGS51 gs218 = {0};
        uint8_t cvn_counter = 0;
        int16_t req_static_torque_delta = 0;
};

#endif
