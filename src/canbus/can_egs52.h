#ifndef EGS52_CAN_H
#define EGS52_CAN_H

#include "can_mcp.h"
#include "../../lib/egs52_ecus/src/MS.h"
#include "../../lib/egs52_ecus/src/EWM.h"

/**
 * EGS52 CAN layer (W210 with EDC15 and EWM shifter), ported from UN52 can_egs52.cpp.
 *
 * MinTCU only LISTENS on the CAN bus (MCP2515 listen-only mode), it never transmits.
 * Accepted frames: MS_210, MS_212, MS_308, MS_312, MS_608 (engine) and EWM_230 (shifter)
 */
class Egs52Can: public McpCan {
    public:
        explicit Egs52Can(uint8_t cs_pin, uint8_t int_pin);
        uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
        bool get_kickdown(const uint32_t expire_time_ms) override;
        bool get_profile_btn_press(const uint32_t expire_time_ms) override;
        ProfileSwitchPos get_profile_switch_pos(const uint32_t expire_time_ms) override;
    protected:
        uint8_t import_frame(uint64_t data, uint16_t id, uint32_t now) override;
    private:
        enum SeenBit : uint8_t {
            SEEN_MS_210 = 1 << 0,
            SEEN_MS_212 = 1 << 1,
            SEEN_MS_308 = 1 << 2,
            SEEN_MS_312 = 1 << 3,
            SEEN_MS_608 = 1 << 4,
            SEEN_EWM_230 = 1 << 5,
        };
        // Torque frames first (RXB0)
        static constexpr uint16_t FILTERS[MCP_NUM_FILTERS] = {
            MS_312_EGS52_CAN_ID, MS_212_EGS52_CAN_ID,
            MS_210_EGS52_CAN_ID, MS_308_EGS52_CAN_ID, MS_608_EGS52_CAN_ID, EWM_230_EGS52_CAN_ID
        };
        ECU_MS ecu_ms;
        ECU_EWM ewm_ecu;
};

#endif
