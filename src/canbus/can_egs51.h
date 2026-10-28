#ifndef EGS51_CAN_H
#define EGS51_CAN_H

#include "can_mcp.h"
#include "../../lib/egs51_ecus/src/MS51.h"
#include "../../lib/egs52_ecus/src/EWM.h"

/**
 * EGS51 CAN layer (Engine ECU from an EGS51 car, Eg. W210 EDC15C6 + EWM shifter), ported from UN52 can_egs51.cpp.
 *
 * MinTCU only LISTENS on the CAN bus (MCP2515 listen-only mode), it never transmits.
 * Accepted frames: MS_210, MS_308, MS_310, MS_608 (engine) and EWM_230 (shifter, same frame as EGS52)
 */
class Egs51Can: public McpCan {
    public:
        explicit Egs51Can(uint8_t cs_pin, uint8_t int_pin);
        uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
    protected:
        uint8_t import_frame(uint64_t data, uint16_t id, uint32_t now) override;
    private:
        enum SeenBit : uint8_t {
            SEEN_MS_210 = 1 << 0,
            SEEN_MS_308 = 1 << 1,
            SEEN_MS_310 = 1 << 2,
            SEEN_MS_608 = 1 << 3,
            SEEN_EWM_230 = 1 << 4,
        };
        // Torque frames first (RXB0). The last filter is not needed, it repeats EWM_230
        static constexpr uint16_t FILTERS[MCP_NUM_FILTERS] = {
            MS_310_EGS51_CAN_ID, MS_210_EGS51_CAN_ID,
            MS_308_EGS51_CAN_ID, MS_608_EGS51_CAN_ID, EWM_230_EGS52_CAN_ID, EWM_230_EGS52_CAN_ID
        };
        ECU_MS51 ms51;
        ECU_EWM ewm;
        // Preserve last value, otherwise will be 0 (Same as UN52)
        int16_t drag_trq = 0;
};

#endif
