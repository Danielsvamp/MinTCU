#ifndef EGS52_CAN_H
#define EGS52_CAN_H

#include "can_hal.h"
#include <mcp2515.h>
#include "../../lib/egs52_ecus/src/MS.h"
#include "../../lib/egs52_ecus/src/EWM.h"

// Frames are moved from the MCP2515 into this ring buffer by the INT pin interrupt,
// so that the 2 RX buffers on the MCP2515 do not overflow while the gearbox loop runs.
#define CAN_RX_RING_SIZE 16 // Must be a power of 2

/**
 * EGS52 CAN layer (W210 with EDC15 and EWM shifter), ported from UN52 can_egs52.cpp.
 *
 * MinTCU only LISTENS on the CAN bus (MCP2515 listen-only mode), it never transmits.
 * Accepted frames: MS_210, MS_212, MS_308, MS_312, MS_608 (engine) and EWM_230 (shifter)
 */
class Egs52Can: public EgsBaseCan {
    public:
        explicit Egs52Can(uint8_t cs_pin, uint8_t int_pin);
        bool begin() override;
        void update() override;
        bool bus_ok(const uint32_t expire_time_ms) override;

        uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
        uint16_t get_rx_overflow_count() override;

        // Only called from the INT pin interrupt
        void on_interrupt();
    private:
        // UN52 ECU objects only say 'valid' based on frame age. On the Mega millis() starts at 0,
        // so a frame that has never been seen looks valid (with 0 data) for the first second.
        // These bits stop that.
        enum SeenBit : uint8_t {
            SEEN_MS_210 = 1 << 0,
            SEEN_MS_212 = 1 << 1,
            SEEN_MS_308 = 1 << 2,
            SEEN_MS_312 = 1 << 3,
            SEEN_MS_608 = 1 << 4,
            SEEN_EWM_230 = 1 << 5,
        };
        bool seen(uint8_t bit) const { return (this->seen_frames & bit) != 0; }

        MCP2515 mcp;
        uint8_t int_pin;
        ECU_MS ecu_ms;
        ECU_EWM ewm_ecu;
        uint8_t seen_frames = 0;
        uint32_t last_rx_time = 0;

        struct RxFrame {
            uint16_t id;
            uint8_t dlc;
            uint8_t data[8];
        };
        volatile RxFrame rx_ring[CAN_RX_RING_SIZE];
        volatile uint8_t rx_head = 0; // Written by ISR only
        volatile uint8_t rx_tail = 0; // Written by update() only
        volatile uint16_t rx_overflows = 0;
};

#endif
