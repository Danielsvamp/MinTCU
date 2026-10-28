#ifndef MCP_CAN_H
#define MCP_CAN_H

#include "can_hal.h"
#include <mcp2515.h>

// Frames are moved from the MCP2515 into this ring buffer by the INT pin interrupt,
// so that the 2 RX buffers on the MCP2515 do not overflow while the gearbox loop runs.
#define CAN_RX_RING_SIZE 16 // Must be a power of 2

// Number of hardware filters on the MCP2515 (RXF0-1 go to RXB0, RXF2-5 to RXB1)
#define MCP_NUM_FILTERS 6

/**
 * MCP2515 part of the CAN layer (Not in UN52, UN52 uses the ESP32 TWAI controller).
 * Shared by the EGS51 and EGS52 CAN layers, they only decode the frames.
 *
 * MinTCU only LISTENS on the CAN bus (MCP2515 listen-only mode), it never transmits.
 */
class McpCan: public EgsBaseCan {
    public:
        /**
         * @param filters The CAN IDs to receive. Put the torque frames first (RXB0)
         */
        McpCan(uint8_t cs_pin, uint8_t int_pin, const uint16_t filters[MCP_NUM_FILTERS]);
        bool begin() override;
        void update() override;
        bool bus_ok(const uint32_t expire_time_ms) override;
        uint16_t get_rx_overflow_count() override;

        // Only called from the INT pin interrupt
        void on_interrupt();
    protected:
        /**
         * @brief Gives a received frame to the ECU objects. Same byte order as UN52 (data[0] is the MSB)
         * @return the bit to set in seen_frames, 0 if the frame was not used
         */
        virtual uint8_t import_frame(uint64_t data, uint16_t id, uint32_t now) = 0;

        // UN52 ECU objects only say 'valid' based on frame age. On the Mega millis() starts at 0,
        // so a frame that has never been seen looks valid (with 0 data) for the first second.
        // These bits stop that.
        bool seen(uint8_t bit) const { return (this->seen_frames & bit) != 0; }
        uint8_t seen_frames = 0;
    private:
        MCP2515 mcp;
        uint8_t int_pin;
        uint16_t filters[MCP_NUM_FILTERS];
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

// Shifter position from the EWM (Same frame and decoding for EGS51 and EGS52 in UN52)
#include "../../lib/egs52_ecus/src/EWM.h"
ShifterPosition ewm_shifter_position(const ECU_EWM* ewm, bool seen, const uint32_t expire_time_ms);

#endif
