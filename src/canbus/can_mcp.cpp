#include "can_mcp.h"
#include <Arduino.h>
#include <tcu_maths.h>
#include "board_config.h"

EgsBaseCan* egs_can_hal = nullptr;

static McpCan* isr_instance = nullptr;

static void can_int_isr() {
    if (nullptr != isr_instance) {
        isr_instance->on_interrupt();
    }
}

McpCan::McpCan(uint8_t cs_pin, uint8_t int_pin, const uint16_t filters[MCP_NUM_FILTERS]) : mcp(cs_pin, 8000000), int_pin(int_pin) {
    for (uint8_t i = 0; i < MCP_NUM_FILTERS; i++) {
        this->filters[i] = filters[i];
    }
}

bool McpCan::begin() {
    if (MCP2515::ERROR_OK != this->mcp.reset()) {
        return false;
    }
    if (MCP2515::ERROR_OK != this->mcp.setBitrate(CAN_500KBPS, CAN_CRYSTAL)) {
        return false;
    }
    // Only let the frames we use into the MCP2515, so the RX buffers don't fill with other traffic.
    // RXB0 (MASK0, RXF0-1) gets the torque frames, RXB1 (MASK1, RXF2-5) the rest
    const MCP2515::RXF rxf[MCP_NUM_FILTERS] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2, MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
    bool ok = true;
    ok &= MCP2515::ERROR_OK == this->mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    for (uint8_t i = 0; i < MCP_NUM_FILTERS; i++) {
        ok &= MCP2515::ERROR_OK == this->mcp.setFilter(rxf[i], false, this->filters[i]);
    }
    if (!ok) {
        return false;
    }
    // Listen only: we never ACK or transmit, so we cannot upset the car's CAN bus
    if (MCP2515::ERROR_OK != this->mcp.setListenOnlyMode()) {
        return false;
    }
    isr_instance = this;
    attachInterrupt(digitalPinToInterrupt(this->int_pin), can_int_isr, FALLING);
    return true;
}

void McpCan::on_interrupt() {
    struct can_frame frame;
    // Empty both RX buffers, INT goes high again once they are empty
    while (MCP2515::ERROR_OK == this->mcp.readMessage(&frame)) {
        if (0 != (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) || 0 == frame.can_dlc) {
            continue;
        }
        uint8_t next = (this->rx_head + 1) & (CAN_RX_RING_SIZE - 1);
        if (next == this->rx_tail) {
            this->rx_overflows++; // Ring full, drop the frame
            continue;
        }
        volatile RxFrame* dest = &this->rx_ring[this->rx_head];
        dest->id = frame.can_id & CAN_SFF_MASK;
        dest->dlc = MIN(frame.can_dlc, 8);
        for (uint8_t i = 0; i < 8; i++) {
            dest->data[i] = (i < frame.can_dlc) ? frame.data[i] : 0;
        }
        this->rx_head = next;
    }
    // Clear error interrupts, otherwise INT stays low forever
    uint8_t irq = this->mcp.getInterrupts();
    if (0 != (irq & (MCP2515::CANINTF_ERRIF | MCP2515::CANINTF_MERRF))) {
        if (0 != (this->mcp.getErrorFlags() & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))) {
            this->rx_overflows++;
            this->mcp.clearRXnOVRFlags();
        }
        this->mcp.clearERRIF();
        this->mcp.clearMERR();
    }
}

void McpCan::update() {
    // If INT is still low here, an edge was missed (Eg. a frame came in before attachInterrupt).
    // Service it with interrupts off so the ISR cannot use SPI at the same time
    if (LOW == digitalRead(this->int_pin)) {
        noInterrupts();
        this->on_interrupt();
        interrupts();
    }
    uint32_t now = GET_CLOCK_TIME();
    while (this->rx_tail != this->rx_head) {
        volatile RxFrame* src = &this->rx_ring[this->rx_tail];
        // Same byte order as UN52 can_hal.cpp: data[0] is the MSB
        uint64_t tmp = 0;
        for (uint8_t i = 0; i < src->dlc; i++) {
            tmp |= (uint64_t)src->data[i] << (8*(7-i));
        }
        uint16_t id = src->id;
        this->rx_tail = (this->rx_tail + 1) & (CAN_RX_RING_SIZE - 1);

        uint8_t bit = this->import_frame(tmp, id, now);
        if (0 != bit) {
            this->seen_frames |= bit;
            this->last_rx_time = now;
        }
    }
}

bool McpCan::bus_ok(const uint32_t expire_time_ms) {
    return 0 != this->seen_frames && (GET_CLOCK_TIME() - this->last_rx_time) < expire_time_ms;
}

uint16_t McpCan::get_rx_overflow_count() {
    noInterrupts();
    uint16_t ret = this->rx_overflows;
    interrupts();
    return ret;
}

ShifterPosition ewm_shifter_position(const ECU_EWM* ewm, bool seen, const uint32_t expire_time_ms) {
    ShifterPosition ret = ShifterPosition::SignalNotAvailable;
    EWM_230_EGS52 dest;
    if (seen && ewm->get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest)) {
        switch (dest.WHC) {
            case EWM_230h_WHC_EGS52::D:
                ret = ShifterPosition::D;
                break;
            case EWM_230h_WHC_EGS52::N:
                ret = ShifterPosition::N;
                break;
            case EWM_230h_WHC_EGS52::R:
                ret = ShifterPosition::R;
                break;
            case EWM_230h_WHC_EGS52::P:
                ret = ShifterPosition::P;
                break;
            case EWM_230h_WHC_EGS52::PLUS:
                ret = ShifterPosition::PLUS;
                break;
            case EWM_230h_WHC_EGS52::MINUS:
                ret = ShifterPosition::MINUS;
                break;
            case EWM_230h_WHC_EGS52::N_ZW_D:
                ret = ShifterPosition::N_D;
                break;
            case EWM_230h_WHC_EGS52::R_ZW_N:
                ret = ShifterPosition::R_N;
                break;
            case EWM_230h_WHC_EGS52::P_ZW_R:
                ret = ShifterPosition::P_R;
                break;
            case EWM_230h_WHC_EGS52::SNV:
            default:
                break;
        }
    }
    return ret;
}
