#include "can_egs52.h"
#include <Arduino.h>
#include <tcu_maths.h>
#include "board_config.h"

EgsBaseCan* egs_can_hal = nullptr;

static Egs52Can* isr_instance = nullptr;

static void can_int_isr() {
    if (nullptr != isr_instance) {
        isr_instance->on_interrupt();
    }
}

Egs52Can::Egs52Can(uint8_t cs_pin, uint8_t int_pin) : mcp(cs_pin, 8000000), int_pin(int_pin) {
}

bool Egs52Can::begin() {
    if (MCP2515::ERROR_OK != this->mcp.reset()) {
        return false;
    }
    if (MCP2515::ERROR_OK != this->mcp.setBitrate(CAN_500KBPS, CAN_CRYSTAL)) {
        return false;
    }
    // Only let the frames we use into the MCP2515, so the RX buffers don't fill with other traffic.
    // RXB0 (MASK0, RXF0-1) gets the torque frames, RXB1 (MASK1, RXF2-5) the rest
    bool ok = true;
    ok &= MCP2515::ERROR_OK == this->mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF0, false, MS_312_EGS52_CAN_ID);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF1, false, MS_212_EGS52_CAN_ID);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF2, false, MS_210_EGS52_CAN_ID);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF3, false, MS_308_EGS52_CAN_ID);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF4, false, MS_608_EGS52_CAN_ID);
    ok &= MCP2515::ERROR_OK == this->mcp.setFilter(MCP2515::RXF5, false, EWM_230_EGS52_CAN_ID);
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

void Egs52Can::on_interrupt() {
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

void Egs52Can::update() {
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

        if (this->ecu_ms.import_frames(tmp, id, now)) {
            switch (id) {
                case MS_210_EGS52_CAN_ID: this->seen_frames |= SEEN_MS_210; break;
                case MS_212_EGS52_CAN_ID: this->seen_frames |= SEEN_MS_212; break;
                case MS_308_EGS52_CAN_ID: this->seen_frames |= SEEN_MS_308; break;
                case MS_312_EGS52_CAN_ID: this->seen_frames |= SEEN_MS_312; break;
                case MS_608_EGS52_CAN_ID: this->seen_frames |= SEEN_MS_608; break;
                default: break;
            }
            this->last_rx_time = now;
        } else if (this->ewm_ecu.import_frames(tmp, id, now)) {
            this->seen_frames |= SEEN_EWM_230;
            this->last_rx_time = now;
        }
    }
}

bool Egs52Can::bus_ok(const uint32_t expire_time_ms) {
    return 0 != this->seen_frames && (GET_CLOCK_TIME() - this->last_rx_time) < expire_time_ms;
}

uint16_t Egs52Can::get_rx_overflow_count() {
    noInterrupts();
    uint16_t ret = this->rx_overflows;
    interrupts();
    return ret;
}

uint8_t Egs52Can::get_pedal_value(const uint32_t expire_time_ms) {
    MS_210_EGS52 ms210;
    if (this->seen(SEEN_MS_210) && this->ecu_ms.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        return ms210.PW;
    } else {
        return 0xFF;
    }
}

// The torque signals are 13 bit, this value (all bits set) is treated as 'not available'.
// (UN52 compares against UINT16_MAX, which a 13 bit field can never reach)
#define TRQ_13BIT_SNV 0x1FFF

CanTorqueData Egs52Can::get_torque_data(const uint32_t expire_time_ms) {
    MS_312_EGS52 ms312;
    MS_212_EGS52 ms212;
    MS_210_EGS52 ms210;
    CanTorqueData ret = TORQUE_NDEF;
    uint32_t now = GET_CLOCK_TIME();
    // Same as UN52 can_egs52.cpp. Maths is done in int16_t (raw/4 fits), the -500 offset can go negative
    if (this->seen(SEEN_MS_312) && this->ecu_ms.get_MS_312(now, expire_time_ms, &ms312)) {
        if (TRQ_13BIT_SNV != ms312.M_STA) {ret.m_converted_static = ((int16_t)ms312.M_STA / 4) - 500;}
        if (TRQ_13BIT_SNV != ms312.M_MIN) {ret.m_min = ((int16_t)ms312.M_MIN / 4) - 500;}
        if (TRQ_13BIT_SNV != ms312.M_MAX) {ret.m_max = ((int16_t)ms312.M_MAX / 4) - 500;}
    }
    if (this->seen(SEEN_MS_212) && this->ecu_ms.get_MS_212(now, expire_time_ms, &ms212)) {
        if (TRQ_13BIT_SNV != ms212.M_ESPV) {ret.m_converted_driver = ((int16_t)ms212.M_ESPV / 4) - 500;}
    }
    if (INT16_MAX != ret.m_max) {
        // Get factor to correct it by
        if (this->seen(SEEN_MS_210) && this->ecu_ms.get_MS_210(now, expire_time_ms, &ms210)) {
            if (UINT8_MAX != ms210.FMMOTMAX) {ret.m_max = (float)ret.m_max * ((float)ms210.FMMOTMAX * 0.0078);}
        }
    }
    if (INT16_MAX != ret.m_converted_static && INT16_MAX != ret.m_converted_driver) {
        int16_t static_converted = ret.m_converted_static;
        int16_t tmp = ret.m_converted_driver;
        // Calculate converted torque from ESP
        if (INT16_MAX != ret.m_max && INT16_MAX != ret.m_min) {
            tmp = MAX(MIN(ret.m_converted_driver, ret.m_max), ret.m_min);
        }
        if (tmp <= 0) {
            tmp = MIN(tmp, static_converted);
        }
        // UN52 torque freezing (for torque requests) and A/C compressor torque (KLA_410) are not ported.
        // We never request torque, and not subtracting the compressor torque only makes pressure a bit higher.
        ret.m_ind = MAX(0, tmp);
        ret.m_converted_driver = tmp;
        ret.m_converted_static = static_converted;
    }
    return ret;
}

int16_t Egs52Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    MS_608_EGS52 ms608;
    int16_t res = INT16_MAX;
    if (this->seen(SEEN_MS_608) && this->ecu_ms.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = (int16_t)ms608.T_MOT - 40;
        }
    }
    return res;
}

uint16_t Egs52Can::get_engine_rpm(const uint32_t expire_time_ms) {
    MS_308_EGS52 ms308;
    if (this->seen(SEEN_MS_308) && this->ecu_ms.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.NMOT;
    } else {
        return UINT16_MAX;
    }
}

ShifterPosition Egs52Can::get_shifter_position(const uint32_t expire_time_ms) {
    ShifterPosition ret = ShifterPosition::SignalNotAvailable;
    EWM_230_EGS52 dest;
    if (this->seen(SEEN_EWM_230) && this->ewm_ecu.get_EWM_230(GET_CLOCK_TIME(), expire_time_ms, &dest)) {
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
