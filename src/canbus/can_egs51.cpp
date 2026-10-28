#include "can_egs51.h"
#include <tcu_maths.h>

constexpr uint16_t Egs51Can::FILTERS[];

Egs51Can::Egs51Can(uint8_t cs_pin, uint8_t int_pin) : McpCan(cs_pin, int_pin, FILTERS) {
}

uint8_t Egs51Can::import_frame(uint64_t data, uint16_t id, uint32_t now) {
    uint8_t ret = 0;
    if (this->ms51.import_frames(data, id, now)) {
        switch (id) {
            case MS_210_EGS51_CAN_ID: ret = SEEN_MS_210; break;
            case MS_308_EGS51_CAN_ID: ret = SEEN_MS_308; break;
            case MS_310_EGS51_CAN_ID: ret = SEEN_MS_310; break;
            case MS_608_EGS51_CAN_ID: ret = SEEN_MS_608; break;
            default: break;
        }
    } else if (this->ewm.import_frames(data, id, now)) {
        ret = SEEN_EWM_230;
    }
    return ret;
}

uint8_t Egs51Can::get_pedal_value(const uint32_t expire_time_ms) {
    MS_210_EGS51 ms210;
    if (this->seen(SEEN_MS_210) && this->ms51.get_MS_210(GET_CLOCK_TIME(), expire_time_ms, &ms210)) {
        return ms210.PW;
    } else {
        return 0xFF;
    }
}

// Same as UN52 can_egs51.cpp. The torque freeze is not ported (MinTCU never requests torque, so
// UN52's gs218.TORQUE_REQ_EN would always be false here)
CanTorqueData Egs51Can::get_torque_data(const uint32_t expire_time_ms) {
    CanTorqueData ret = TORQUE_NDEF;
    MS_310_EGS51 ms310;
    MS_210_EGS51 ms210;
    int16_t m_esp = INT16_MAX;
    uint32_t now = GET_CLOCK_TIME();

    if (this->seen(SEEN_MS_310) && this->seen(SEEN_MS_210) &&
        this->ms51.get_MS_310(now, expire_time_ms, &ms310) &&
        this->ms51.get_MS_210(now, expire_time_ms, &ms210)) {
        if (UINT8_MAX != ms310.IND_TORQUE) {
            ret.m_ind = ((int16_t)ms310.IND_TORQUE)*3;
        }
        if (UINT8_MAX != ms310.MIN_TORQUE) {
            ret.m_min = ((int16_t)ms310.MIN_TORQUE)*3;
        }
        if (UINT8_MAX != ms310.MAX_TORQUE) {
            ret.m_max = ((int16_t)ms310.MAX_TORQUE)*3;
            ret.m_max = (float)ret.m_max * (float)(ms310.MAX_TRQ_FACTOR*0.0078);
        }
        if (UINT8_MAX != ms310.DRG_TORQUE) {
            // Preserve last value, otherwise will be 0
            this->drag_trq = ((int16_t)ms310.DRG_TORQUE)*3;
        }
        if (UINT8_MAX != ms210.M_ESP) {
            m_esp = ((int16_t)ms210.M_ESP)*3;
        }
    }
    if (
        INT16_MAX != ret.m_min &&
        INT16_MAX != ret.m_max &&
        INT16_MAX != ret.m_ind &&
        INT16_MAX != m_esp
    ) {
        ret.m_min -= this->drag_trq;
        ret.m_max -= this->drag_trq;
        ret.m_ind -= this->drag_trq;

        m_esp = MAX(0, MIN(m_esp - (int16_t)this->drag_trq, ret.m_max));
        ret.m_ind = MAX(ret.m_min, MIN(ret.m_ind, ret.m_max));

        int16_t driver_converted = m_esp;
        int16_t static_converted = ret.m_ind;

        ret.m_converted_driver = driver_converted;
        ret.m_converted_static = static_converted;
    }
    return ret;
}

int16_t Egs51Can::get_engine_coolant_temp(const uint32_t expire_time_ms) {
    MS_608_EGS51 ms608;
    int16_t res = INT16_MAX;
    if (this->seen(SEEN_MS_608) && this->ms51.get_MS_608(GET_CLOCK_TIME(), expire_time_ms, &ms608)) {
        if (ms608.T_MOT != UINT8_MAX) {
            res = (int16_t)ms608.T_MOT - 40;
        }
    }
    return res;
}

uint16_t Egs51Can::get_engine_rpm(const uint32_t expire_time_ms) {
    MS_308_EGS51 ms308;
    if (this->seen(SEEN_MS_308) && this->ms51.get_MS_308(GET_CLOCK_TIME(), expire_time_ms, &ms308)) {
        return ms308.NMOT;
    } else {
        return UINT16_MAX;
    }
}

ShifterPosition Egs51Can::get_shifter_position(const uint32_t expire_time_ms) {
    return ewm_shifter_position(&this->ewm, this->seen(SEEN_EWM_230), expire_time_ms);
}
