#include "can_egs52.h"
#include <tcu_maths.h>

constexpr uint16_t Egs52Can::FILTERS[];

Egs52Can::Egs52Can(uint8_t cs_pin, uint8_t int_pin) : McpCan(cs_pin, int_pin, FILTERS, false) {
}

uint8_t Egs52Can::import_frame(uint64_t data, uint16_t id, uint32_t now) {
    uint8_t ret = 0;
    if (this->ecu_ms.import_frames(data, id, now)) {
        switch (id) {
            case MS_210_EGS52_CAN_ID: ret = SEEN_MS_210; break;
            case MS_212_EGS52_CAN_ID: ret = SEEN_MS_212; break;
            case MS_308_EGS52_CAN_ID: ret = SEEN_MS_308; break;
            case MS_312_EGS52_CAN_ID: ret = SEEN_MS_312; break;
            case MS_608_EGS52_CAN_ID: ret = SEEN_MS_608; break;
            default: break;
        }
    } else if (this->ewm_ecu.import_frames(data, id, now)) {
        ret = SEEN_EWM_230;
    }
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
    return ewm_shifter_position(&this->ewm_ecu, this->seen(SEEN_EWM_230), expire_time_ms);
}

bool Egs52Can::get_kickdown(const uint32_t expire_time_ms) {
    return ewm_kickdown(&this->ewm_ecu, this->seen(SEEN_EWM_230), expire_time_ms);
}

bool Egs52Can::get_profile_btn_press(const uint32_t expire_time_ms) {
    return ewm_profile_btn_press(&this->ewm_ecu, this->seen(SEEN_EWM_230), expire_time_ms);
}

ProfileSwitchPos Egs52Can::get_profile_switch_pos(const uint32_t expire_time_ms) {
    return ewm_profile_switch_pos(&this->ewm_ecu, this->seen(SEEN_EWM_230), expire_time_ms);
}
