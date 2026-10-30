#include "can_egs51.h"
#include <tcu_maths.h>

constexpr uint16_t Egs51Can::FILTERS[];

Egs51Can::Egs51Can(uint8_t cs_pin, uint8_t int_pin) : McpCan(cs_pin, int_pin, FILTERS, true) {
    // Same start values as UN52
    this->gs218.TORQUE_REQ = 0xFE;
    this->gs218.bytes[7] = 0xFE;
    this->gs218.bytes[4] = 0x48;
    this->gs218.bytes[3] = 0x64;
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
    } else if (BS_208_EGS51_CAN_ID == id && this->esp51.import_frames(data, id, now)) {
        ret = SEEN_BS_208;
    }
    return ret;
}

// Same as UN52 can_egs51.cpp
uint16_t Egs51Can::get_rear_right_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->seen(SEEN_BS_208) && this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (0x3FFF != bs208.DHR) {
            ret = bs208.DHR;
        }
    }
    return ret;
}

uint16_t Egs51Can::get_rear_left_wheel(const uint32_t expire_time_ms) {
    BS_208_EGS51 bs208;
    uint16_t ret = UINT16_MAX;
    if (this->seen(SEEN_BS_208) && this->esp51.get_BS_208(GET_CLOCK_TIME(), expire_time_ms, &bs208)) {
        if (0x3FFF != bs208.DHL) {
            ret = bs208.DHL;
        }
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

// Same as UN52 can_egs51.cpp
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

        bool freeze = this->gs218.TORQUE_REQ_EN;
        // Change torque values based on freezing or not
        if (freeze) {
            ret.m_converted_driver = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        } else {
            this->req_static_torque_delta = driver_converted - static_converted;
            ret.m_converted_driver = driver_converted;
        }
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

// UN52's EGS51 layer reads kickdown from the EWM, but not the program button or switch (UN52
// Egs51Can::get_profile_btn_press returns false). The EWM sends the same EWM_230 frame in an
// EGS51 car, so these use UN52's EGS52 code.
bool Egs51Can::get_kickdown(const uint32_t expire_time_ms) {
    return ewm_kickdown(&this->ewm, this->seen(SEEN_EWM_230), expire_time_ms);
}

bool Egs51Can::get_profile_btn_press(const uint32_t expire_time_ms) {
    return ewm_profile_btn_press(&this->ewm, this->seen(SEEN_EWM_230), expire_time_ms);
}

ProfileSwitchPos Egs51Can::get_profile_switch_pos(const uint32_t expire_time_ms) {
    return ewm_profile_switch_pos(&this->ewm, this->seen(SEEN_EWM_230), expire_time_ms);
}

// Setters, same as UN52 can_egs51.cpp (The GS_418 parts are left out)

void Egs51Can::set_clutch_status(TccClutchStatus status) {
    switch(status) {
        case TccClutchStatus::Open:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = false;
            break;
        case TccClutchStatus::OpenToSlipping:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = true;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Slipping:
            gs218.TCC_CLOSED = false;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::SlippingToClosed:
            gs218.TCC_CLOSED = true;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = true;
            break;
        case TccClutchStatus::Closed:
            gs218.TCC_CLOSED = true;
            gs218.TCC_OPEN = false;
            gs218.TCC_SLIPPING = false;
            break;
        default:
            break;
    }
}

void Egs51Can::set_actual_gear(GearboxGear actual) {
    switch(actual) {
        case GearboxGear::First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GIC = GS_218h_GIC_EGS51::G_SNV;
            break;
    }
}

void Egs51Can::set_target_gear(GearboxGear target) {
    switch(target) {
        case GearboxGear::First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_D5;
            break;
        case GearboxGear::Park:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_P;
            break;
        case GearboxGear::Neutral:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_N;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_R2;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GZC = GS_218h_GZC_EGS51::G_SNV;
            break;
    }
}

void Egs51Can::set_shifter_position(ShifterPosition pos) {
    if (ShifterPosition::N == pos || ShifterPosition::P == pos) {
        this->gs218.PN = true;
    } else {
        this->gs218.PN = false;
    }
}

void Egs51Can::set_gearbox_ok(bool is_ok) {
    this->gs218.GB_OK = is_ok;
    this->gs218.LIMP_MODE = !is_ok;
}

void Egs51Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        this->gs218.TORQUE_REQ_EN = false;
        this->gs218.SE = false;
        this->gs218.TORQUE_REQ = 0xFE;
    } else {
        // Just enable the request
        this->gs218.TORQUE_REQ_EN = true;
        this->gs218.SE = true;
        int32_t floor = MAX(0, (int32_t)(amount_nm/3.0F) + (this->drag_trq/3));
        this->gs218.TORQUE_REQ = MIN(0xFD, floor);
    }
}

void Egs51Can::set_garage_shift_state(bool enable, bool to_d) {
    this->gs218.GARAGE_SHIFT = enable;
}

void Egs51Can::set_tcc_trq_multiplier(float multi) {
    float clamped = MAX(100, MIN(254, multi*100));
    gs218.TCC_MULTI = (uint8_t)clamped;
}

void Egs51Can::tx_frames() {
    GS_218_EGS51 gs_218tx;
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_218tx = {gs218.raw};
    // Now set CVN Counter (Increases every frame)
    gs_218tx.FEHLER = cvn_counter;
    cvn_counter++;
    this->send_frame(GS_218_EGS51_CAN_ID, 6, gs_218tx.raw);
}
