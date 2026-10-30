#include "can_egs52.h"
#include <tcu_maths.h>

constexpr uint16_t Egs52Can::FILTERS[];

Egs52Can::Egs52Can(uint8_t cs_pin, uint8_t int_pin) : McpCan(cs_pin, int_pin, FILTERS, true) {
    // Same default values as UN52 (The GS_338/GS_418 ones are left out)
    this->gs218.raw = 0;
    this->gs218.MKRIECH = 0;
    this->set_target_gear(GearboxGear::SignalNotAvailable);
    this->set_actual_gear(GearboxGear::SignalNotAvailable);
    this->gs218.SCHALT = false;
    this->gs218.GIC = GS_218h_GIC_EGS52::G_SNV;
    gs218.CALID_CVN_AKT = true;
    gs218.G_G = false;
    this->gs218.ALF = true; // Fix for KG systems where cranking would stop when TCU turns on
    this->gs218.FPC_AAD = GS_218h_FPC_AAD_EGS52::SNV; // We don't use this (Yet)
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
        int16_t driver_converted = tmp;

        // Check if freezing torque should be done
        bool freeze = this->gs218.MMIN_EGS != 0 || this->gs218.MMAX_EGS != 0;
        // Change torque values based on freezing or not
        if (freeze) {
            driver_converted = MAX(driver_converted - this->req_static_torque_delta, static_converted);
        } else {
            this->req_static_torque_delta = driver_converted - static_converted;
        }
        // UN52 A/C compressor torque (KLA_410) is not ported, not subtracting it only makes pressure a bit higher.
        ret.m_ind = MAX(0, driver_converted);
        ret.m_converted_driver = driver_converted;
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

// Setters, same as UN52 can_egs52.cpp (The GS_338/GS_418 parts are left out)

void Egs52Can::set_clutch_status(TccClutchStatus status) {
    switch(status) {
        case TccClutchStatus::Open:
            gs218.K_G_B = false;
            gs218.K_O_B = true;
            gs218.K_S_B = false;
            break;
        case TccClutchStatus::OpenToSlipping:
            gs218.K_G_B = false;
            gs218.K_O_B = true;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::Slipping:
            gs218.K_G_B = false;
            gs218.K_O_B = false;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::SlippingToClosed:
            gs218.K_G_B = true;
            gs218.K_O_B = false;
            gs218.K_S_B = true;
            break;
        case TccClutchStatus::Closed:
            gs218.K_G_B = true;
            gs218.K_O_B = false;
            gs218.K_S_B = false;
            break;
        default:
            break;
    }
}

void Egs52Can::set_actual_gear(GearboxGear actual) {
    switch (actual) {
        case GearboxGear::Park:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_P;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_R2;
            break;
        case GearboxGear::Neutral:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_N;
            break;
        case GearboxGear::First:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GIC = GS_218h_GIC_EGS52::G_SNV;
            break;
    }
}

void Egs52Can::set_target_gear(GearboxGear target) {
    switch (target) {
        case GearboxGear::Park:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_P;
            break;
        case GearboxGear::Reverse_First:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_R;
            break;
        case GearboxGear::Reverse_Second:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_R2;
            break;
        case GearboxGear::Neutral:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_N;
            break;
        case GearboxGear::First:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D1;
            break;
        case GearboxGear::Second:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D2;
            break;
        case GearboxGear::Third:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D3;
            break;
        case GearboxGear::Fourth:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D4;
            break;
        case GearboxGear::Fifth:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_D5;
            break;
        case GearboxGear::SignalNotAvailable:
        default:
            this->gs218.GZC = GS_218h_GZC_EGS52::G_SNV;
            break;
    }
}

void Egs52Can::set_gearbox_ok(bool is_ok) {
    gs218.GET_OK = is_ok; // Gearbox OK
    gs218.GSP_OK = is_ok; // Gearbox profile OK
    gs218.GS_NOTL = !is_ok; // Emergency mode activated
}

void Egs52Can::set_garage_shift_state(bool enable, bool to_d) {
    gs218.KS = enable;
}

void Egs52Can::set_torque_request(TorqueRequestControlType control_type, TorqueRequestBounds limit_type, float amount_nm) {
    if (control_type == TorqueRequestControlType::None) {
        gs218.DYN0_AMR_EGS = false;
        gs218.DYN1_EGS = false;
    } else if (control_type == TorqueRequestControlType::FastAsPossible) {
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = false;
    } else if (control_type == TorqueRequestControlType::BackToDemandTorque) {
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = true;
    } else { // Normal speed
        gs218.DYN0_AMR_EGS = true;
        gs218.DYN1_EGS = false;
    }
    if (control_type != TorqueRequestControlType::None) {
        gs218.M_EGS = (amount_nm + 500) * 4;
        if (limit_type == TorqueRequestBounds::LessThan) {
            gs218.MMIN_EGS = true;
            gs218.MMAX_EGS = false;
        } else if (limit_type == TorqueRequestBounds::MoreThan) {
            gs218.MMIN_EGS = false;
            gs218.MMAX_EGS = true;
        } else {
            gs218.MMIN_EGS = true;
            gs218.MMAX_EGS = true;
        }
    } else {
        gs218.MMIN_EGS = false;
        gs218.MMAX_EGS = false;
        gs218.M_EGS = 0;
    }
}

/**
 * Parity calculation for torque numbers on GS218 and GS418
 * 
 * Each torque request member is a struct of 16 bits comprised of the following fields:
 * 1. Toggle bit (1 bit)
 * 2. Max request - bool (1 bit)
 * 3. Min request - bool (1 bit)
 * 4. Required torque - 13 bits
 */
inline bool calc_torque_parity(uint16_t s) {
    uint16_t p = s;
    p ^= (p >> 1);
    p ^= (p >> 2);
    p ^= (p >> 4);
    p ^= (p >> 8);
    return (p & 1) == 1;
}

// UN52 Egs52Can::tx_frames, GS_218 part
uint64_t Egs52Can::make_gs218_tx() {
    GS_218_EGS52 gs_218tx;
    // Copy current CAN frame values to here so we don't
    // accidentally modify parity calculations
    gs_218tx = {gs218.raw};

    // Firstly we have to deal with toggled bits!
    // As toggle bits need to be toggled every 40ms,
    // and egs52 Tx interval is 20ms,
    // we can achieve this with 2 booleans
    gs_218tx.MTGL_EGS = toggle;
    // Now do parity calculations
    gs_218tx.MPAR_EGS = calc_torque_parity(gs_218tx.raw >> 48);
    if (time_to_toggle) {
        toggle = !toggle;
    }
    time_to_toggle = !time_to_toggle;

    // Now set CVN Counter (Increases every frame)
    gs_218tx.FEHLER = cvn_counter;
    cvn_counter++;
    return gs_218tx.raw;
}

void Egs52Can::tx_frames() {
    // Always 8 bytes (Same as UN52)
    this->send_frame(GS_218_EGS52_CAN_ID, 8, this->make_gs218_tx());
}
