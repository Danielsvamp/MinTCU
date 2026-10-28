#include "shifter_ewm.h"
#include "programselector/programselectorbuttonewm.h"
#include "programselector/programselectorswitchewm.h"
#include "../canbus/can_hal.h"
#include "../nvs/eeprom_config.h"

// Same as UN52 shifter_ewm.cpp (Without the SLR selector)

ShifterEwm::ShifterEwm(const ETS_MODULE_SETTINGS *shifter_settings)
{
	switch (shifter_settings->ewm_selector_type)
	{
	case EwmSelectorType::Switch:
		programselector = new ProgramSelectorSwitchEWM();
		break;
	case EwmSelectorType::Button:
		programselector = new ProgramSelectorButtonEwm();
		break;
	default:
		programselector = nullptr;
		break;
	}
}

AbstractProfile *ShifterEwm::get_profile(void)
{
	AbstractProfile *result = nullptr;
	if (nullptr != programselector)
	{
		result = programselector->get_profile();
	} else {
		// null selector can be if the selector has no profile button (Jeep/Sprinter)
		result = profiles[VEHICLE_CONFIG.default_profile];
	}
	return result;
}

void ShifterEwm::set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos)
{
	if (nullptr != programselector)
	{
		switch (programselector->get_type()) {
			case ProgramSelectorType::EWMButton:
				(reinterpret_cast<ProgramSelectorButtonEwm *>(programselector))->set_button_pressed(is_pressed);
				break;
			case ProgramSelectorType::EWMSwitch:
				(reinterpret_cast<ProgramSelectorSwitchEWM *>(programselector))->set_profile_switch_pos(pos);
				break;
			default:
				break;
		}
	}
}

ShifterStyle ShifterEwm::get_shifter_type() {
	return ShifterStyle::EWM;
}

void ShifterEwm::update(void)
{
	set_program_button_pressed(egs_can_hal->get_profile_btn_press(expire_time_ms), egs_can_hal->get_profile_switch_pos(expire_time_ms));
}
