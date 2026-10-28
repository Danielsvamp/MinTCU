#ifndef SHIFTER_EWM_H
#define SHIFTER_EWM_H

#include "shifter.h"
#include "../nvs/module_settings.h"
#include "programselector/programselector.hpp"

/*
    The program selector part of UN52 shifter_ewm.h (The gearbox reads the selector position from
    the CAN layer directly). No SLR selector and no diagnostics.
*/
class ShifterEwm
{
public:
	ShifterEwm(const ETS_MODULE_SETTINGS* shifter_settings);
	AbstractProfile* get_profile(void);
	void set_program_button_pressed(const bool is_pressed, const ProfileSwitchPos pos);
	ShifterStyle get_shifter_type(void);
	void update(void);
private:
	const uint32_t expire_time_ms = 250u;
	ProgramSelector* programselector;
};

#endif // SHIFTER_EWM_H
