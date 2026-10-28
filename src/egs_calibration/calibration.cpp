#include "calibration_structs.h"

// Compiled in calibration, values come from the struct defaults in calibration_structs.h
static HydraulicCalibration HYDR_CAL;
static MechanicalCalibration MECH_CAL;
static TorqueConverterCalibration TCC_CAL;

HydraulicCalibration* HYDR_PTR = &HYDR_CAL;
MechanicalCalibration* MECH_PTR = &MECH_CAL;
TorqueConverterCalibration* TCC_CFG_PTR = &TCC_CAL;
