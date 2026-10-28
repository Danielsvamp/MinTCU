// NO CALIBRATION GENERATED YET. Run tools/egs_cal_gen.py to create this file from the UN52
// config app egs_db.bin (see README). Until then the gearbox stays in failsafe (MPC 0mA = max pressure).
#include "calibration_structs.h"

extern const bool CAL_GENERATED;
const bool CAL_GENERATED = false;
extern const char CAL_DESCRIPTION[];
const char CAL_DESCRIPTION[] = "NONE";

HydraulicCalibration HYDR_CAL = {};

MechanicalCalibration MECH_CAL = {};

TorqueConverterCalibration TCC_CAL = {};
