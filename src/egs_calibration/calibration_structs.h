#ifndef __CALIBRATION_STRUCT_H_
#define __CALIBRATION_STRUCT_H_

#include <stdint.h>

/**
 * Calibration data from EGS52/53 ported to UN52
*/

#define SHIFT_ARRAY_LEN 8 // Arrays with all 8 shifts (1-2 2-3 3-4 4-5 2-1 3-2 4-3 5-4)

typedef struct {
    // Momentum (Up)
    // X: Output speed (x30)
    // Y: Engine torque (Static) (x5)
    // Z: Momentum (Nm) (x5)
    uint8_t momentum_1_2_x[3];
    uint8_t momentum_2_3_x[3];
    uint8_t momentum_3_4_x[3];
    uint8_t momentum_4_5_x[3];
    uint8_t momentum_1_2_y[2];
    uint8_t momentum_2_3_y[2];
    uint8_t momentum_3_4_y[2];
    uint8_t momentum_4_5_y[2];
    uint8_t momentum_1_2_z[6];
    uint8_t momentum_2_3_z[6];
    uint8_t momentum_3_4_z[6];
    uint8_t momentum_4_5_z[6];
    // Momentum (Down)
    uint8_t momentum_2_1_x[6];
    uint8_t momentum_3_2_x[6];
    uint8_t momentum_4_3_x[6];
    uint8_t momentum_5_4_x[6];
    uint8_t momentum_2_1_y[10];
    uint8_t momentum_3_2_y[10];
    uint8_t momentum_4_3_y[10];
    uint8_t momentum_5_4_y[10];
    uint8_t momentum_2_1_z[60];
    uint8_t momentum_3_2_z[60];
    uint8_t momentum_4_3_z[60];
    uint8_t momentum_5_4_z[60];
    // Torque adder (Up)
    uint8_t trq_adder_1_2_x[6];
    uint8_t trq_adder_2_3_x[6];
    uint8_t trq_adder_3_4_x[6];
    uint8_t trq_adder_4_5_x[6];
    uint8_t trq_adder_1_2_y[8];
    uint8_t trq_adder_2_3_y[8];
    uint8_t trq_adder_3_4_y[8];
    uint8_t trq_adder_4_5_y[8];
    uint8_t trq_adder_1_2_z[48];
    uint8_t trq_adder_2_3_z[48];
    uint8_t trq_adder_3_4_z[48];
    uint8_t trq_adder_4_5_z[48];
    // Torque adder (Down)
    uint8_t trq_adder_2_1_x[3];
    uint8_t trq_adder_3_2_x[3];
    uint8_t trq_adder_4_3_x[3];
    uint8_t trq_adder_5_4_x[3];
    uint8_t trq_adder_2_1_y[4];
    uint8_t trq_adder_3_2_y[4];
    uint8_t trq_adder_4_3_y[4];
    uint8_t trq_adder_5_4_y[4];
    uint8_t trq_adder_2_1_z[12];
    uint8_t trq_adder_3_2_z[12];
    uint8_t trq_adder_4_3_z[12];
    uint8_t trq_adder_5_4_z[12];
} __attribute__ ((packed)) ShiftAlgorithmPack;

typedef struct {
    uint16_t p_multi_1 = 431;
    uint16_t p_multi_other = 592;
    uint16_t lp_reg_spring_pressure = 1828;
    uint16_t min_mpc_pressure = 500;
    uint8_t filter_factor = 15;
    uint8_t mpc_flush_temp_threshold = 75;
    uint16_t mpc_no_flush_time = 30000;
    uint16_t mpc_flush_time = 50;
    uint16_t extra_p_not_shifting = 0;
    uint16_t shift_pressure_addr_percent = 30;
    uint16_t inlet_pressure_offset = 1000;
    uint16_t inlet_pressure_input_min = 3180;
    uint16_t inlet_pressure_input_max = 8820;
    uint16_t inlet_pressure_output_min = 2690;
    uint16_t inlet_pressure_output_max = 8820;
    uint16_t extra_pressure_pump_speed_min = 1000;
    uint16_t extra_pressure_pump_speed_max = 4000;
    uint16_t extra_pressure_adder_r1_1 = 1500;
    uint16_t extra_pressure_adder_other_gears = 1000;
    uint16_t shift_pressure_addr_percent = 37;

    // Pressure solenoid current map
    uint16_t pcs_map_x[7] = { // X axis of pcsKF. Pressure in mbar

        50, 600, 1000, 2350, 5600, 6600, 7700
    };

    uint16_t pcs_map_y[4] = { // Y axis of pcsKF. These are raw values, true temperature is offset by -50

        25, 70, 110, 200
    };

    uint16_t pcs_map_z[28] = { // Z of pcsKF. Current in mA(?)
        //7, 4,

        //50, 600, 1000, 2350, 5600, 6600, 7700
        
        //25, 70, 110, 200

        1100,	1085,	954,	700,	450,	350,	200,
        1077,	925,	830,	675,	415,	320,	0,
        1000,	835,	780,	650,	400,	288,	0,
        975,	795,	745,	625,	370,	260,	0,
    };


} __attribute__ ((packed)) HydraulicCalibration;

typedef struct {
    uint16_t ratio_table[8] = {0, 3595, 2186, 1405, 1000, 831, 3167, 1926};

    uint8_t gb_ty = 0;
    uint16_t intertia_factor[8] = {1645, 1556, 1405, 1203, 1644, 1556, 1405, 1203};

    //Per clutch/gear friction map
    uint16_t friction_map[48] = {/*
        X-Axis 
        K1      K2       K3      B1     B2      B3    | Y-Axis      */
        4709,	0,	    0,	    3574,	0,  	0,      // 0
        0,	    0,	    3076,	2303,	2685,	0,      // 1
        1845,	0,	    1871,	0,	    1633,	0,      // 2
        0,	    1101,	0,	    0,	    1109,	0,      // 3
        958,	1673,	971,	0,	    0,  	0,      // 4
        0,	    1390,	807,	604,	0,	    0,      // 5
        0,	    0,	    3076,	2303,	0,	    3387,   // -1
        1845,	0,	    1871,	0,	    0,	    2060,   // -2
    };

    uint16_t max_torque_on_clutch[4] = {1000, 896, 564, 1000};
    uint16_t max_torque_off_clutch[4] = {1000, 928, 1518, 758};
    uint16_t release_spring_pressure[6] = {1102, 949, 1139, 1033, 1512, 274};
    uint16_t intertia_torque[8] = {18, 26, 126, 18, 18, 26, 126, 255};
    uint8_t strongest_loaded_clutch_idx[8] = {2, 2, 1, 1, 1, 2, 2, 16};
    uint16_t turbine_drag[8] = {10, 35, 59, 16, 18, 25, 30, 121};
    uint16_t atf_density_minus_50c = 828;
    uint16_t atf_density_drop_per_c = 0;
    uint16_t atf_density_centrifugal_force_factor[3] = {64, 40120, 2847};

    //fix this
    uint16_t tccKF[35] = { // Z of pcsKF. Current in mA(?)
        //7, 4,

        //50, 600, 1000, 2350, 5600, 6600, 7700
        
        //25, 70, 110, 200

        0,	7680,	15360,	20480,	30720,	40960,	64535,
        0,	8960,	16640,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
        0,	10240,	17920,	20480,	30720,	40960,	64535,
    };

    // Strongest currently holding clutch per gear
    uint8_t strongestClutchKL[8] = { // Axis - Gears 0, 1, 2, 3, 4, 5, -1, -2
        255, 2, 2, 1, 1, 1, 2, 2
    };

    // Release spring pressure per clutch
    uint16_t releaseSpringPressureKL[6] = { // Axis - Clutches K1-B3
        1270, 846, 1205, 1139, 1289, 488
    };
} __attribute__ ((packed)) MechanicalCalibration;

typedef struct {
    uint16_t multiplier_map_x[2];
    uint16_t multiplier_map_z[2];
    uint16_t pump_map_x[11];
    uint16_t pump_map_z[11];
} __attribute__ ((packed)) TorqueConverterCalibration;

typedef struct {
    uint8_t unk;
    uint8_t _padding;
    uint16_t min_trq_filling_phase;
    uint16_t min_trq_filling_ramp[8];
    uint16_t unk1;
    uint16_t unk2;
    uint16_t release_filling_p[5];
    uint8_t cycles_ramp_to_low_filling;
    uint8_t cycles_low_filling_p[5];
    uint8_t max_trq_ramp_filling;
    uint8_t cycles_fill_ramp1;
    uint8_t cycles_fill_ramp2;
    uint8_t _padding1;
    uint16_t fill_hold1_p;
    uint16_t fill_hold2_p;
    uint8_t extra_p_filling_doubleshift[3];
    uint8_t unk_temp1;
    uint8_t unk_temp2;
    uint8_t filling_trq_lim_c;
    int16_t tolorance_trq_filling;
    uint16_t tolorance_filling_43;
    uint16_t tolorance_filling_32_21;
    uint16_t high_filling_p[5];
    uint16_t max_trq_change_filling;
    uint8_t something_trq_ramp_mclaren;
    uint8_t something_filling_time_mclaren[5];
    uint8_t temp_very_cold_filling;
    uint8_t _padding3;
    uint16_t very_cold_filling_p[5];
    uint16_t rpm_thresh_skip_filling1;
    uint16_t unk_p1;
    uint8_t min_temp_adapt_43;
    uint8_t padding3;
    uint16_t trq_threshold_reduction_filling_p;
    uint16_t max_reduction_filling_p[6];
    uint8_t downshift_pedal_jump_abort[5];
} __attribute__ ((packed)) FillingCalibration;

typedef struct {
    uint32_t magic;
    uint16_t len;
    uint16_t crc;
    char tcc_cal_name[16];
    TorqueConverterCalibration tcc_cal;
    char mech_cal_name[16];
    MechanicalCalibration mech_cal;
    char hydr_cal_name[16];
    HydraulicCalibration hydr_cal;
    char shift_algo_pack_name[16];
    ShiftAlgorithmPack shift_algo_cal;
    //char filling_cal_name[16];
    //FillingCalibration filling_cal;
} __attribute__ ((packed)) CalibrationInfo;
// To check if we overflow

extern CalibrationInfo* CAL_RAM_PTR;
extern HydraulicCalibration* HYDR_PTR;
extern MechanicalCalibration* MECH_PTR;
extern TorqueConverterCalibration* TCC_CFG_PTR;
extern ShiftAlgorithmPack* SHIFT_ALGO_CFG_PTR;
extern FillingCalibration* FILLING_PTR;

#endif