# MinTCU
I need to shift

Yes this code is borrowed from [ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw) because it works well, I can use stock calibration data and I don't want to reinvent the wheel.

I've never touched c++, I'm learning while doing, I'm realizing it takes a while

## What runs right now

Only the pressure part of UN52, nothing shifts yet:

- CAN (MCP2515, **listen only**, it never transmits): engine torque (MS_312, MS_212), RPM (MS_308), pedal (MS_210), coolant temp (MS_608) and the shifter (EWM_230). This is the EGS52 frame set. If your car only sends 0x310 instead of 0x312/0x212 it is the EGS51 set, and the CAN layer has to change.
- MPC pressure from input torque, same calculation as UN52 (`find_working_mpc_pressure` -> `calc_current_linear_sol` -> pcs map -> mA).
- No current sensing: the PWM duty is calculated from the wanted current, the battery voltage and the coil resistance at ATF temperature (open loop, see `src/solenoids/cc_solenoid.h`).
- SPC is at 0 mA (max pressure), Y3/Y4/Y5/TCC are off, so the gearbox sits in 2nd in D and in reverse in R, same as limp mode.
- Failsafe: engine off, torque frames missing for 200 ms, shifter missing, or battery outside 9-16 V -> MPC 0 mA = max pressure. It has to be OK for 1 s before it leaves failsafe. Watchdog resets the Mega if the loop hangs for 250 ms.

## Before the first test

1. Check the pins and constants at the top of `src/board_config.h` (MCP2515 CS/INT, crystal, ADC reference, battery divider, ATF pull-up).
2. Every solenoid MOSFET needs a flyback diode across the solenoid and a 10k gate pull-down, so a reset means 0 mA.
3. Measure the MPC solenoid + harness resistance at the TCU plug and put it in `cc_reference_resistance` (`src/nvs/module_settings.h`).
4. Fill `TorqueConverterCalibration` in `src/egs_calibration/calibration_structs.h` from your EGS calibration (the values there are placeholders).
5. Put an ammeter in series with the MPC once and compare it with the `mpc_ma_est` column.

## Serial log (115200, 10 Hz, CSV)

`ms,rpm,pedal,trq_sta,trq_drv,trq_in,shifter,gear,atf,vbatt,mpc_targ,mpc_corr,mpc_ma,mpc_ma_est,duty_permille,failsafe,can_ovf`

- `atf` ends with `s` when the sensor is missing and 120 C is used for the pressure map
- `failsafe` is hex bit flags: 1 = engine off, 2 = no torque, 4 = no shifter, 8 = recovering
- `can_ovf` counts lost CAN frames, it should stay at 0
