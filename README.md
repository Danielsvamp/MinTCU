# MinTCU
I need to shift

Yes this code is borrowed from [ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw) because it works well, I can use stock calibration data and I don't want to reinvent the wheel.

I've never touched c++, I'm learning while doing, I'm realizing it takes a while

## What runs right now

Only the pressure part of UN52, nothing shifts yet:

- CAN (MCP2515, **listen only**, it never transmits), EGS51 or EGS52 frame set, chosen with `egs_can_type` in `src/nvs/eeprom_config.cpp` (UN52 numbers: 1 = EGS51, 2 = EGS52). Default is EGS51 (W210 EDC15C6 from an EGS51 car).
  - EGS51: torque (MS_310), pedal + ESP torque (MS_210), RPM (MS_308), coolant temp (MS_608)
  - EGS52: torque (MS_312, MS_212), pedal (MS_210), RPM (MS_308), coolant temp (MS_608)
  - Both: shifter (EWM_230)
- Speed sensors: N2 and N3 (pins 18/19, 60 pulses/rev) give the input shaft speed like UN52 (`calc_turbine_rpm`). A dedicated output speed sensor (Hall) goes on pin 3. Its pulses per rev go in `input_sensor_pulses_per_rev` (UN52 name) in `src/nvs/eeprom_config.cpp`, while that is 0 the output sensor is not used. The pins take a clean 0/5 V square wave, condition the sensor signals first.
- MPC pressure from input torque (with the torque converter multiplication from the input speed), same calculation as UN52 (`find_working_mpc_pressure` -> `calc_current_linear_sol` -> pcs map -> mA).
- No current sensing: the PWM duty is calculated from the wanted current, the battery voltage and the coil resistance at ATF temperature (open loop, see `src/solenoids/cc_solenoid.h`).
- SPC is at 0 mA (max pressure), Y3/Y4/Y5/TCC are off, so the gearbox sits in 2nd in D and in reverse in R, same as limp mode.
- Failsafe: engine off, torque frames missing for 200 ms, shifter missing, or battery outside 9-16 V -> MPC 0 mA = max pressure. It has to be OK for 1 s before it leaves failsafe. Watchdog resets the Mega if the loop hangs for 250 ms.

## Before the first test

1. Check the pins and constants at the top of `src/board_config.h` (MCP2515 CS/INT, crystal, ADC reference, battery divider, ATF pull-up).
2. Every solenoid MOSFET needs a flyback diode across the solenoid and a 10k gate pull-down, so a reset means 0 mA.
3. Measure the MPC solenoid + harness resistance at the TCU plug and put it in `cc_reference_resistance` (`src/nvs/module_settings.h`).
4. Check the calibration is for your gearbox (see below). Without one MinTCU stays in failsafe.
5. Set `egs_can_type` and, once the output sensor is fitted, `input_sensor_pulses_per_rev` in `src/nvs/eeprom_config.cpp`.
6. Put an ammeter in series with the MPC once and compare it with the `mpc_ma_est` column.

## Calibration

The calibration is the original EGS data from the UN52 config app database (`egs_db.bin` in
[ultimate-nag52-config-app](https://github.com/rnd-ash/ultimate-nag52-config-app)). Nothing is typed in by hand,
`tools/egs_cal_gen.py` writes `src/egs_calibration/calibration_data.cpp` from it.

The one in the repo is for chassis 210.016 with a 722.634 (hydraulic STDP, mechanical var7, converter 0000).
All EGS part numbers for that chassis/gearbox in the database use the same data. For another car:

```
python3 tools/egs_cal_gen.py egs_db.bin --list --gearbox 722.634      # what is in the database
python3 tools/egs_cal_gen.py egs_db.bin --egs 0305452032 --chassis 210.016 --gearbox 722.634
```

## Serial log (115200, 10 Hz, CSV)

`ms,rpm,n2,n3,input_rpm,output_rpm,ratio,pedal,trq_sta,trq_drv,trq_in,shifter,gear,atf,vbatt,mpc_targ,mpc_corr,mpc_ma,mpc_ma_est,duty_permille,failsafe,can_ovf`

- `output_rpm` is -1 without an output speed sensor, `ratio` is input/output speed (0 when stopped or in P/N)
- `atf` ends with `s` when the sensor is missing and 120 C is used for the pressure map
- `failsafe` is hex bit flags: 1 = engine off, 2 = no torque, 4 = no shifter, 8 = recovering, 10 = no calibration
- `can_ovf` counts lost CAN frames, it should stay at 0
