# MinTCU
I need to shift

Yes this code is borrowed from [ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw) because it works well, I can use stock calibration data and I don't want to reinvent the wheel.

I've never touched c++, I'm learning while doing, I'm realizing it takes a while

## What runs right now

The pressure part of UN52, garage shifts (N/P to D/R), automatic shifting with the UN52 profiles, and + / - on the selector:

- CAN (MCP2515, **listen only**, it never transmits), EGS51 or EGS52 frame set, chosen with `egs_can_type` in `src/nvs/eeprom_config.cpp` (UN52 numbers: 1 = EGS51, 2 = EGS52). Default is EGS51 (W210 EDC15C6 from an EGS51 car).
  - EGS51: torque (MS_310), pedal + ESP torque (MS_210), RPM (MS_308), coolant temp (MS_608)
  - EGS52: torque (MS_312, MS_212), pedal (MS_210), RPM (MS_308), coolant temp (MS_608)
  - Both: shifter, kickdown and program button / switch (EWM_230)
- Speed sensors: N2 and N3 (pins 18/19, 60 pulses/rev) give the input shaft speed like UN52 (`calc_turbine_rpm`). A dedicated output speed sensor (Hall) goes on pin 3. Its pulses per rev go in `input_sensor_pulses_per_rev` (UN52 name) in `src/nvs/eeprom_config.cpp`, while that is 0 the output sensor is not used. The pins take a clean 0/5 V square wave, condition the sensor signals first.
- MPC pressure from input torque (with the torque converter multiplication from the input speed), same calculation as UN52 (`find_working_mpc_pressure` -> `calc_current_linear_sol` -> pcs map -> mA).
- No current sensing: the PWM duty is calculated from the wanted current, the battery voltage and the coil resistance at ATF temperature (open loop, see `src/solenoids/cc_solenoid.h`).
- Garage shifts, same as UN52: moving the selector from P/N to D or R engages the clutch (B2 for D, B3 for R) with the UN52 SPC/MPC ramps and shift solenoids, watching the input speed (N2/N3) slow down. Back to N/P aborts it. D engages 2nd (the 722.6 limp gear, same as UN52), the reverse gear is calculated as R1 (more pressure). If N2/N3 read nothing with the engine running, or in failsafe, D/R engages like limp mode (SPC at max pressure). Without the output speed sensor the 'car is still moving' check (output_rpm_force_n) cannot work, so only select D or R when stopped.
- Automatic shifting, same as UN52 (`src/profiles.cpp`): all six profiles (Standard, Comfort, Winter, Agility, Manual, Race) with the UN52 default shift maps (diesel or petrol from `engine_type`) and shift time maps. Kickdown (KD from the EWM) asks for a downshift, with UN52's kickdown re-arm. Before redline it upshifts on its own (all profiles but Race). Differences from UN52:
  - The maps are read straight from flash and are always the UN52 defaults (UN52 copies them into RAM so the config app can change them, that does not fit in the Mega's 8 KB RAM).
  - No brake pedal input (no BS_200 on the W124), so 'no upshift while braking' never happens.
  - Nothing is shown on the cluster (MinTCU never transmits).
- Program selector, same as UN52 with the EWM (`src/shifter/`), set by `ETS_CURRENT_SETTINGS` in `src/nvs/module_settings.h` (UN52 defaults, except the selector type):
  - `ewm_selector_type = Button` (UN52 default): each press of the program button goes to the next enabled profile. By default that is Standard and Manual (`ewm_enable_c/w/a/r` are false). The profile is saved to EEPROM when going into P (`ewm_save_profile`, not for M/R/W unless `ewm_save_profile_manual`), so the car starts in it next time. Before anything is saved it starts in `default_profile` (Standard) from `eeprom_config.cpp`.
  - `ewm_selector_type = Switch` (MinTCU default, the W210 selector has the S/W switch): top = `switch_profile_idx_top` (Comfort), bottom = `switch_profile_idx_bottom` (Standard). Either can be Standard, Comfort, Agility, Manual or Race (`SelectableGearboxProfile`, Winter is not selectable in UN52).
  - UN52 does not read the program button or switch for EGS51 cars. The EWM sends the same frame there, so MinTCU uses UN52's EGS52 code for both.
  - `prof` in the log shows the profile, so you can check which way round your switch is.
- + / - on the EWM, same as UN52: in D asks for one gear up or down, in any profile. The shift is refused if the engine would go below 900 RPM (upshift) or above redline - 100 (downshift). The manual profile only downshifts on its own when the input speed is below 300 RPM with the pedal released, and upshifts at the redline. The UN52 crossover and releasing shift algorithms (`src/shifting_algo/`) run with the original shift calibration (SM06 for 210.016). Differences from UN52:
  - No torque requests to the engine (MinTCU never transmits on CAN). This is UN52 with torque requests off, the algorithms add pressure instead.
  - No shift adaptation.
  - No output speed sensor yet: `output_rpm` is calculated from the input speed and the gear ratio. During a shift it carries on at the rate it was changing before the shift. So MinTCU cannot see a wrong gear (for example after a reset while driving, it assumes 2nd).
  - `diff_ratio` and `engine_drag_torque` in `eeprom_config.cpp` are the UN52 defaults (1.000 and 40 Nm).
- Shift solenoids Y3/Y4/Y5 (pins 6/7/8): 250 ms full on, then held at the UN52 hold PWM.
- Torque converter lock-up (TCC), same as UN52 (`src/torque_converter.cpp`): in a forward gear above 1000 RPM input it fills the converter clutch and then controls the slip (engine RPM - input RPM) to the UN52 slip target map, with the UN52 PID and adaptation. Differences from UN52:
  - The TCC adaptation is saved to the Mega's EEPROM (instead of the ESP32 NVS) when the selector goes into P with the parking lock in, same as UN52. It is written in the background, one byte at a time and only bytes that changed (`src/nvs/eeprom_impl.cpp`). If the saved data is missing or broken (CRC), that gear starts from the UN52 default (0) again.
  - The TCC solenoid is on **pin 5** (Timer3). It is driven like UN52 (100 Hz, full on for up to 1.5 ms, then the 10 kHz hold PWM), with the times rounded to 100 us.
  - The EDC15 (EGS51) does not ask for the TCC to open, so that part is never used. It is not read on EGS52 yet either.
- Outside a shift SPC is at 0 mA (max pressure), like limp mode.
- Failsafe: no calibration, engine off, torque frames missing for 200 ms, shifter missing, or battery outside 9-16 V -> MPC 0 mA = max pressure. It has to be OK for 1 s before it leaves failsafe. Watchdog resets the Mega if the loop hangs for 250 ms.

## Before the first test

1. Check the pins and constants at the top of `src/board_config.h` (MCP2515 CS/INT, crystal, ADC reference, battery divider, ATF pull-up). The TCC solenoid is on pin 5 (it was 13).
2. Every solenoid MOSFET needs a flyback diode across the solenoid and a 10k gate pull-down, so a reset means 0 mA.
3. Measure the MPC solenoid + harness resistance at the TCU plug and put it in `cc_reference_resistance` (`src/nvs/module_settings.h`).
4. Check the calibration is for your gearbox (see below). Without one MinTCU stays in failsafe.
5. Set `egs_can_type` and, once the output sensor is fitted, `input_sensor_pulses_per_rev` in `src/nvs/eeprom_config.cpp`.
6. Put an ammeter in series with the MPC once and compare it with the `mpc_ma_est` column.
7. Check which way round the S/W switch is with the `prof` log column, and pick the two profiles with `switch_profile_idx_top` / `switch_profile_idx_bottom` in `src/nvs/module_settings.h`.

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

`ms,rpm,n2,n3,input_rpm,output_rpm,ratio,pedal,trq_sta,trq_drv,trq_in,shifter,gear,atf,vbatt,mpc_targ,mpc_corr,mpc_ma,mpc_ma_est,duty_permille,spc_ma,shift_sol,garage,shift,tcc,slip,prof,failsafe,can_ovf`

- `output_rpm` ends with `e` when it is calculated (no output speed sensor), `ratio` is input/output speed (0 when stopped or in P/N)
- `shift_sol` is the active shift circuits in hex: 1 = 1-2 (Y3), 2 = 2-3 (Y5), 4 = 3-4 (Y4), 8 = 4-5 (Y3)
- `garage` is `S<stage>.<substage>` during a garage shift, otherwise the last result: 0 = none yet, 1 = OK, 2 = aborted, 3 = skipped (no input speed)
- `shift` is `C<n>.<phase>` (crossover) or `R<n>.<phase>` (releasing) during an up/downshift, `n` is the UN52 `GearChange` (1 = 1-2, 2 = 2-3, 3 = 3-4, 4 = 4-5, 5 = 2-1, 6 = 3-2, 7 = 4-3, 8 = 5-4). Otherwise the last result: 0 = none yet, 1 = OK, 2 = failed, 3 = aborted
- `tcc` is `O` (open) or `S` (slipping / locked) and the TCC pressure in mbar, `slip` is engine RPM - input RPM
- `prof` is the profile letter (S C W A M R), with a `K` after it while kickdown is pressed
- `gear` numbers are the UN52 `GearboxGear` enum (1 = 1st ... 5 = 5th, 8 = P, 9 = N, 10 = R1)
- `atf` ends with `s` when the sensor is missing and 120 C is used for the pressure map
- `failsafe` is hex bit flags: 1 = engine off, 2 = no torque, 4 = no shifter, 8 = recovering, 10 = no calibration
- `can_ovf` counts lost CAN frames, it should stay at 0
