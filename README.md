# MinTCU
I need to shift

MinTCU is a standalone gearbox controller (TCU) for the Mercedes 722.6 automatic, running on a plain Arduino Mega 2560. It is a port of [ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw) (UN52), the open source EGS52 replacement, from the ESP32 to the Mega.

Yes this code is borrowed from UN52 because it works well, I can use stock calibration data and I don't want to reinvent the wheel. The UN52 names, maps and defaults are kept as they are, and the calibration is the original Mercedes EGS data, so nothing in here is made up.

I've never touched c++, I'm learning while doing, I'm realizing it takes a while

---

The vibecoding begins 😬

> **Status: not tested on a car yet.** The firmware compiles and has only been run in a simulator (simavr) with fake sensors and CAN frames. It has not driven a real gearbox. Until it has, treat everything below as "should work", and be ready for the gearbox to sit in limp mode.

## Contents

- [What it is for](#what-it-is-for)
- [What it does](#what-it-does)
- [What you need](#what-you-need)
- [Wiring](#wiring)
- [Build and flash](#build-and-flash)
- [Set it up for your car](#set-it-up-for-your-car)
- [Calibration](#calibration)
- [Before the first test](#before-the-first-test)
- [Using it](#using-it)
- [Serial log](#serial-log-115200-10-hz-csv)
- [How it works, in detail](#how-it-works-in-detail)
- [Credits](#credits)

## What it is for

The car this is built for is a W124 that has been given:

- a 722.634 gearbox (from a W210 with an EGS51 TCU, chassis/variant 210.016),
- the W210 CAN selector lever (EWM) with the S/W switch,
- the W210 EDC15C6 (CR2) engine ECU, from an EGS51 car.

The W124 has no CAN of its own, so the EDC15 and the EWM are the only things on the bus. MinTCU listens to them and drives the gearbox. Any car with a 722.6, an EWM and an engine ECU that sends the EGS51 or EGS52 CAN frames should be close, but you will need the calibration for your gearbox (see [Calibration](#calibration)) and you will have to check the settings for your car.

## What it does

- Reads engine torque, RPM, pedal and coolant temperature from the engine ECU over CAN (EGS51 or EGS52 frames).
- Reads the selector, + / -, kickdown and the program switch (or button) from the EWM.
- **Listens only. MinTCU never sends anything on CAN**, so there are no torque requests to the engine and nothing on the cluster.
- Reads the N2/N3 input speed sensors, the ATF temperature / park-neutral circuit, battery voltage and (optionally) an output speed sensor.
- Line and shift pressure (MPC, SPC), garage shifts (P/N to D/R), automatic shifting with all six UN52 profiles, manual + / - shifts, torque converter lock-up (TCC) with adaptation saved to EEPROM.
- Failsafe and limp mode, with a fault memory in EEPROM.
- A 10 Hz CSV log on the USB serial port.

What it does not do (yet):

- No current sensing. The solenoids are driven open loop (the PWM duty is calculated, not measured), so there is no solenoid fault detection either.
- No shift adaptation.
- No brake input (the W124 has none on CAN).
- No settings over CAN or a config app. Everything is set in the source and flashed.

## What you need

**Electronics**

- Arduino Mega 2560 (a plain one, 5 V).
- An MCP2515 CAN module (SPI). Check whether its crystal is 8 or 16 MHz.
- One logic level low side MOSFET per solenoid, 6 in total: MPC, SPC, Y3, Y4, Y5 and TCC. Built and calculated with IRLB3034.
- A flyback diode across each solenoid (1N5822, the duty calculation assumes it).
- A 10k gate pull-down on each MOSFET, so the solenoids are off while the Mega resets. Off is the safe state: MPC/SPC at 0 mA is maximum pressure and the gearbox stays in its limp gear.
- A battery voltage divider (100k / 22k, same as UN52) and a pull-up for the ATF sensor (2k).
- Signal conditioning for N2, N3 and the output speed sensor. The Mega pins need a clean 0 / 5 V square wave and must never see more than 5 V.
- Optional but recommended: an output speed sensor. The plan for this car is an inductive proximity sensor reading the bolt heads on the prop shaft flex disc. Without it, output speed is calculated from the input speed and the gear, and MinTCU can't tell if it is in the wrong gear.

**Car side**

- Engine ECU sending EGS51 frames (MS_310, MS_210, MS_308, MS_608) or EGS52 frames (MS_312, MS_212, MS_210, MS_308, MS_608).
- An EWM selector (EWM_230 frame).
- The 722.6 electrical plug: the solenoids, N2/N3 and the ATF temperature sensor.

**Software**

- [PlatformIO](https://platformio.org/) (VS Code extension or the `pio` command line). It downloads the AVR toolchain and the `autowp-mcp2515` library by itself.
- Python 3, only if you need a different calibration.
- A serial terminal at 115200 baud to read the log.

## Wiring

All of it is in `src/board_config.h`. Check it against your board before flashing.

| Mega pin | Connects to | Notes |
| --- | --- | --- |
| 11 | MPC (modulating pressure) MOSFET gate | Timer1, must stay on 11 |
| 12 | SPC (shift pressure) MOSFET gate | Timer1, must stay on 12 |
| 6 | Y3 shift solenoid (1-2 / 4-5) MOSFET gate | Timer4 PWM |
| 7 | Y4 shift solenoid (3-4) MOSFET gate | Timer4 PWM |
| 8 | Y5 shift solenoid (2-3) MOSFET gate | Timer4 PWM |
| 5 | TCC (torque converter) MOSFET gate | Timer3 |
| 18 | N2 input speed sensor | 0 / 5 V, 60 pulses/rev |
| 19 | N3 input speed sensor | 0 / 5 V, 60 pulses/rev |
| 3 | Output speed sensor (optional) | 0 / 5 V |
| A0 | ATF temperature sensor (and park/neutral) | 2k pull-up to 5 V |
| A1 | Battery (solenoid supply) voltage | via 100k / 22k divider |
| 53 | MCP2515 CS | |
| 2 | MCP2515 INT | must be an interrupt pin |
| 50 / 51 / 52 | MCP2515 SO / SI / SCK | the Mega's SPI pins |

The solenoids are switched on their ground side: battery + to the solenoid, solenoid to the MOSFET drain, source to ground. The flyback diode goes across the solenoid (cathode to battery +).

## Build and flash

1. Install PlatformIO and open this folder.
2. Plug in the Mega and run `pio run -t upload` (or Upload in VS Code). The environment is `megaatmega2560` in `platformio.ini`.
3. Open the serial monitor with `pio device monitor` (115200 baud is set in `platformio.ini`).

It has been compiled with avr-gcc and the Arduino core directly, but not yet with PlatformIO itself, so please report it if `pio run` fails. It uses about 92 KB of flash and 2.7 KB of the Mega's 8 KB RAM.

## Set it up for your car

Everything is set in the source. These are the places to look:

- `src/board_config.h`: pins, the MCP2515 crystal (`CAN_CRYSTAL`), the real 5 V of your board (`ADC_VREF_MV`), the battery divider and the ATF pull-up.
- `src/nvs/eeprom_config.cpp` (`VEHICLE_CONFIG`, UN52 names):
  - `egs_can_type`: 1 = EGS51 frames, 2 = EGS52 frames. Default 1 (EDC15C6 from an EGS51 car).
  - `engine_type`: 0 = diesel, 1 = petrol. Picks the shift maps and the redline (`red_line_rpm_diesel` / `red_line_rpm_petrol`).
  - `input_sensor_pulses_per_rev`: pulses per revolution of your output speed sensor. Leave it at 0 until the sensor is fitted.
  - `diff_ratio`, `wheel_circumference`, `engine_drag_torque`: the UN52 defaults, change them for your car.
  - `default_profile`: the profile used before one has been saved.
- `src/nvs/module_settings.h`:
  - `cc_reference_resistance`: the MPC/SPC solenoid + harness resistance (measure it at the TCU plug).
  - `ewm_selector_type`: `Switch` for an EWM with the S/W switch (default here), `Button` for one with the program button.
  - `switch_profile_idx_top` / `switch_profile_idx_bottom`: the profiles for the two switch positions (default Comfort / Standard).
  - `coast_lock_profiles`: the profiles where the torque converter stays locked when you lift off, for engine braking (default Agility, Manual, Race).

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

Without a valid calibration MinTCU stays in failsafe (flag `10` in the log).

## Before the first test

1. Check the pins and constants at the top of `src/board_config.h` (MCP2515 CS/INT, crystal, ADC reference, battery divider, ATF pull-up).
2. Every solenoid MOSFET needs a flyback diode across the solenoid and a 10k gate pull-down, so a reset means 0 mA.
3. Measure the MPC solenoid + harness resistance at the TCU plug and put it in `cc_reference_resistance` (`src/nvs/module_settings.h`).
4. Check the calibration is for your gearbox (see above). Without one MinTCU stays in failsafe.
5. Set `egs_can_type` and, once the output sensor is fitted, `input_sensor_pulses_per_rev` in `src/nvs/eeprom_config.cpp`.
6. Put an ammeter in series with the MPC once and compare it with the `mpc_ma_est` column.
7. Check which way round the S/W switch is with the `prof` log column, and pick the two profiles with `switch_profile_idx_top` / `switch_profile_idx_bottom` in `src/nvs/module_settings.h`.
8. First drive: watch the log. `failsafe` should be `0` with the engine running, and N2/N3 should agree in 2nd, 3rd and 4th.

## Using it

- **D / R**: select them with the car stopped. Without an output speed sensor MinTCU can't see that the car is still rolling. D starts in 2nd and the profile shifts from there.
- **S/W switch** (or program button): picks the profile. The `prof` column shows which one is active.
- **+ / -**: one gear up or down, in any profile. It refuses a shift that would put the engine under 900 RPM or over the redline.
- **Kickdown**: asks for a downshift.
- **Faults**: stored faults are printed at boot (`Stored faults: 0x..`). Send `C` over the serial port to clear them. See [Fault memory](#how-it-works-in-detail) for the bits.
- **Failsafe / limp**: if something is wrong (no CAN, no calibration, bad sensors, battery out of range) the solenoids go to maximum pressure and the gear is held. It comes back by itself 1 s after the problem is gone.

## Serial log (115200, 10 Hz, CSV)

`ms,rpm,n2,n3,input_rpm,output_rpm,ratio,pedal,trq_sta,trq_drv,trq_in,shifter,gear,atf,vbatt,mpc_targ,mpc_corr,mpc_ma,mpc_ma_est,duty_permille,spc_ma,shift_sol,garage,shift,tcc,slip,prof,faults,failsafe,can_ovf`

- `output_rpm` ends with `e` when it is calculated (no output speed sensor), `ratio` is input/output speed (0 when stopped or in P/N)
- `shift_sol` is the active shift circuits in hex: 1 = 1-2 (Y3), 2 = 2-3 (Y5), 4 = 3-4 (Y4), 8 = 4-5 (Y3)
- `garage` is `S<stage>.<substage>` during a garage shift, otherwise the last result: 0 = none yet, 1 = OK, 2 = aborted, 3 = skipped (no input speed)
- `shift` is `C<n>.<phase>` (crossover) or `R<n>.<phase>` (releasing) during an up/downshift, `n` is the UN52 `GearChange` (1 = 1-2, 2 = 2-3, 3 = 3-4, 4 = 4-5, 5 = 2-1, 6 = 3-2, 7 = 4-3, 8 = 5-4). Otherwise the last result: 0 = none yet, 1 = OK, 2 = failed, 3 = aborted
- `tcc` is `O` (open) or `S` (slipping / locked) and the TCC pressure in mbar, `slip` is engine RPM - input RPM
- `prof` is the profile letter (S C W A M R), with a `K` after it while kickdown is pressed
- `faults` is the fault bits seen since the engine was started (see Limp mode / Fault memory below)
- `gear` numbers are the UN52 `GearboxGear` enum (1 = 1st ... 5 = 5th, 8 = P, 9 = N, 10 = R1)
- `atf` ends with `s` when the sensor is missing and 120 C is used for the pressure map
- `failsafe` is hex bit flags: 1 = engine off, 2 = no torque, 4 = no shifter, 8 = recovering, 10 = no calibration, 20 = limp mode
- `can_ovf` counts lost CAN frames, it should stay at 0

## How it works, in detail

The pressure part of UN52, garage shifts (N/P to D/R), automatic shifting with the UN52 profiles, and + / - on the selector:

- CAN (MCP2515), EGS51 or EGS52 frame set, chosen with `egs_can_type` in `src/nvs/eeprom_config.cpp` (UN52 numbers: 1 = EGS51, 2 = EGS52). Default is EGS51 (W210 EDC15C6 from an EGS51 car).
  - EGS51: torque (MS_310), pedal + ESP torque (MS_210), RPM (MS_308), coolant temp (MS_608)
  - EGS52: torque (MS_312, MS_212), pedal (MS_210), RPM (MS_308), coolant temp (MS_608)
  - Both: shifter, kickdown and program button / switch (EWM_230)
- Speed sensors: N2 and N3 (pins 18/19, 60 pulses/rev) give the input shaft speed like UN52 (`calc_turbine_rpm`). A dedicated output speed sensor goes on pin 3. Its pulses per rev go in `input_sensor_pulses_per_rev` (UN52 name) in `src/nvs/eeprom_config.cpp`, while that is 0 the output sensor is not used. The pins take a clean 0/5 V square wave, condition the sensor signals first.
- MPC pressure from input torque (with the torque converter multiplication from the input speed), same calculation as UN52 (`find_working_mpc_pressure` -> `calc_current_linear_sol` -> pcs map -> mA).
- No current sensing: the PWM duty is calculated from the wanted current, the battery voltage and the coil resistance at ATF temperature (open loop, see `src/solenoids/cc_solenoid.h`).
- Garage shifts, same as UN52: moving the selector from P/N to D or R engages the clutch (B2 for D, B3 for R) with the UN52 SPC/MPC ramps and shift solenoids, watching the input speed (N2/N3) slow down. Back to N/P aborts it. D engages 2nd (the 722.6 limp gear, same as UN52), the reverse gear is calculated as R1 (more pressure). If N2/N3 read nothing with the engine running, or in failsafe, D/R engages like limp mode (SPC at max pressure). Without the output speed sensor the 'car is still moving' check (output_rpm_force_n) cannot work, so only select D or R when stopped.
- Automatic shifting, same as UN52 (`src/profiles.cpp`): all six profiles (Standard, Comfort, Winter, Agility, Manual, Race) with the UN52 default shift maps (diesel or petrol from `engine_type`) and shift time maps. Kickdown (KD from the EWM) asks for a downshift, with UN52's kickdown re-arm. Before redline it upshifts on its own (all profiles but Race). Differences from UN52:
  - The maps are read straight from flash and are always the UN52 defaults (UN52 copies them into RAM so the config app can change them, that does not fit in the Mega's 8 KB RAM).
  - No brake pedal input (no BS_200 on the W124), so 'no upshift while braking' never happens.
  - Nothing is shown on the cluster (MinTCU does not send GS_418).
- Program selector, same as UN52 with the EWM (`src/shifter/`), set by `ETS_CURRENT_SETTINGS` in `src/nvs/module_settings.h` (UN52 defaults, except the selector type):
  - `ewm_selector_type = Button` (UN52 default): each press of the program button goes to the next enabled profile. By default that is Standard and Manual (`ewm_enable_c/w/a/r` are false). The profile is saved to EEPROM when going into P (`ewm_save_profile`, not for M/R/W unless `ewm_save_profile_manual`), so the car starts in it next time. Before anything is saved it starts in `default_profile` (Standard) from `eeprom_config.cpp`.
  - `ewm_selector_type = Switch` (MinTCU default, the W210 selector has the S/W switch): top = `switch_profile_idx_top` (Comfort), bottom = `switch_profile_idx_bottom` (Standard). Either can be Standard, Comfort, Agility, Manual or Race (`SelectableGearboxProfile`, Winter is not selectable in UN52).
  - UN52 does not read the program button or switch for EGS51 cars. The EWM sends the same frame there, so MinTCU uses UN52's EGS52 code for both.
  - `prof` in the log shows the profile, so you can check which way round your switch is.
- + / - on the EWM, same as UN52: in D asks for one gear up or down, in any profile. The shift is refused if the engine would go below 900 RPM (upshift) or above redline - 100 (downshift). The manual profile only downshifts on its own when the input speed is below 300 RPM with the pedal released, and upshifts at the redline. The UN52 crossover and releasing shift algorithms (`src/shifting_algo/`) run with the original shift calibration (SM06 for 210.016). Differences from UN52:
  - Torque requests, same as UN52: during an up/downshift the shift algorithm asks the engine for less torque (GS_218 `TORQUE_REQ`, in 3 Nm steps, with the engine drag torque added back like UN52), and the torque used for the pressures is held while it does (UN52's torque freeze). Which shifts ask is `en_trq_req_1_2` ... `en_trq_req_5_4` in `SBS_CURRENT_SETTINGS` (UN52 defaults). With the EGS52 frame set, or while the CAN controller is listen-only, the algorithms are told requests are off, and they add pressure instead (UN52 with torque requests off).
  - Not tested on a car yet: whether the EDC15C6 acts on the request has to be checked in the log (`trq_sta` should drop during a shift when `trq_req` has a value).
  - Fixed from UN52: `TCC_MULTI` in GS_218 is `bool TCC_MULTI: 8` in UN52's `GS51.h`, so UN52 sends 0x01 instead of 100-254. MinTCU's copy uses `uint8_t`.
  - No shift adaptation.
  - No output speed sensor yet: `output_rpm` is calculated from the input speed and the gear ratio. During a shift it carries on at the rate it was changing before the shift. So MinTCU cannot see a wrong gear (for example after a reset while driving, it assumes 2nd).
  - `diff_ratio` and `engine_drag_torque` in `eeprom_config.cpp` are the UN52 defaults (1.000 and 40 Nm).
- Shift solenoids Y3/Y4/Y5 (pins 6/7/8): 250 ms full on, then held at the UN52 hold PWM.
- Torque converter lock-up (TCC), same as UN52 (`src/torque_converter.cpp`): in a forward gear above 1000 RPM input it fills the converter clutch and then controls the slip (engine RPM - input RPM) to the UN52 slip target map, with the UN52 PID and adaptation. Differences from UN52:
  - The TCC adaptation is saved to the Mega's EEPROM (instead of the ESP32 NVS) when the selector goes into P with the parking lock in, same as UN52. It is written in the background, one byte at a time and only bytes that changed (`src/nvs/eeprom_impl.cpp`). If the saved data is missing or broken (CRC), that gear starts from the UN52 default (0) again.
  - The TCC solenoid is on **pin 5** (Timer3). It is driven like UN52 (100 Hz, full on for up to 1.5 ms, then the 10 kHz hold PWM), with the times rounded to 100 us.
  - The EDC15 (EGS51) does not ask for the TCC to open, so that part is never used. It is not read on EGS52 yet either.
  - Coasting: UN52 keeps an engaged converter slipping a little (70 rpm at 1000 rpm input, 40 rpm from 1500 to 3500, 10 rpm above 4000), and opens it below 1000 rpm input. MinTCU adds `coast_lock_profiles` (`TCC_MODULE_SETTINGS` in `src/nvs/module_settings.h`): in those profiles it stays locked (10 rpm target) on coast for maximum engine braking. Default: Agility, Manual and Race. Like UN52, a converter that is open when you lift off stays open.
- Outside a shift SPC is at 0 mA (max pressure), like limp mode.
- Failsafe: no calibration, engine off, torque frames missing for 200 ms, shifter missing, or battery outside 9-16 V -> MPC 0 mA = max pressure. It has to be OK for 1 s before it leaves failsafe. Watchdog resets the Mega if the loop hangs for 250 ms.
- Limp mode (MinTCU, not in UN52): the same as failsafe (MPC/SPC 0 mA = max pressure, shift solenoids and TCC off, the gear is held). Failsafe flag `20`. It leaves limp 1 s after the fault is gone. Faults that cause it:
  - N2 and N3 disagree by more than 100 rpm in 2nd/3rd/4th (UN52's sanity check), or N3 turns faster than 100 rpm in 1st/5th, for 1 s.
  - With the output sensor: the ratio matches no gear for 1 s (a slipping clutch).
  - A shift fails 3 times in a row.
- Fault memory (MinTCU): every fault is saved to EEPROM, printed at boot (`Stored faults: 0x..`), and shown in the `faults` log column for the current drive. Send `C` over serial to clear it. Bits (`GearboxFault` in `src/gearbox.h`): 1 = N2/N3, 2 = shift failed 3 times, 4 = ratio/slip, 8 = wrong gear found (UN52's gear correction), 10 = ATF sensor open/short (not in P), 20 = CAN frames lost and back with the engine running, 40 = battery voltage out of range with the engine running.
- No solenoid fault detection: that needs current sensing (UN52 measures solenoid current), MinTCU drives the solenoids open loop.

## Credits

- [ultimate-nag52-fw](https://github.com/rnd-ash/ultimate-nag52-fw) by rnd-ash and contributors: almost all of the gearbox logic, names and defaults.
- [ultimate-nag52-config-app](https://github.com/rnd-ash/ultimate-nag52-config-app): the `egs_db.bin` calibration database.
- [autowp-mcp2515](https://github.com/autowp/arduino-mcp2515): the MCP2515 driver.
- Much of the port to the Mega was written with Claude (Claude Code), as UN52's AI policy asks to be stated.
