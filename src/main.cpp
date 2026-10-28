/*
    Nu skaja faktist fösök få na ådentli byrjan
    Ja ska ha MPC ti arbeit från kalibrering å sensore
    Allt ska lånas från UN52 så langt he ba gar, fö he funkkar

    Now I'm actually gonna try to get a decent start
    I need MPC to work from calibration and sensors
    Everything must be borrowed from UN52 as far as possible, because that works

    What runs now:
    - CAN (MCP2515, listen only, EGS51 or EGS52 frames): engine torque, RPM, pedal, coolant temp, shifter position
    - N2/N3 input speed and a dedicated output speed sensor
    - MPC pressure from torque, same calculation as UN52, open loop current -> PWM
    - SPC at 0mA (max), Y3/Y4/Y5/TCC off, so the gearbox stays in 2nd (D) or R like limp mode
    - Failsafe: no torque / no shifter / engine off / bad battery voltage = 0mA = max pressure
*/

#include <Arduino.h>
#include <stdint.h>
#include <avr/wdt.h>
#include "board_config.h"
#include "gearbox.h"
#include "canbus/can_egs51.h"
#include "canbus/can_egs52.h"
#include "nvs/eeprom_config.h"
#include "tcu_io/tcu_io.hpp"
#include "sensors.h"
#include "egs_calibration/calibration_structs.h"

extern const char CAL_DESCRIPTION[];

// After a watchdog reset the watchdog stays enabled with the shortest timeout. Turn it off as early
// as possible, otherwise the Mega can reset over and over (Some old Mega bootloaders don't do this).
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((used)) __attribute__((section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

void setup() {
    initPins(); // All solenoids off first
    Serial.begin(115200);
    Serial.print(F("MinTCU boot, reset cause 0x"));
    Serial.println(mcusr_mirror, HEX);

    Solenoids::init_all_solenoids();

    // Same numbers as UN52 VEHICLE_CONFIG.egs_can_type
    if (1 == VEHICLE_CONFIG.egs_can_type) {
        Serial.println(F("CAN: EGS51"));
        egs_can_hal = new Egs51Can(toRaw(Pin::can_cs), toRaw(Pin::can_int));
    } else {
        Serial.println(F("CAN: EGS52"));
        egs_can_hal = new Egs52Can(toRaw(Pin::can_cs), toRaw(Pin::can_int));
    }
    if (!egs_can_hal->begin()) {
        // Keep going, every getter returns 'not available', so the gearbox stays in failsafe (0mA)
        Serial.println(F("CAN init FAILED (Check wiring, CS pin and CAN_CRYSTAL)"));
    }

    gearbox = new Gearbox(); // Also sets up the sensors (TCUIO::setup_io_layer)
    Serial.print(F("Calibration: "));
    Serial.println(CAL_DESCRIPTION);
    if (!EGSCal::init_egs_calibration()) {
        Serial.println(F("NO VALID CALIBRATION, staying in failsafe (Run tools/egs_cal_gen.py)"));
    }
    if (!Sensors::using_dedicated_output_rpm()) {
        Serial.println(F("No output speed sensor (Set VEHICLE_CONFIG.input_sensor_pulses_per_rev)"));
    }

    Serial.println(F("ms,rpm,n2,n3,input_rpm,output_rpm,ratio,pedal,trq_sta,trq_drv,trq_in,shifter,gear,atf,vbatt,mpc_targ,mpc_corr,mpc_ma,mpc_ma_est,duty_permille,spc_ma,shift_sol,garage,failsafe,can_ovf"));
    // If the loop ever hangs for 250ms, the Mega resets. Pins go high-Z in reset,
    // the gate pull-downs turn the solenoids off = max pressure
    wdt_enable(WDTO_250MS);
}

void loop() {
    wdt_reset();
    egs_can_hal->update();
    gearbox->controller_loop();
}
