/*
    Nu skaja faktist fösök få na ådentli byrjan
    Ja ska ha MPC ti arbeit från kalibrering å sensore
    Allt ska lånas från UN52 så langt he ba gar, fö he funkkar

    Now I'm actually gonna try to get a decent start
    I need MPC to work from calibration and sensors
    Everything must be borrowed from UN52 as far as possible, because that works
*/

#include <Arduino.h>
#include <stdint.h>
#include "gearbox.h"

#include "shifter/shifter.h"
#include "shifter/shifter_ewm.h"


#include <AVR_PWM.h>
#undef B1

/*
    My arduino pinout saved here so I don't forget what's what

// SHIFT PINS
const int shift23 = 22;
const int shift34 = 23;
const int shift12_45 = 24;

// PWM PINS
const int mpc = 11;
const int spc = 12;
const int tcc = 13;
*/

// v Arduino testing v

uint8_t MPC_PWM_PIN = 11;
float MPC_PWM_FREQ = 1000.0f;
float TARGET_DUTY  = 50.0f;

AVR_PWM* mpc_pwm = nullptr;


void setup() {
    shifter = new ShifterEwm(&VEHICLE_CONFIG, &ETS_CURRENT_SETTINGS);
    gearbox = new Gearbox(shifter);
}

void loop() {
    if (gearbox != nullptr) {
         gearbox->controller_loop();
    }
}





