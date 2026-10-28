#include "profiles.h"
#include <tcu_maths.h>
#include "clock.hpp"
#include "maps.h"
#include "nvs/eeprom_config.h"
#include <tcu_maths_impl.h>
static_assert(SHIFT_MAP_X_SIZE*SHIFT_MAP_Y_SIZE == SHIFT_MAP_SIZE, "");

// Same as UN52 profiles.cpp (int is int32_t here, int is 16 bit on the AVR)

// Same headers as UN52 profiles.h
static const int16_t shift_table_x_header[SHIFT_MAP_X_SIZE] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
static const int16_t upshift_y_headers[SHIFT_MAP_Y_SIZE] = {1,2,3,4};
static const int16_t downshift_y_headers[SHIFT_MAP_Y_SIZE] = {2,3,4,5};
static const int16_t shift_time_table_x_header[6] = {0, 20, 40, 60, 80, 100};
// UN52 makes this per profile (StoredMap copies it). LookupFlashMap keeps a pointer, so it is
// shared (Same values for every profile)
static int16_t shift_rpm_points[5] = {};

AbstractProfile::AbstractProfile(bool is_diesel,
        const int16_t* def_upshift_data_diesel,
        const int16_t* def_downshift_data_diesel,
        const int16_t* def_upshift_data_petrol,
        const int16_t* def_downshift_data_petrol,
        const int16_t* def_upshift_time_data,
        const int16_t* def_downshift_time_data
    ) {

    this->is_diesel = is_diesel;

    const int16_t* default_map;
    /** Upshift map **/
    if (is_diesel) {
        default_map = def_upshift_data_diesel;
    } else {
        default_map = def_upshift_data_petrol;
    }
    this->upshift_table = new LookupFlashMap(shift_table_x_header, SHIFT_MAP_X_SIZE, upshift_y_headers, SHIFT_MAP_Y_SIZE, default_map);

    /** Downshift map **/
    if (is_diesel) {
        default_map = def_downshift_data_diesel;
    } else {
        default_map = def_downshift_data_petrol;
    }
    this->downshift_table = new LookupFlashMap(shift_table_x_header, SHIFT_MAP_X_SIZE, downshift_y_headers, SHIFT_MAP_Y_SIZE, default_map);

    // Up/downshift time tables
    int16_t redline = is_diesel ? VEHICLE_CONFIG.red_line_rpm_diesel : VEHICLE_CONFIG.red_line_rpm_petrol;
    int16_t step_size = (redline-1000) / 4;
    shift_rpm_points[0] = (int16_t)1000;
    shift_rpm_points[1] = (int16_t)(1000+(step_size));
    shift_rpm_points[2] = (int16_t)(1000+(step_size*2));
    shift_rpm_points[3] = (int16_t)(1000+(step_size*3));
    shift_rpm_points[4] = redline;
    this->upshift_time_map = new LookupFlashMap(shift_time_table_x_header, 6, shift_rpm_points, 5, def_upshift_time_data);
    this->downshift_time_map = new LookupFlashMap(shift_time_table_x_header, 6, shift_rpm_points, 5, def_downshift_time_data);
}

ShiftCharacteristics AbstractProfile::get_shift_characteristics(GearChange requested, SensorData* sensors) {
    ShiftCharacteristics result = {
        .target_shift_time = 500,
        .extra_speed = this->extra_shift_dynamics()
    };
    switch (requested) {
        case GearChange::_1_2:
        case GearChange::_2_3:
        case GearChange::_3_4:
        case GearChange::_4_5:
            result.target_shift_time = this->get_upshift_time(sensors->input_rpm, ((float)sensors->pedal_pos*100.0)/250.0);
            break;
        case GearChange::_5_4:
        case GearChange::_4_3:
        case GearChange::_3_2:
        case GearChange::_2_1:
            result.target_shift_time = this->get_downshift_time(sensors->input_rpm, ((float)sensors->pedal_pos*100.0)/250.0);
            break;
        default:
            result.target_shift_time = 500;
            break;
    }

    return result;
}


AgilityProfile::AgilityProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        A_DIESEL_UPSHIFT_MAP,
        A_DIESEL_DOWNSHIFT_MAP,
        A_PETROL_UPSHIFT_MAP,
        A_PETROL_DOWNSHIFT_MAP,
        A_UPSHIFT_TIME_MAP,
        A_DOWNSHIFT_TIME_MAP
    ) {
}


bool AgilityProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) {
        return false;
    }
    if (this->upshift_table != nullptr) { // TEST TABLE
        // RPM where we will upshift based on the current load
        uint16_t upshift_map_val = this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
        // Add some extra RPM for catalyst warm up (+1000RPM at -10C, negated at 40C and higher)
        upshift_map_val += interpolate_float(sensors->atf_temp, 1000, 0, -10, 40, InterpType::Linear);
        int32_t time_since_last_shift = GET_CLOCK_TIME() - sensors->last_shift_time; // ms
        // Increase the threshold higher RPMs closer to shift
        // (Inverted since raw values are time SINCE shift)
        upshift_map_val += interpolate_float(time_since_last_shift, 0, 1500, 5000, 0, InterpType::Linear);
        int32_t mmax = sensors->max_torque;
        if (0 != mmax) {
            // Score of 0-1
            float engine_load_percent = (float)MAX(0, sensors->converted_driver_torque) / (float)sensors->max_torque;
            upshift_map_val += interpolate_float(engine_load_percent, 1000, 0, 0.8, 0.2, InterpType::Linear);
        }
        bool can_upshift = sensors->input_rpm > upshift_map_val;
        if (sensors->pedal_pos == 0 || sensors->pedal_delta_per_second < -5) {
            can_upshift = false;
        }
        if (sensors->brake_pressed) { can_upshift = false; }
        return can_upshift;
    } else {
        return false;
    }
}

bool AgilityProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) { return false; }
    if (this->downshift_table != nullptr) { // TEST TABLE
        int32_t adder = 0;
        if (sensors->pedal_delta_per_second >= 0) {
            adder = interpolate_float(sensors->pedal_delta_per_second, 0, 100, 1, 500, InterpType::Linear);
        } else {
            adder = interpolate_float(sensors->pedal_delta_per_second, -25, 0, -50, 0, InterpType::Linear);
        }
        int32_t pedal_percentage = (float)(sensors->pedal_pos)/2.5;
        pedal_percentage = MAX(0, MIN(pedal_percentage + adder, 100));
        return sensors->input_rpm < this->downshift_table->get_value(pedal_percentage, (float)current_gear);
    } else {
        return false;
    }
}

ComfortProfile::ComfortProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        C_DIESEL_UPSHIFT_MAP,
        C_DIESEL_DOWNSHIFT_MAP,
        C_PETROL_UPSHIFT_MAP,
        C_PETROL_DOWNSHIFT_MAP,
        C_UPSHIFT_TIME_MAP,
        C_DOWNSHIFT_TIME_MAP
    ) {
}

bool ComfortProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) {
        return false;
    }
    if (this->upshift_table != nullptr) { // TEST TABLE
        bool can_upshift = sensors->input_rpm > this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
        if (sensors->brake_pressed) { can_upshift = false; }
        //if (can_upshift) {
        //    if (sensors->max_torque != 0) {
        //        float demanded_load = (MAX(sensors->driver_requested_torque, 0) * 100) / sensors->max_torque;
        //        if (demanded_load > 30) {
        //            can_upshift = false;
        //        }
        //    }
        //}
        return can_upshift;
    } else {
        return false;
    }
}

bool ComfortProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    }
    if (this->downshift_table != nullptr) { // TEST TABLE
        return sensors->input_rpm < this->downshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
    } else {
        return false;
    }
}

WinterProfile::WinterProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        C_UPSHIFT_TIME_MAP,
        C_DOWNSHIFT_TIME_MAP
    ) {
}

bool WinterProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool WinterProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Second) {
        return false;
    }
    return manual->should_downshift(current_gear, sensors);
}


StandardProfile::StandardProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        S_DIESEL_UPSHIFT_MAP,
        S_DIESEL_DOWNSHIFT_MAP,
        S_PETROL_UPSHIFT_MAP,
        S_PETROL_DOWNSHIFT_MAP,
        S_UPSHIFT_TIME_MAP,
        S_DOWNSHIFT_TIME_MAP
    ) {
}

bool StandardProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::Fifth) { return false; }
    if (this->upshift_table != nullptr) { // TEST TABLE
        // RPM where we will upshift based on the current load
        uint16_t upshift_map_val = this->upshift_table->get_value(sensors->pedal_pos/2.5, (float)current_gear);
        // Add some extra RPM for catalyst warm up (+1000RPM at -10C, negated at 40C and higher)
        upshift_map_val += interpolate_float(sensors->atf_temp, 1000, 0, -10, 40, InterpType::Linear);
        int32_t time_since_last_shift = GET_CLOCK_TIME() - sensors->last_shift_time; // ms
        // Increase the threshold higher RPMs closer to shift
        // (Inverted since raw values are time SINCE shift)
        upshift_map_val += interpolate_float(time_since_last_shift, 0, 1000, 5000, 0, InterpType::Linear);
        int32_t mmax = sensors->max_torque;
        if (0 != mmax) {
            // Score of 0-1
            float engine_load_percent = (float)MAX(0, sensors->converted_driver_torque) / (float)sensors->max_torque;
            upshift_map_val += interpolate_float(engine_load_percent, 1000, 0, 0.8, 0.2, InterpType::Linear);
        }
        bool can_upshift = sensors->input_rpm > upshift_map_val;
        if (sensors->pedal_pos == 0 || sensors->pedal_delta_per_second < -5) {
            can_upshift = false;
        }
        if (sensors->brake_pressed) { can_upshift = false; }
        return can_upshift;
    } else {
        return false;
    }
}

void StandardProfile::update(SensorData* sensors) {
}

bool StandardProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) { return false; }
    if (this->downshift_table != nullptr) { // TEST TABLE
        int32_t adder = 0;
        if (sensors->pedal_delta_per_second >= 0) {
            adder = interpolate_float(sensors->pedal_delta_per_second, 0, 50, 1, 200, InterpType::Linear);
        } else {
            adder = interpolate_float(sensors->pedal_delta_per_second, -25, 0, -50, 0, InterpType::Linear);
        }
        int32_t pedal_percentage = (float)(sensors->pedal_pos)/2.5;
        pedal_percentage = MAX(0, MIN(pedal_percentage + adder, 100));
        return sensors->input_rpm < this->downshift_table->get_value(pedal_percentage, (float)current_gear);
    } else {
        return false;
    }
}

ManualProfile::ManualProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        M_UPSHIFT_TIME_MAP,
        M_DOWNSHIFT_TIME_MAP
    ) {
}

bool ManualProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return false;
}

bool ManualProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    if (current_gear == GearboxGear::First) {
        return false;
    } else if (sensors->input_rpm < 300 && sensors->pedal_pos == 0) {
        return true;
    } else {
        return false;
    }
}


RaceProfile::RaceProfile(bool is_diesel): AbstractProfile(
        is_diesel,
        M_DIESEL_UPSHIFT_MAP,
        M_DIESEL_DOWNSHIFT_MAP,
        M_PETROL_UPSHIFT_MAP,
        M_PETROL_DOWNSHIFT_MAP,
        R_UPSHIFT_TIME_MAP,
        R_DOWNSHIFT_TIME_MAP
    ) {
}

bool RaceProfile::should_upshift(GearboxGear current_gear, SensorData* sensors) {
    return manual->should_upshift(current_gear, sensors);
}

bool RaceProfile::should_downshift(GearboxGear current_gear, SensorData* sensors) {
    return manual->should_downshift(current_gear, sensors);
}

/* Now initialized in Gearbox() */
AgilityProfile* agility = nullptr;
ComfortProfile* comfort = nullptr;
WinterProfile* winter = nullptr;
ManualProfile* manual = nullptr;
StandardProfile* standard = nullptr;
RaceProfile* race = nullptr;

AbstractProfile *profiles[NUM_PROFILES] = {};

void Profiles::init_profiles(bool is_diesel) {
    standard = new StandardProfile(is_diesel);
    comfort = new ComfortProfile(is_diesel);
    winter = new WinterProfile(is_diesel);
    agility = new AgilityProfile(is_diesel);
    manual = new ManualProfile(is_diesel);
    race = new RaceProfile(is_diesel);

    profiles[(uint8_t)GearboxProfile::Standard] = standard;
    profiles[(uint8_t)GearboxProfile::Comfort] = comfort;
    profiles[(uint8_t)GearboxProfile::Winter] = winter;
    profiles[(uint8_t)GearboxProfile::Agility] = agility;
    profiles[(uint8_t)GearboxProfile::Manual] = manual;
    profiles[(uint8_t)GearboxProfile::Race] = race;
}
