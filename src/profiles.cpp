#include "profiles.h"
#include "maps.h"
#include "nvs/eeprom_config.h"

// Same as UN52 profiles.cpp, for the Manual profile

AbstractProfile::AbstractProfile(bool is_diesel,
        const int16_t* def_upshift_time_data,
        const int16_t* def_downshift_time_data
    ) {
    this->is_diesel = is_diesel;

    // Up/downshift time tables
    int16_t redline = is_diesel ? VEHICLE_CONFIG.red_line_rpm_diesel : VEHICLE_CONFIG.red_line_rpm_petrol;
    int16_t step_size = (redline-1000) / 4;
    int16_t shift_rpm_points[5] = {(int16_t)1000,  (int16_t)(1000+(step_size)), (int16_t)(1000+(step_size*2)), (int16_t)(1000+(step_size*3)), redline};
    this->upshift_time_map = new LookupAllocMap(shift_time_table_x_header, 6, shift_rpm_points, 5, def_upshift_time_data, SHIFT_TIME_MAP_SIZE);
    if (!this->upshift_time_map->is_allocated()) {
        delete this->upshift_time_map;
        this->upshift_time_map = nullptr;
    }
    this->downshift_time_map = new LookupAllocMap(shift_time_table_x_header, 6, shift_rpm_points, 5, def_downshift_time_data, SHIFT_TIME_MAP_SIZE);
    if (!this->downshift_time_map->is_allocated()) {
        delete this->downshift_time_map;
        this->downshift_time_map = nullptr;
    }
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

ManualProfile::ManualProfile(bool is_diesel) : AbstractProfile(
        is_diesel,
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

ManualProfile* manual = nullptr;

void Profiles::init_profiles(bool is_diesel) {
    manual = new ManualProfile(is_diesel);
}
