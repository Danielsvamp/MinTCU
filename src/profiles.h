// Profile code

#ifndef PROFILES_H
#define PROFILES_H

#include "common_structs.h"
#include <lookupmap.h>

/*
    The part of UN52's profiles.h the shift algorithms use. MinTCU only has the Manual profile
    (+/- on the selector). UN52's shift time maps are StoredMaps (NVS), here they always hold
    UN52's default maps.
*/

const int16_t shift_time_table_x_header[6] = {0, 20, 40, 60, 80, 100};

class AbstractProfile {
public:
    AbstractProfile(bool is_diesel,
        const int16_t* def_upshift_time_data,
        const int16_t* def_downshift_time_data
    );
    virtual ~AbstractProfile() = default;
    ShiftCharacteristics get_shift_characteristics(GearChange requested, SensorData* sensors);
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors) = 0;
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors) = 0;

    virtual bool extra_shift_dynamics(void) {
        return false;
    }

    uint16_t get_upshift_time(uint16_t input_rpm, float pedal_percent) {
        uint16_t result = 750u;
        if (nullptr != this->upshift_time_map) {
            result = (uint16_t)(this->upshift_time_map->get_value(pedal_percent, (float)input_rpm));
        }
        return result;
    }

    uint16_t get_downshift_time(uint16_t input_rpm, float pedal_percent) {
        uint16_t result = 750u;
        if (nullptr != this->downshift_time_map) {
            result = (uint16_t)(this->downshift_time_map->get_value(pedal_percent, (float)input_rpm));
        }
        return result;
    }

protected:
    bool is_diesel;
    LookupAllocMap* upshift_time_map = nullptr;
    LookupAllocMap* downshift_time_map = nullptr;
};

class ManualProfile : public AbstractProfile {
public:
    explicit ManualProfile(bool is_diesel);
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    bool extra_shift_dynamics(void) override {
        return true;
    }
};

extern ManualProfile* manual;

namespace Profiles {
    void init_profiles(bool is_diesel);
}

#endif
