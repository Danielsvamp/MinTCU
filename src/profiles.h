// Profile code

#ifndef PROFILES_H
#define PROFILES_H

#include "common_structs.h"
#include <lookupflashmap.h>

/*
    Same as UN52 profiles.h, with these differences:
    - UN52's maps are StoredMaps (NVS, editable in the config app). MinTCU has no config app, and
      would need about 3KB of RAM to copy every profile's maps, so the maps are read straight from
      flash (LookupFlashMap) and are always UN52's default maps. The NVS key names are left out.
    - get_display_gear() and the tag_id are left out (MinTCU sends nothing on CAN, so there is no
      cluster display)
*/

const uint8_t NUM_PROFILES = 6u; // S, C, W, A, M, R
const uint8_t PROFILE_IDX_S = 0u;
const uint8_t PROFILE_IDX_C = 1u;
const uint8_t PROFILE_IDX_W = 2u;
const uint8_t PROFILE_IDX_A = 3u;
const uint8_t PROFILE_IDX_M = 4u;
const uint8_t PROFILE_IDX_R = 5u;

const uint8_t SHIFT_MAP_X_SIZE = 11u;
const uint8_t SHIFT_MAP_Y_SIZE = 4u;

enum class GearboxProfile : uint8_t{
    // Standard (S)
    Standard = 0u,
    // Comfort (C)
    Comfort = 1u,
    // Winter (W)
    Winter = 2u,
    // Agility (A)
    Agility = 3u,
    // Manual (M)
    Manual = 4u,
    // Race (R)
    Race = 5u,
    Individual = 6u,
    // Reserved for 'no profile' (_)
    Underscore = 7u,
    // Failure (F)
    Failure = 8u
};

/**
 * A profile is designed to read the current conditions of the gearbox and request the gearbox to do something
 */
class AbstractProfile {
public:
    static const uint8_t MAX_PROFILES = 4u;

    AbstractProfile(
        bool is_diesel,
        const int16_t* def_upshift_data_diesel,
        const int16_t* def_downshift_data_diesel,
        const int16_t* def_upshift_data_petrol,
        const int16_t* def_downshift_data_petrol,
        const int16_t* def_upshift_time_data,
        const int16_t* def_downshift_time_data
    );
    virtual void update(SensorData* sensors) {};
    virtual GearboxProfile get_profile(void) const = 0;
    virtual bool should_upshift(GearboxGear current_gear, SensorData* sensors) = 0;
    virtual bool should_downshift(GearboxGear current_gear, SensorData* sensors) = 0;
    ShiftCharacteristics get_shift_characteristics(GearChange requested, SensorData* sensors);
    virtual bool extra_shift_dynamics(void) {
        return false;
    }

    LookupFlashMap* get_upshift_map(void) {
        return this->upshift_table;
    }
    LookupFlashMap* get_downshift_map(void) {
        return this->downshift_table;
    }
    LookupFlashMap* get_upshift_time_map(void) {
        return this->upshift_time_map;
    }
    LookupFlashMap* get_downshift_time_map(void) {
        return this->downshift_time_map;
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

    virtual GearboxGear get_start_gear(void) const {
        return GearboxGear::First;
    }
    uint8_t get_profile_id(void) { return (uint8_t)get_profile(); }
protected:
    LookupFlashMap* upshift_table = nullptr;
    LookupFlashMap* downshift_table = nullptr;
    LookupFlashMap* upshift_time_map = nullptr;
    LookupFlashMap* downshift_time_map = nullptr;
private:
    bool is_diesel;
};

class AgilityProfile : public AbstractProfile {
public:
    explicit AgilityProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Agility; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    bool extra_shift_dynamics(void) override {
        return true;
    }
};

class ComfortProfile : public AbstractProfile {
public:
    explicit ComfortProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Comfort; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    GearboxGear get_start_gear(void) const override {
        return GearboxGear::Second;
    }
};

class WinterProfile : public AbstractProfile {
public:
    explicit WinterProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Winter; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    GearboxGear get_start_gear(void) const override {
        return GearboxGear::Second;
    }
};

class StandardProfile : public AbstractProfile {
public:
    explicit StandardProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Standard; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    void update(SensorData *sd) override;
};

class ManualProfile : public AbstractProfile {
public:
    explicit ManualProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Manual; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
};

class RaceProfile : public AbstractProfile {
public:
    explicit RaceProfile(bool is_diesel);
    GearboxProfile get_profile(void) const override { return GearboxProfile::Race; }
    bool should_upshift(GearboxGear current_gear, SensorData* sensors) override;
    bool should_downshift(GearboxGear current_gear, SensorData* sensors) override;
    bool extra_shift_dynamics(void) override {
        return true;
    }
};

extern AgilityProfile* agility;
extern ComfortProfile* comfort;
extern WinterProfile* winter;
extern ManualProfile* manual;
extern StandardProfile* standard;
extern RaceProfile* race;

extern AbstractProfile *profiles[NUM_PROFILES];

namespace Profiles {
    void init_profiles(bool is_diesel);
}

#endif
