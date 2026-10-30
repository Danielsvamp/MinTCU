#ifndef TORQUE_CONVERTER_H__
#define TORQUE_CONVERTER_H__

#include <stdint.h>
#include "can_defines.h"
#include "common_structs.h"
#include "nvs/eeprom_config.h"
#include <string.h>
#include "pressure_manager.h"
#include "nvs/module_settings.h"
#include "stored_map.h"

/*
    Ported from UN52 torque_converter.h/.cpp. Changes:
    - int is 16 bit on the Mega2560, so every int is int32_t here (Same as the shift algorithms)
    - The adaptation maps are saved to the Mega's EEPROM (see stored_map.h and nvs/eeprom_impl.h)
*/

class AbstractProfile;

enum class InternalTccState {
    Open = 0,
    Slipping = 1,
};


class TorqueConverter {
    public:
        TorqueConverter(uint16_t max_gb_rating);

        /**
         * @brief Lets the torque converter code poll and see what is next to do with the converters
         * clutch
         *
         * @param curr_gear The current gear the transmission is in
         * @param max_lockup The maximum allowed lockup type for the torque converter
         * @param sensors Sensor data used as input
         * @param shifting True if the car is currently transitioning to new gear
         */
        void update(GearboxGear curr_gear, GearboxGear targ_gear, PressureManager* pm, AbstractProfile* profile, SensorData* sensors);
        TccClutchStatus get_clutch_state(void);
        void save() {
            if (this->tcc_adapt_map_d1) {
                this->tcc_adapt_map_d1->save_to_eeprom();
            }
            if (this->tcc_adapt_map_d2) {
                this->tcc_adapt_map_d2->save_to_eeprom();
            }
            if (this->tcc_adapt_map_d3) {
                this->tcc_adapt_map_d3->save_to_eeprom();
            }
            if (this->tcc_adapt_map_d4) {
                this->tcc_adapt_map_d4->save_to_eeprom();
            }
            if (this->tcc_adapt_map_d5) {
                this->tcc_adapt_map_d5->save_to_eeprom();
            }

        };

        void diag_toggle_tcc_sol(bool en);

        void set_stationary();

        void shift_start(bool upshift, bool release_shifting);
        void shift_end();
        uint16_t get_slip_now();
        InternalTccState __get_internal_state(void);
        uint8_t get_current_state();
        uint8_t get_target_state();
        uint16_t get_command_pressure();
        uint16_t get_map_pressure();
        int16_t get_pid_pressure();

        uint8_t get_adapt_timer();
        uint8_t get_pid_timer();
        
        uint16_t get_slip_targ() {
            return this->slip_target;
        }

        inline StoredMap* get_adapt_map_d1() {
            return this->tcc_adapt_map_d1;
        }

        inline StoredMap* get_adapt_map_d2() {
            return this->tcc_adapt_map_d2;
        }

        inline StoredMap* get_adapt_map_d3() {
            return this->tcc_adapt_map_d3;
        }

        inline StoredMap* get_adapt_map_d4() {
            return this->tcc_adapt_map_d4;
        }

        inline StoredMap* get_adapt_map_d5() {
            return this->tcc_adapt_map_d5;
        }

        inline StoredMap* get_rpm_slip_map() {
            return this->slip_rpm_target_map;
        }

        inline uint32_t get_engine_power() {
            return 0;
        }

        inline int16_t get_engine_load_percent() {
            return this->engine_load_percent;
        }

        inline uint32_t get_absorbed_power() {
            return this->timer_till_adapt;
        }

    private:
        uint16_t calculate_slip_target(SensorData* sensors);
        void calculate_min_pressure(SensorData* sensors, GearboxGear current_g);
        void process_open_or_slip_state(SensorData* sd, GearboxGear current_g);
        void calculate_torque_correction(SensorData* sensors);
        bool check_if_pulling(SensorData* sensors);

        int32_t rated_max_torque;
        bool pulling = false;
        bool is_shifting = false;
        bool was_shifting = true;
        bool upshifting = false;
        bool release_shifting = false;
        bool tcc_solenoid_enabled = true;


        int32_t tcc_commanded_pressure = 0;
        int32_t tcc_shift_pressure = 0;

        uint32_t prefill_start_time = 0;
        InternalTccState current_tcc_state = InternalTccState::Open;
        InternalTccState target_tcc_state = InternalTccState::Open;
        StoredMap* slip_rpm_target_map;
        bool pending_changes = false;
        int16_t engine_load_percent = 0;

        bool init_tables_ok = false;



        StoredMap* tcc_adapt_map_d1 = nullptr;
        StoredMap* tcc_adapt_map_d2 = nullptr;
        StoredMap* tcc_adapt_map_d3 = nullptr;
        StoredMap* tcc_adapt_map_d4 = nullptr;
        StoredMap* tcc_adapt_map_d5 = nullptr;

        bool was_stationary = true;
        uint16_t slip_target = 100;
        uint8_t profile_id = UINT8_MAX; // MinTCU, for coast_lock_profiles

        uint8_t command_p_stage = 0;
        uint8_t timer_command_p = 0;


        int32_t min_tcc_pressure = 0;
        // x100
        int32_t filtered_engine_trq = 0;
        // x100
        int32_t filtered_pump_trq = 0;
        // x100
        int32_t torque_correction_adapt = 0;
        // x100
        int32_t filtered_engine_rpm = 0;
        // x100
        int32_t filtered_input_rpm = 0;
        // x100
        int32_t old_actual_slip_abs = 0;
        // x100
        int32_t actual_slip_abs = 0;

        uint16_t input_side = 0;
        uint16_t tcc_engine_trq = 0;

        uint8_t timer_inc_slip = 0;
        uint8_t timer_till_adapt = 0;
        uint8_t timer_till_pid = 0;
        int32_t targ_slip_pid = 0;
        int32_t pid_pressure = 0;
        int32_t map_pressure = 0;

        inline StoredMap* get_tcc_adapt_map(GearboxGear g) {
            StoredMap* ptr = nullptr;
            if (GearboxGear::First == g) {
                ptr = this->tcc_adapt_map_d1;
            } else if (GearboxGear::Second == g) {
                ptr = this->tcc_adapt_map_d2;
            } else if (GearboxGear::Third == g) {
                ptr = this->tcc_adapt_map_d3;
            } else if (GearboxGear::Fourth == g) {
                ptr = this->tcc_adapt_map_d4;
            } else if (GearboxGear::Fifth == g) {
                ptr = this->tcc_adapt_map_d5;
            }
            return ptr;
        }

        inline int32_t get_tcc_adapt_map_pressure(GearboxGear g, SensorData* sd) {
            StoredMap* map = this->get_tcc_adapt_map(g);
            int32_t ret = 0;
            if (nullptr != map) {
                ret = map->get_value(this->engine_load_percent, sd->atf_temp);
            }
            return ret;
        }

        void fill_tcc(GearboxGear g, SensorData* sd);

        int32_t pid_p_weight = 0;
        int32_t pid_i_weight = 0;

        int32_t pid_i_val = 0;
        int32_t pid_i_val_old = 0;
};

#endif
