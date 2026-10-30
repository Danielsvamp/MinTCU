#ifndef STORED_MAP_H
#define STORED_MAP_H

#include <stdint.h>
#include <lookupmap.h>

#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

/*
    UN52 StoredMap: a map that is loaded from (and saved to) the ESP32 NVS, starting from the
    firmware default map. Same here, but saved to the Mega's EEPROM (nvs/eeprom_impl.h).
    Only the maps listed in EEPROM_MAP_SLOTS (eeprom_impl.cpp) are saved, a map with a name that is
    not listed (or nullptr) always starts from its default and save_to_eeprom() fails.
    Same names as UN52.
*/
class StoredMap : public LookupAllocMap {
    public:
        StoredMap(
            const char* eeprom_key_name,
            const uint16_t data_element_count,
            const int16_t* x_headers,
            const int16_t* y_headers,
            const uint16_t x_size,
            const uint16_t y_size,
            const int16_t* default_map,
            const bool default_in_flash = false // MinTCU: default_map is in flash (TCU_FLASH)
        );
        esp_err_t init_status() const { return this->init_state; }
        /**
         * @brief Save new map contents to EEPROM (This will mean next TCU load will use the new map)
         * MinTCU: queued, EEPROM::poll() writes it in the background
         */
        esp_err_t save_to_eeprom(void);
    private:
        esp_err_t init_state = ESP_FAIL;
        const char* data_name = nullptr;
        uint16_t data_element_count = 0;
};

#endif
