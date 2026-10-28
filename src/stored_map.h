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
    UN52 StoredMap is a map that is loaded from (and saved to) the ESP32 NVS, starting from the
    firmware default map.

    MinTCU has nowhere to save it yet (The Mega's EEPROM is not used), so this StoredMap always
    starts from the default map, and changes (TCC adaptation) only last until the Mega is reset.
    save_to_eeprom() does nothing. Same names as UN52.
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
            const int16_t* default_map
        );
        esp_err_t init_status() const { return this->init_state; }
        esp_err_t save_to_eeprom(void) { return ESP_OK; }
    private:
        esp_err_t init_state = ESP_FAIL;
};

#endif
