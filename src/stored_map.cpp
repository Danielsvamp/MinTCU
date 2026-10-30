#include "stored_map.h"
#include "nvs/eeprom_impl.h"

StoredMap::StoredMap(const char *eeprom_key_name,
                           const uint16_t data_element_count,
                           const int16_t *x_headers,
                           const int16_t *y_headers,
                           const uint16_t x_size,
                           const uint16_t y_size,
                           const int16_t *default_map,
                           const bool default_in_flash) : LookupAllocMap(x_headers,
                                                                x_size,
                                                                y_headers,
                                                                y_size,
                                                                default_map,
                                                                data_element_count,
                                                                default_in_flash)
{
    // Same checks as UN52 (Size matches, and the map could be allocated)
    if ((x_size * y_size) == data_element_count && this->is_allocated()) {
        // UN52 reads into a new buffer and copies it in, MinTCU reads straight into the map (less RAM)
        this->init_state = EEPROM::read_nvs_map_data(eeprom_key_name, this->get_current_data(), default_map, data_element_count, default_in_flash);
        if (ESP_OK == this->init_state) {
            this->data_element_count = data_element_count;
            this->data_name = eeprom_key_name;
        }
    }
}

esp_err_t StoredMap::save_to_eeprom(void)
{
    return EEPROM::write_nvs_map_data(this->data_name, this->get_current_data(), this->data_element_count);
}
