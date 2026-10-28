#include "stored_map.h"

StoredMap::StoredMap(const char *eeprom_key_name,
                           const uint16_t data_element_count,
                           const int16_t *x_headers,
                           const int16_t *y_headers,
                           const uint16_t x_size,
                           const uint16_t y_size,
                           const int16_t *default_map) : LookupAllocMap(x_headers,
                                                                x_size,
                                                                y_headers,
                                                                y_size,
                                                                default_map,
                                                                data_element_count)
{
    // Same checks as UN52 (Size matches, and the map could be allocated)
    if ((x_size * y_size) == data_element_count && this->is_allocated()) {
        this->init_state = ESP_OK;
    }
}
