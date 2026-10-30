#ifndef EEPROM_IMPL_H
#define EEPROM_IMPL_H

#include <stdint.h>
#include <stddef.h>
#include "stored_map.h"

/*
    UN52 stores maps in the ESP32 NVS by name (nvs/eeprom_config.cpp, EEPROM::read_nvs_map_data and
    EEPROM::write_nvs_map_data). MinTCU stores them in the Mega's internal EEPROM (4 KB) instead.

    Layout: a 4 byte header (magic, layout version), then one fixed slot per map name in
    EEPROM_MAP_SLOTS (eeprom_impl.cpp). A slot is the element count, the data and a CRC16. A slot with
    a wrong count or CRC (never written, layout changed, power lost while writing) is not used, the
    map then starts from its default (Same as UN52 when the map is not in NVS).

    A Mega EEPROM byte takes 3.4ms to write, so a write would block the gearbox for up to a second.
    write_nvs_map_data() only queues the map, EEPROM::poll() (called from loop()) writes one byte at
    a time when the EEPROM is ready, and only bytes that changed (Less wear, the EEPROM is good for
    about 100000 writes per byte).
*/
namespace EEPROM {
    esp_err_t read_nvs_map_data(const char* map_name, int16_t* dest, const int16_t* default_map, size_t map_element_count, bool default_in_flash = false);
    esp_err_t write_nvs_map_data(const char* map_name, const int16_t* to_write, size_t arr_size);
    /**
     * @brief Last profile of the EWM program button (Same as UN52, NVS key NVS_KEY_LAST_PROFILE).
     * Two bytes after the map slots (the profile and its inverse). Saving writes them straight away
     * (eeprom_update_byte, up to 7ms), UN52 only saves when the selector goes into P.
     */
    esp_err_t ewm_btn_get_saved_profile(uint8_t* dest);
    esp_err_t ewm_btn_save_profile(uint8_t save_profile);
    /**
     * @brief MinTCU fault memory (GearboxFault bits, not in UN52). Two bytes after the last profile
     * (the faults and their inverse), written straight away. Nothing saved reads as no faults.
     */
    uint8_t read_fault_memory(void);
    void write_fault_memory(uint8_t faults);
    /**
     * @brief Call from loop(), writes the queued maps in the background
     */
    void poll(void);
    /**
     * @brief True while a map is still being written
     */
    bool busy(void);
}

#endif
