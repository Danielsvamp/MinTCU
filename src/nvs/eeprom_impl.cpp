#include "eeprom_impl.h"
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <string.h>

// MinTCU EEPROM layout. Change EEPROM_LAYOUT_VERSION when the slots change, old data is then ignored
static const uint8_t EEPROM_MAGIC_0 = 'M';
static const uint8_t EEPROM_MAGIC_1 = 'T';
static const uint8_t EEPROM_LAYOUT_VERSION = 1;
static const uint16_t EEPROM_HEADER_SIZE = 4;

struct MapSlot {
    const char* name; // UN52 NVS key name (nvs/all_keys.h)
    uint16_t element_count;
};

// The maps that are saved. Slot address = header + the slots before it
static const MapSlot EEPROM_MAP_SLOTS[] = {
    {"TCC_ADAPT_1_1", 6*6},
    {"TCC_ADAPT_1_2", 6*6},
    {"TCC_ADAPT_1_3", 6*6},
    {"TCC_ADAPT_1_4", 6*6},
    {"TCC_ADAPT_1_5", 6*6},
};
static const uint8_t EEPROM_SLOT_COUNT = sizeof(EEPROM_MAP_SLOTS) / sizeof(EEPROM_MAP_SLOTS[0]);
static const uint16_t MAX_SLOT_ELEMENTS = 6*6;

// Slot: count (2 bytes), data, crc (2 bytes)
static uint16_t slot_size(uint8_t idx) {
    return 2 + (EEPROM_MAP_SLOTS[idx].element_count * sizeof(int16_t)) + 2;
}

static uint16_t slot_address(uint8_t idx) {
    uint16_t addr = EEPROM_HEADER_SIZE;
    for (uint8_t i = 0; i < idx; i++) {
        addr += slot_size(i);
    }
    return addr;
}

static int8_t find_slot(const char* map_name) {
    if (nullptr == map_name) {
        return -1;
    }
    for (uint8_t i = 0; i < EEPROM_SLOT_COUNT; i++) {
        if (0 == strcmp(map_name, EEPROM_MAP_SLOTS[i].name)) {
            return i;
        }
    }
    return -1;
}

static uint16_t crc_of(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = _crc16_update(crc, data[i]);
    }
    return crc;
}

static bool header_ok() {
    return eeprom_read_byte((uint8_t*)0) == EEPROM_MAGIC_0 &&
        eeprom_read_byte((uint8_t*)1) == EEPROM_MAGIC_1 &&
        eeprom_read_byte((uint8_t*)2) == EEPROM_LAYOUT_VERSION;
}

esp_err_t EEPROM::read_nvs_map_data(const char* map_name, int16_t* dest, const int16_t* default_map, size_t map_element_count) {
    if (nullptr == map_name || nullptr == dest || 0 == map_element_count) {
        return ESP_FAIL;
    }
    size_t byte_count = map_element_count*sizeof(int16_t);
    int8_t slot = find_slot(map_name);
    bool ok = false;
    if (slot >= 0 && header_ok() && EEPROM_MAP_SLOTS[slot].element_count == map_element_count) {
        uint16_t addr = slot_address(slot);
        uint16_t count = eeprom_read_word((uint16_t*)addr);
        if (count == map_element_count) {
            eeprom_read_block(dest, (const void*)(addr + 2), byte_count);
            uint16_t crc = eeprom_read_word((uint16_t*)(addr + 2 + byte_count));
            ok = crc == crc_of((const uint8_t*)dest, byte_count);
        }
    }
    if (!ok) {
        // Not saved yet (or not valid): use the default map (Same as UN52)
        if (nullptr == default_map) {
            return ESP_FAIL;
        }
        memcpy(dest, default_map, byte_count);
    }
    return ESP_OK;
}

// Background writer state
static uint8_t pending_slots = 0; // Bit per slot
static const int16_t* pending_src[8] = {nullptr};
static int8_t writing_slot = -1;
static uint16_t write_pos = 0;
static uint16_t write_len = 0;
static uint16_t write_addr = 0;
// The slot being written, copied when the slot starts (so a map that changes while it is written
// still gets a matching CRC)
static uint8_t write_buf[2 + MAX_SLOT_ELEMENTS*sizeof(int16_t) + 2];

esp_err_t EEPROM::write_nvs_map_data(const char* map_name, const int16_t* to_write, size_t arr_size) {
    int8_t slot = find_slot(map_name);
    if (slot < 0 || nullptr == to_write || EEPROM_MAP_SLOTS[slot].element_count != arr_size) {
        return ESP_FAIL;
    }
    pending_src[slot] = to_write;
    pending_slots |= (1 << slot);
    return ESP_OK;
}

bool EEPROM::busy(void) {
    return 0 != pending_slots || writing_slot >= 0;
}

void EEPROM::poll(void) {
    if (!eeprom_is_ready()) {
        return;
    }
    if (writing_slot < 0) {
        if (0 == pending_slots) {
            return;
        }
        // Header first (Only written once, eeprom_update_byte skips bytes that are the same)
        if (!header_ok()) {
            eeprom_update_byte((uint8_t*)0, EEPROM_MAGIC_0);
            eeprom_update_byte((uint8_t*)1, EEPROM_MAGIC_1);
            eeprom_update_byte((uint8_t*)2, EEPROM_LAYOUT_VERSION);
            eeprom_update_byte((uint8_t*)3, 0);
        }
        for (uint8_t i = 0; i < EEPROM_SLOT_COUNT; i++) {
            if (pending_slots & (1 << i)) {
                pending_slots &= ~(1 << i);
                uint16_t count = EEPROM_MAP_SLOTS[i].element_count;
                uint16_t bytes = count * sizeof(int16_t);
                memcpy(&write_buf[0], &count, 2);
                memcpy(&write_buf[2], pending_src[i], bytes);
                uint16_t crc = crc_of(&write_buf[2], bytes);
                memcpy(&write_buf[2 + bytes], &crc, 2);
                writing_slot = i;
                write_pos = 0;
                write_len = slot_size(i);
                write_addr = slot_address(i);
                break;
            }
        }
        return;
    }
    // One byte per call. Only bytes that changed are written (eeprom_is_ready() is checked above,
    // so this does not wait)
    while (write_pos < write_len) {
        uint8_t* addr = (uint8_t*)(write_addr + write_pos);
        uint8_t b = write_buf[write_pos];
        write_pos++;
        if (eeprom_read_byte(addr) != b) {
            eeprom_write_byte(addr, b);
            return;
        }
    }
    writing_slot = -1;
}

static uint16_t last_profile_address() {
    return slot_address(EEPROM_SLOT_COUNT);
}

esp_err_t EEPROM::ewm_btn_get_saved_profile(uint8_t* dest) {
    uint16_t addr = last_profile_address();
    uint8_t v = eeprom_read_byte((const uint8_t*)addr);
    uint8_t inv = eeprom_read_byte((const uint8_t*)(addr + 1));
    if (nullptr == dest || (uint8_t)~v != inv) {
        // Never saved (Erased EEPROM is 0xFF 0xFF)
        return ESP_FAIL;
    }
    *dest = v;
    return ESP_OK;
}

esp_err_t EEPROM::ewm_btn_save_profile(uint8_t save_profile) {
    uint16_t addr = last_profile_address();
    eeprom_update_byte((uint8_t*)addr, save_profile);
    eeprom_update_byte((uint8_t*)(addr + 1), (uint8_t)~save_profile);
    return ESP_OK;
}

uint8_t EEPROM::read_fault_memory(void) {
    uint16_t addr = last_profile_address() + 2;
    uint8_t v = eeprom_read_byte((const uint8_t*)addr);
    uint8_t inv = eeprom_read_byte((const uint8_t*)(addr + 1));
    return ((uint8_t)~v == inv) ? v : 0;
}

void EEPROM::write_fault_memory(uint8_t faults) {
    uint16_t addr = last_profile_address() + 2;
    eeprom_update_byte((uint8_t*)addr, faults);
    eeprom_update_byte((uint8_t*)(addr + 1), (uint8_t)~faults);
}
