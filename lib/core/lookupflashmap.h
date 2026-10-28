#ifndef LOOKUPFLASHMAP_H
#define LOOKUPFLASHMAP_H

#include <stdint.h>

/*
    MinTCU only (Not in UN52). A read-only map whose data stays in flash (TCU_FLASH), the
    headers are in RAM. get_value() is the same lookup and interpolation as LookupMap::get_value().

    UN52 copies every default map into RAM (StoredMap). The Mega has 8KB of RAM, so maps that are
    never changed (the profile shift maps) are read from flash instead.
*/
class LookupFlashMap {
    public:
        LookupFlashMap(const int16_t* _xHeader, const uint8_t _xHeaderSize, const int16_t* _yHeader, const uint8_t _yHeaderSize, const int16_t* _flashData);
        float get_value(const float xValue, const float yValue) const;
    private:
        const int16_t* xHeader;
        const int16_t* yHeader;
        const int16_t* data; // In flash
        uint8_t xHeaderSize;
        uint8_t yHeaderSize;
};

#endif /* lookupflashmap.h */
