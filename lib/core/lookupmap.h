#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include <lookuptable.h>


struct LookupCache {
    // IEEE754 float
    float x_val;
    float y_val;
    uint32_t timestamp_ms;
} __attribute__((packed));

// UN52 has 5. The cache is only read by the UN52 diagnostic app (copy_lookup_cache), which MinTCU
// does not have, and every map has its own copy (60 bytes of RAM each with 5). MinTCU keeps 1.
const int MAX_LOOKUP_CACHE = 1;

class LookupMap {
    public:
        float get_value(const float xValue, const float yValue);
        float get_value(const float xValue, const float yValue, const uint8_t lookup_cache_idx);
        bool add_value(const int16_t sample_point_value, const int16_t x_value, const int16_t y_value, const float threshold);
        void get_y_headers(uint16_t *size, int16_t **headers);
        float get_x_header_interpolated(const float value, const int16_t y) const;
        int16_t* get_current_data(void) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        uint16_t data_size();
        void copy_lookup_cache(LookupCache* dest) const;
    protected:
        LookupTable* table;
        LookupHeader* yHeader;
        uint16_t yHeaderSize;
        LookupCache lookup_cache[MAX_LOOKUP_CACHE] = {
            {0,0,0}
        };
};

class LookupAllocMap : public LookupMap {
    public:
        LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        bool add_data(const int16_t* map, const uint16_t size);
        bool is_allocated(void) const;
        ~LookupAllocMap();
};

class LookupRefMap : public LookupMap {
    public:
        LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize);
};

class LookupByteMap : public LookupMap {
    public:
        LookupByteMap(uint8_t* _xHeader, const uint16_t _xHeaderSize, uint8_t* _yHeader, const uint16_t _yHeaderSize, uint8_t* _data, const uint16_t _dataSize);
        bool is_allocated(void) const;
        bool add_data(const uint8_t* map, const uint16_t size);
        ~LookupByteMap();
    private:
        int16_t* x_alloc;
        int16_t* y_alloc;
        int16_t* z_alloc;
        uint16_t z_size;
};

#endif /* lookupmap.h */
