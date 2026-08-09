#include <Arduino.h>
#include <stdint.h>
#include <common.h>
#include <tcu_maths.h>
#include <tcu_maths_impl.h>

// lookupheader.h
class LookupHeader {
    public:
        int16_t get_value(const uint16_t index) const;
        uint16_t get_size(void) const;
        int16_t* get_data(void) const;  
    protected:
        int16_t* header;
        uint16_t size;

};

class LookupRefHeader: public LookupHeader {
    public:
        /// @brief manages a table header for a lookup table or lookup map
        /// @param _header 
        /// @param _length 
        LookupRefHeader(int16_t* _header, const uint16_t _size);
};

// lookupheader.cpp
LookupRefHeader::LookupRefHeader(int16_t *_header, const uint16_t _size)
:LookupHeader()
{
    size = _size;
    header = _header;
}

// lookuptable.h
class LookupTable {
    public:
        float get_value(float xValue);
        /// @brief This functions generates a corresponding header-value based on the parameter. This function does only work on tables with increasing x-values.
        /// @param xValue the value to be looked up
        /// @return the interpolated header-value
        float get_header_interpolated(const float value) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        int16_t* get_current_data(void);
        const LookupHeader* get_header(void);
        uint16_t data_size(void) const;
    protected:
        uint16_t xHeaderSize;
        uint16_t dataSize;
        int16_t* data;
        LookupHeader* xHeader;
};

class LookupRefTable: public LookupTable {
    public:
        LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize);
};

// lookuptable.cpp
LookupRefTable::LookupRefTable(int16_t* _xHeader, uint16_t _xHeaderSize, int16_t* _data, uint16_t _dataSize)
:LookupTable() {
    this->xHeader = new LookupRefHeader(_xHeader, _xHeaderSize);
    this->data = _data;
    this->dataSize = _dataSize;
}

// lookupmap.h
class LookupMap {
    public:
        float get_value(const float xValue, const float yValue);
        void get_y_headers(uint16_t *size, int16_t **headers);
        float get_x_header_interpolated(const float value, const int16_t y) const;
        int16_t* get_current_data(void) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        uint16_t data_size();
    protected:
        LookupTable* table;
        LookupHeader* yHeader;
        uint16_t yHeaderSize;
};

class LookupRefMap : public LookupMap {
    public:
        LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize);
};

// lookupmap.cpp
float LookupMap::get_value(const float xValue, const float yValue)
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();

    // part 1a - identification of the indices for x-value
    search_value<int16_t>(xValue, xHeader->get_data(), xHeader->get_size(), &x_idx_min, &x_idx_max);
    
    // part 1b - identification of the indices for y-value
    search_value<int16_t>(yValue, yHeader->get_data(), yHeader->get_size(), &y_idx_min, &y_idx_max);
    
    // part 2: do the interpolation
    const int16_t x1 = xHeader->get_value(x_idx_min);
    const int16_t x2 = xHeader->get_value(x_idx_max);
    const int16_t y1 = yHeader->get_value(y_idx_min);
    const int16_t y2 = yHeader->get_value(y_idx_max);

    // some precalculations for making the code more readable, although somewhat inefficient
    const float f_11 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_min];
    const float f_12 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_max];
    const float f_21 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_min];
    const float f_22 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_max];

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, xValue);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, xValue);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, yValue);
}

LookupRefMap::LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupRefTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->yHeader = new LookupRefHeader(_yHeader, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}