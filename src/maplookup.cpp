#include <Arduino.h>
#include <stdint.h>
#include <common.h>

// tcu_maths.h
float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x);

// tcu_maths.cpp
float interpolate(const float f_1, const float f_2, const int16_t x_1, const int16_t x_2, const float x)
{
    // cast values from signed integer values to floating values in order to avoid casting the same value twice
    const float x_1_f = (float)x_1;
    const float x_2_f = (float)x_2;
    // See https://en.wikipedia.org/wiki/Linear_interpolation for details. Return f_1, if x_1 and x_2 are identical.
    return (x_1 != x_2) ? f_1 + ((f_2 - f_1) / (x_2_f - x_1_f)) * (x - x_1_f) : f_1;
}

//tcu_maths_impl.h
template <typename T> void search_value(const T value, const T *values, const uint16_t size, uint16_t *idvalue_min, uint16_t *idvalue_max)
{
	// Set minimum index to the first element of the field.
    *idvalue_min = 0u;
    // Set maximum index to the last element of the field.
    *idvalue_max = size - 1u;
    // Check, if search value is smaller than smallest element of the field.
    if (value > values[0]) {
        if (value < values[*idvalue_max]) {
            // Search value is in between the limits of the smallest and the biggest element of the field.
            do {
                // Calculate the middle of the remaining list. If the size is odd, it is rounded down.
                uint16_t idvalue_mid = (*idvalue_min + *idvalue_max) >> 1;
                if (value < values[idvalue_mid]) {
                    // Search value is smaller than the element in the middle of the remaining list.
                    *idvalue_max = idvalue_mid;
                }
                else if (value > values[idvalue_mid]) {
                    // Search value is bigger than the element in the middle of the remaining list.
                    *idvalue_min = idvalue_mid;
                }
                else {
                    // Search value is also an element of the field.
                    *idvalue_min = idvalue_mid;
                    *idvalue_max = idvalue_mid;
                }
                // Reduce the remaining search area until it is narrowed down to two consecutive elements.
            } while (1u < ((*idvalue_max) - (*idvalue_min)));
        }
        else {
            // Search value is as big as or bigger then the biggest element in the field.
            *idvalue_min = *idvalue_max;
        }
    }
    else {
        // Search value is as small as or smaller than smallest element of the field.
        *idvalue_max = *idvalue_min;
    }
}

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