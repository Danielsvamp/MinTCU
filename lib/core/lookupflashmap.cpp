#include <lookupflashmap.h>
#include <tcu_maths_impl.h>

static int16_t read_flash_value(const int16_t* data, const uint16_t idx) {
#ifdef __AVR__
    return (int16_t)pgm_read_word(&data[idx]);
#else
    return data[idx];
#endif
}

LookupFlashMap::LookupFlashMap(const int16_t* _xHeader, const uint8_t _xHeaderSize, const int16_t* _yHeader, const uint8_t _yHeaderSize, const int16_t* _flashData) {
    this->xHeader = _xHeader;
    this->xHeaderSize = _xHeaderSize;
    this->yHeader = _yHeader;
    this->yHeaderSize = _yHeaderSize;
    this->data = _flashData;
}

// Same as LookupMap::get_value (Without the lookup cache)
float LookupFlashMap::get_value(const float xValue, const float yValue) const
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;

    // part 1a - identification of the indices for x-value
    search_value<int16_t>(xValue, this->xHeader, this->xHeaderSize, &x_idx_min, &x_idx_max);

    // part 1b - identification of the indices for y-value
    search_value<int16_t>(yValue, this->yHeader, this->yHeaderSize, &y_idx_min, &y_idx_max);

    // part 2: do the interpolation
    const int16_t x1 = this->xHeader[x_idx_min];
    const int16_t x2 = this->xHeader[x_idx_max];
    const int16_t y1 = this->yHeader[y_idx_min];
    const int16_t y2 = this->yHeader[y_idx_max];

    const float f_11 = (float)read_flash_value(this->data, (y_idx_min * this->xHeaderSize) + x_idx_min);
    const float f_12 = (float)read_flash_value(this->data, (y_idx_min * this->xHeaderSize) + x_idx_max);
    const float f_21 = (float)read_flash_value(this->data, (y_idx_max * this->xHeaderSize) + x_idx_min);
    const float f_22 = (float)read_flash_value(this->data, (y_idx_max * this->xHeaderSize) + x_idx_max);

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, xValue);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, xValue);
    return interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, yValue);
}
