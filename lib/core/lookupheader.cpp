#include <lookupheader.h>
#include <string.h>
#include <stdlib.h>

int16_t LookupHeader::get_value(const uint16_t index) const
{
    int16_t result = INT16_MAX;
    if(index <= size){
        result = header[index];
    }
    return result;
}

uint16_t LookupHeader::get_size(void) const
{
    return size;
}

int16_t * LookupHeader::get_data(void) const
{
    return header;
}

LookupRefHeader::LookupRefHeader(int16_t *_header, const uint16_t _size)
:LookupHeader()
{
    size = _size;
    header = _header;
}

// Same as UN52 (TCU_HEAP_ALLOC / TCU_FREE are malloc / free here)
LookupAllocHeader::LookupAllocHeader(const int16_t *_header, const uint16_t _size)
:LookupHeader()
{
    size = _size;
    header = static_cast<int16_t*>(malloc(_size * sizeof(int16_t)));
    allocation_successful = (nullptr != header);
    if(allocation_successful){
        (void)memcpy(header, _header, size*sizeof(int16_t));
    }
}

LookupAllocHeader::~LookupAllocHeader(void)
{
    if(allocation_successful){
        free(header);
    }
}
