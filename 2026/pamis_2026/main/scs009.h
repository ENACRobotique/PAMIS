#pragma once
#include "inttypes.h"

namespace scs009
{
    void move_scs(uint8_t id, uint16_t pos);
    int16_t readPos_scs(uint8_t id);
}
