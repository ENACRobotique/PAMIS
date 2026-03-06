#pragma once
#include "inttypes.h"

namespace sts3032{
    void move(uint8_t id, uint16_t pos);
    int16_t readPos(uint8_t id);
}
