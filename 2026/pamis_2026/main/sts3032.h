#pragma once
#include "inttypes.h"

namespace sts3032{
    int move(uint8_t id, uint16_t pos);
    int setSpeed(uint8_t id, uint16_t speed);
    int16_t readPos(uint8_t id);
    uint8_t moving(uint8_t id);
}
