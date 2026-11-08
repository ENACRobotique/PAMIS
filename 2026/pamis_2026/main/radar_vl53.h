#pragma once

#include <cstddef>
#include <cstdint>
#include "driver/i2c_master.h"

namespace radar_vl53 {

constexpr std::size_t RADAR_NB = 5;

void start(i2c_master_bus_handle_t* bus_handle);
uint16_t get_distance(uint16_t radar_nb);

} // namespace radar_vl53
