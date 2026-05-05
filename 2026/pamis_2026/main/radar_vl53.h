#pragma once
#include "driver/i2c_master.h"
#define RADAR_NB 5

void radar_vl53_start(i2c_master_bus_handle_t* bus_handle);
uint16_t get_distance(uint16_t radar_nb);

