#pragma once
#include "driver/i2c_master.h"

typedef struct
{
    float accel_x; 
    float accel_y;
    float accel_z;
    float giro_x;
    float giro_y;
    float giro_z;
}imu_t;

void imu_init(i2c_master_bus_handle_t* bus_handle);

esp_err_t register_read(uint8_t reg, uint8_t* buff, size_t length);