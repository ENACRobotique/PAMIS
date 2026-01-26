#pragma once
#include "driver/gpio.h"

constexpr gpio_num_t LED1 = GPIO_NUM_5;
constexpr gpio_num_t LED2 = GPIO_NUM_6;

// Odometry period in milliseconds
constexpr uint32_t ODOMETRY_PERIOD_MS = 20;

