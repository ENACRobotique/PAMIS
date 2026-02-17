#pragma once
#include "driver/gpio.h"

constexpr gpio_num_t LED1 = GPIO_NUM_5;
constexpr gpio_num_t LED2 = GPIO_NUM_6;

constexpr gpio_num_t FDC1 = GPIO_NUM_10;
constexpr gpio_num_t FDC2 = GPIO_NUM_9;
constexpr gpio_num_t FDC3 = GPIO_NUM_3;

// Odometry period in milliseconds
constexpr uint32_t ODOMETRY_PERIOD_MS = 20;

constexpr gpio_num_t SCL = GPIO_NUM_47;
constexpr gpio_num_t SDA = GPIO_NUM_48;