#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"

constexpr gpio_num_t LED1 = GPIO_NUM_5;
constexpr gpio_num_t LED2 = GPIO_NUM_6;

constexpr gpio_num_t FDC1 = GPIO_NUM_10;
constexpr gpio_num_t FDC2 = GPIO_NUM_9;
constexpr gpio_num_t FDC3 = GPIO_NUM_3;

// Odometry period in milliseconds
constexpr uint32_t ODOMETRY_PERIOD_MS = 20;

constexpr gpio_num_t SCL = GPIO_NUM_47;
constexpr gpio_num_t SDA = GPIO_NUM_48;

constexpr uart_port_t SAP_UART = UART_NUM_1;

constexpr gpio_num_t TX1 = GPIO_NUM_17;
constexpr gpio_num_t RX1 = GPIO_NUM_18;
constexpr gpio_num_t RTS = GPIO_NUM_16;