#ifndef PINLAYOUT_H
#define PINLAYOUT_H

#include <HardwareSerial.h>
#include <Arduino.h>

/*constexpr uint32_t MOT1_STEP = PA1;
constexpr uint32_t MOT1_DIR = PB0;
constexpr uint32_t MOT2_STEP = PA0;
constexpr uint32_t MOT2_DIR = PB1;*/

constexpr uint32_t R_MEAS_PIN = PA4;

constexpr uint32_t SERVO1 = PA8;
constexpr uint32_t SERVO2 = PA11;

constexpr uint32_t PRESSURE_CLK = PA10;
constexpr uint32_t PRESSURE_HAT = PA5;

constexpr uint32_t DISPLAY_CLK = PB5;
constexpr uint32_t DISPLAY_DIO = PB4;



#define UART_TX PA2
#define UART_RX PA3



#endif /* PINLAYOUT_H */
