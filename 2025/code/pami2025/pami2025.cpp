#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"


// VL53L1x example:
// https://github.com/alex-mous/VL53L1X-C-API-for-Raspberry-Pi-Pico/blob/main/examples/continuous_measurement/continuous.c


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA 5
#define I2C_SCL 4
#define LED1 2
#define LED2 3
#ifndef DELAY
#define DELAY 250
#endif


#define I2C_DEV_ADDR 0x29


// int64_t alarm_callback(alarm_id_t id, void *user_data) {
//     // Put your timeout handler code in here
//     return 0;
// }


// // UART defines
// // By default the stdout UART is `uart0`, so we will use the second one
// #define UART_ID uart1
// #define BAUD_RATE 115200

// // Use pins 4 and 5 for UART1
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define UART_TX_PIN 4
// #define UART_RX_PIN 5


    void led_init(){
        gpio_init(LED1);
        gpio_set_dir(LED1,GPIO_OUT);
        gpio_init(LED2);
        gpio_set_dir(LED2,GPIO_OUT);

        // #elif defined(CYW43_WL_GPIO_LED_PIN)
        // return cyw43_arch_init();
        // #endif
    }

    void led_light(bool i){
        gpio_put(LED1,i);
        gpio_put(LED2,!i);
        // #if defined(CYW43_WL_GPIO_LED_PIN)
        // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, i);
        // #endif
    }

int main()
{
    stdio_init_all();

    VL53L1X_Status_t status;
    VL53L1X_Result_t results;

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // I2C Initialisation. Using it at 400Khz.
    // i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // Timer example code - This example fires off the callback after 2000ms
    // add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("robot", "farmingmars", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    // // Set up our UART
    // uart_init(UART_ID, BAUD_RATE);
    // // Set the TX and RX pins by using the function select on the GPIO
    // // Set datasheet for more information on function select
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    
    // Send out a string, with CR/LF conversions
    // uart_puts(UART_ID, " Hello, UART!\n");
    
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

    led_init();


    if (VL53L1X_I2C_Init(I2C_DEV_ADDR, I2C_PORT) < 0) {
        printf("Error initializing sensor.\n");
        return 0;
    }

    // Ensure the sensor has booted
    uint8_t sensorState;
    do {
    status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
    } while (sensorState == 0);
    printf("Sensor booted.\n");

    // Initialize and configure sensor
    status = VL53L1X_SensorInit(I2C_DEV_ADDR);
    status += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 1);
    status += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
    status += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
    status += VL53L1X_StartRanging(I2C_DEV_ADDR);


    bool first_range = true;
    while (true) {
        // led_light(true);
        // sleep_ms(DELAY);
        // led_light(false);
        // sleep_ms(DELAY);

        // Wait until we have new data
        uint8_t dataReady;
        do {
            status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
            sleep_us(1);
        } while (dataReady == 0);

        // Read and display result
        status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
        printf("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
            results.status, results.distance, results.ambient, results.sigPerSPAD, results.numSPADs);

        // Clear the sensor for a new measurement
        status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
        if (first_range) {  // Clear twice on first measurement
            status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
            first_range = false;
        }
    }
}
