#include "radar_vl53.h"
#include "vl53l1_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
extern "C" {
    #include "VL53L1X_api.h"
}
#include "endian.h"
#define RADAR_NB 5
i2c_master_bus_handle_t* vl53_bus_handle;

    gpio_config_t radar_sht_conf = {
        .pin_bit_mask = (1 << GPIO_NUM_14) | (1 << GPIO_NUM_21) | (1 << GPIO_NUM_11) | (1 << GPIO_NUM_12) | (1 << GPIO_NUM_13),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    

VL53L1_Dev_t radars[RADAR_NB] = {
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_11,
        .addr = 0x30,
        .actif = true
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_12,
        .addr = 0x31,
        .actif = true
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_13,
        .addr = 0x32,
        .actif = false
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_14,
        .addr = 0x33,
        .actif = false
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_21,
        .addr = 0x34,
        .actif = false
    }
};


static void read_radar(void* arg) {
    gpio_config(&radar_sht_conf);

    for(int i = 0; i<RADAR_NB; i++){
        gpio_set_level(radars[i].shutdown_gpio,0);

    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    for(int i = 0; i<RADAR_NB; i++){

        if(!radars[i].actif){continue;}
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*vl53_bus_handle, &radars[i].dev_config, &radars[i].dev_handle));
        gpio_set_level(radars[i].shutdown_gpio,1);
        uint8_t booted = 0;
        /* Wait for device booted */
        while(true){
            VL53L1X_ERROR status = VL53L1X_BootState(&radars[i],&booted);
            if(booted){
                break;
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        VL53L1X_SensorInit(&radars[i]); //sensor initialization
        //VL53L1X_SetInterMeasurementPeriod();

        VL53L1X_SetI2CAddress(&radars[i], radars[i].addr*2);
        radars[i].dev_config.device_address = radars[i].addr;
        i2c_master_bus_rm_device(radars[i].dev_handle);
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*vl53_bus_handle, &radars[i].dev_config, &radars[i].dev_handle));
        VL53L1X_StartRanging(&radars[i]);
    }

    
    uint8_t isDataReady = 0;
    uint8_t rangeStatus = 0;
    uint16_t distance = 0;
    uint16_t bdistance = 0;
    while(true) {
        for(int i = 0; i<RADAR_NB; i++){
            if(!radars[i].actif){continue;}
            while(!isDataReady){
                VL53L1X_CheckForDataReady(&radars[i],&isDataReady);
            }

            isDataReady = 0;
            VL53L1X_GetRangeStatus(&radars[i], &rangeStatus);
            VL53L1X_GetDistance(&radars[i], &distance);
            bdistance = __bswap16(distance);
            VL53L1X_ClearInterrupt(&radars[i]);
            printf("distance %d : %u\n", i, bdistance);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        
    }

}


void radar_vl53_start(i2c_master_bus_handle_t* bus_handle){
    vl53_bus_handle = bus_handle;
    xTaskCreate( read_radar, "Read_radar", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
}
