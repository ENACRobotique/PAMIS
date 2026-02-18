#include "radar_vl53.h"
#include "vl53l1_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern "C" {
    #include "VL53L1X_api.h"
}
#include "endian.h"


VL53L1_Dev_t radar1 = {
    .dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x29,
        .scl_speed_hz = 400000
    }
};

static void read_radar(void* arg) {
    uint8_t booted;
    /* Wait for device booted */
    while(true){
        VL53L1X_ERROR status = VL53L1X_BootState(&radar1,&booted);
        if(booted){
            break;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    VL53L1X_SensorInit(&radar1); //sensor initialization
    //VL53L1X_SetInterMeasurementPeriod();
    VL53L1X_StartRanging(&radar1);

    uint8_t isDataReady = 0;
    uint8_t rangeStatus = 0;
    uint16_t distance = 0;
    uint16_t bdistance = 0;
    while(true) {
        while(!isDataReady){
            VL53L1X_CheckForDataReady(&radar1,&isDataReady);

        }
        isDataReady = 0;
        VL53L1X_GetRangeStatus(&radar1, &rangeStatus);
        VL53L1X_GetDistance(&radar1, &distance);
        bdistance = __bswap16(distance);
        VL53L1X_ClearInterrupt(&radar1);
        printf("distance :%u\n", bdistance);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
    }

}


void radar_vl53_start(i2c_master_bus_handle_t* bus_handle){
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &radar1.dev_config, &radar1.dev_handle));
    xTaskCreate( read_radar, "Read_radar", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
}
