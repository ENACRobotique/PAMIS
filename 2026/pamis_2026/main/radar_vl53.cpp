#include "radar_vl53.h"
#include "vl53l1_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
extern "C" {
    #include "VL53L1X_api.h"
}
#include "endian.h"
#include "telelogs.h"

#define RADAR_NB 5

#define RADAR_NB_RETRY 5


static i2c_master_bus_handle_t* vl53_bus_handle;

static gpio_config_t radar_sht_conf = {
    .pin_bit_mask = (1 << GPIO_NUM_14) | (1 << GPIO_NUM_21) | (1 << GPIO_NUM_11) | (1 << GPIO_NUM_12) | (1 << GPIO_NUM_13),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

static VL53L1_Dev_t radars[RADAR_NB] = {
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
        .actif = true
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_14,
        .addr = 0x33,
        .actif = true
    },
    {
        .dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x29,
            .scl_speed_hz = 400000
        },
        .shutdown_gpio = GPIO_NUM_21,
        .addr = 0x34,
        .actif = true
    }
};

static VL53L1X_ERROR init_radar(VL53L1_Dev_t* radar) {
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*vl53_bus_handle, &radar->dev_config, &radar->dev_handle));
        // turn sensor ON
        gpio_set_level(radar->shutdown_gpio,1);
        
        // Wait for device booted
        uint16_t nb_errors = 0;
        while(true){
            uint8_t booted = 0;
            VL53L1X_ERROR status = VL53L1X_BootState(radar, (uint8_t*)&booted);
            if(status == 0 && booted){
                break;
            }
            
            if(status) {
                if(nb_errors++ > RADAR_NB_RETRY) {
                    // too many I2C errors, disable this radar and abort init
                    // TODO raise an error somewhere ?
                    radar->actif = false;
                    return VL53L1X_ERROR_TIMEOUT;
                }
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        //sensor initialization
        VL53L1X_SensorInit(radar);

        // change I2C address
        VL53L1X_SetI2CAddress(radar, radar->addr*2);
        radar->dev_config.device_address = radar->addr;
        i2c_master_bus_rm_device(radar->dev_handle);
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*vl53_bus_handle, &radar->dev_config, &radar->dev_handle));

        // if needed, for quicker measurements
        // VL53L1X_SetDistanceMode(radars, 1);
        // VL53L1X_SetTimingBudgetInMs(radars, 20);
        // VL53L1X_SetInterMeasurementInMs(radars, 20);

        // start continuous ranging
        VL53L1X_StartRanging(radar);
        
        return VL53L1X_ERROR_NONE;
}

static VL53L1X_ERROR radar_read(VL53L1_Dev_t* radar,  VL53L1X_Result_t* result)
{
    // wait until data ready
    while (true)
    {
        uint8_t isDataReady = 0;
        VL53L1X_ERROR status = VL53L1X_CheckForDataReady(radar, &isDataReady);
        if (status == VL53L1X_ERROR_NONE)
        {
            if (isDataReady)
            {
                break;
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        else
        {
            return status;
        }
    }

    VL53L1X_GetResult(radar, result);
    VL53L1X_ClearInterrupt(radar);

    return VL53L1X_ERROR_NONE;
}

static void radarTask(void* arg) {
    // configure GPIO dor shutdown pins
    gpio_config(&radar_sht_conf);

    // shutdown all sensors
    for(int i = 0; i<RADAR_NB; i++){
        gpio_set_level(radars[i].shutdown_gpio,0);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);    // be sure that sensor are off
    
    // init sensors
    for(int i = 0; i<RADAR_NB; i++){
        if(radars[i].actif){
            init_radar(&radars[i]);
        }
    }

    while(true) {
        for(int i = 0; i<RADAR_NB; i++){
            if(radars[i].actif){
                VL53L1X_Result_t result;
                VL53L1X_ERROR status = radar_read(&radars[i], &result);
                // check that read succeded and the measurement is valid
                if (status == VL53L1X_ERROR_NONE && result.Status == 0)
                {
                    char name[50];
                    snprintf(name, 50, "dist %d", i);
                    telelogs_send_float(name, result.Distance);

                    // TODO: make this distance accessible

                    // char Ambient[50];
                    // snprintf(Ambient, 50, "Ambient %d", i);
                    // telelogs_send_float(Ambient, result.Ambient);

                    // char SigPerSPAD[50];
                    // snprintf(SigPerSPAD, 50, "SigPerSPAD %d", i);
                    // telelogs_send_float(SigPerSPAD, result.SigPerSPAD);

                    // char NumSPADs[50];
                    // snprintf(NumSPADs, 50, "NumSPADs %d", i);
                    // telelogs_send_float(NumSPADs, result.NumSPADs);
                }
                else {
                    // sensor not responding or bad measurement
                    // TODO: lets consider there are no obstacles ?
                }

                vPortYield();
            }
        }
        
    }
}



void radar_vl53_start(i2c_master_bus_handle_t* bus_handle){
    vl53_bus_handle = bus_handle;
    xTaskCreate( radarTask, "radar", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
}
