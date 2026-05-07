#include "strat.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "telelogs.h"
#include "locomotion.h"
#include "config.h"
#include "imu.h"
#include "radar_vl53.h"
#include "sts3032.h"
#include "SAP_controller.h"
#include "math.h"





void strat_fondation(void* arg) {
        
    // wait to plug tirette
    while (gpio_get_level(FDC1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    locomotion.enableSteppers(true);

    // wait to unplug tirette
    while (!gpio_get_level(FDC1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }


    
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI/2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI/2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI/2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI/2);


    while(true) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

