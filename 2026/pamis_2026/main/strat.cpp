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



void strat_ninja(void* arg) {
        
    Position test[2] = {
        {.x=0, .y=500, .theta=0},
        {.x=500, .y=500, .theta=M_PI},
    };

    while(!sap_ping(1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    sts3032::move(7,3050);
    sts3032::move(1,1100);
    locomotion.vider_frigos();
    locomotion.moveBlocking(650,0);
    locomotion.moveBlocking(-290,0);
    locomotion.moveBlocking(0,-M_PI/2);
    locomotion.moveBlocking(25,0);
    locomotion.moveBlocking(-345,0);
    locomotion.sortir_caisse();
    locomotion.placer_frigo_1();
    locomotion.moveBlocking(240,0);
    locomotion.pousser_caisse();
    locomotion.sortir_caisse();
    locomotion.placer_frigo_2();
    locomotion.moveBlocking(135,0);
    locomotion.pousser_caisse();
    locomotion.sortir_caisse();
    locomotion.moveBlocking(50,0);
    locomotion.moveBlocking(0,M_PI/2);
    locomotion.moveBlocking(245,0);



    



    
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



    while(true) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    Position a = locomotion.getPos();
    printf("%f, %f, %f\n", a.x, a.y, a.theta);
    locomotion.trajectory(test, 2);
    vTaskDelay(500 / portTICK_PERIOD_MS);


    while(true) {
        auto pos = locomotion.getPos();
        char pos_str[30];
        snprintf(pos_str, sizeof(pos_str), "x=%d,y=%d,theta=%.2f",(int)pos.x,(int)pos.y,pos.theta);
        telelogs_send_string("Odom", pos_str);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}



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



    while(true) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

