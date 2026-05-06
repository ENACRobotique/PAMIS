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
    int cote;
    if (gpio_get_level(FDC3)){
        cote=1;
    }
    else{
        cote=-1;
    }
    Position test[2] = {
        {.x=0, .y=500, .theta=0},
        {.x=500, .y=500, .theta=M_PI},
    };

    while(!sap_ping(1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    sts3032::move(7,3050);
    sts3032::move(1,1100);
    locomotion.set_seuils(0,200,0,0,0);
    locomotion.set_speed(3000,500);
    locomotion.vider_frigos(cote);
    locomotion.moveWallid(570,0,cote);
    locomotion.taper_mur(cote);
    locomotion.set_seuils(200,0,0,0,0);
    locomotion.moveWallid(-290,0,cote);
    locomotion.set_seuils(0,200,100,100,0);
    locomotion.moveWallid(0,-M_PI/2,cote);
    locomotion.taper_mur(cote);
    locomotion.set_seuils(200,0,0,0,0);
    locomotion.moveWallid(-345,0,cote);
    locomotion.set_seuils(0,200,100,100,0);
    locomotion.sortir_caisse(cote);
    locomotion.placer_frigo_1(cote);
    locomotion.moveWallid(230,0,cote);
    locomotion.pousser_caisse(cote);
    locomotion.sortir_caisse(cote);
    locomotion.placer_frigo_2(cote);
    locomotion.moveWallid(110,0,cote);
    locomotion.pousser_caisse(cote);
    locomotion.sortir_caisse(cote);
    locomotion.moveWallid(50,0,cote);
    locomotion.moveWallid(0,M_PI/2,cote);
    locomotion.moveWallid(245,0,cote);



    



    
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

