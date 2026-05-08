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
#include "Astar.h"
#include "evitement.h"
#include "math.h"

#define YELLOW 0


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

void strat_pami2026(void* ID)
{

    Position pos_depart;
    Position pos_arrive;
    uint32_t PAMI_ID = (uint32_t) ID;


    if (YELLOW)
    {
        switch (PAMI_ID)
        {
        case 1:
        {
            pos_depart = {50, 1600, M_PI};
            pos_arrive = {100, 950, -M_PI / 2.0};
            break;
        }
        case 2:
        {
            pos_depart = {50, 1700, 0};
            pos_arrive = {700, 250, 0};
            break;
        }
        case 3:
        {
            pos_depart = {50, 1800, 0};
            pos_arrive = {800, 950, 0};
            break;
        }
        case 4:
        {
            pos_depart = {50, 1900, 0};
            pos_arrive = {1250, 1300, 0};
            break;
        }
        case 5:
        {
            pos_depart = {300, 1900, 0};
            pos_arrive = {1350, 800, 0};
            break;
        }
        default:
            break;
        }
    }
    else
    {
        switch (PAMI_ID)
        {
        case 1:
        {
            pos_depart = {2900, 1600, -M_PI};
            pos_arrive = {2900, 950, -M_PI / 2.0};
            break;
        }
        case 2:
        {
            pos_depart = {2950, 1700, -M_PI};
            pos_arrive = {2300, 250, -M_PI / 2.0};
            break;
        }
        case 3:
        {
            pos_depart = {2950, 1800, -M_PI};
            pos_arrive = {2200, 950, -M_PI / 2.0};
            break;
        }
        case 4:
        {
            pos_depart = {2950, 1900, 0};
            pos_arrive = {1750, 1300, 0};
            break;
        }
        case 5:
        {
            pos_depart = {2700, 1900, 0};
            pos_arrive = {1650, 800, 0};
            break;
        }
        default:
            break;
        }
    }

    locomotion.setPos(pos_depart);
    // wait to plug tirette
    // while (gpio_get_level(FDC1))
    // {
    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }

    locomotion.enableSteppers(true);

    // wait to unplug tirette
    // while (!gpio_get_level(FDC1))
    // {
    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }

    // wait 85 secondes (pour etre sur de pas partir avant)
    // vTaskDelay(pdMS_TO_TICKS(85000));

    Position chemin_suivi[200];
    printf("on a demarrer la strat simple \n");

    // on calcule la traj ici
    int nb_points = calcul_chemin(pos_depart, pos_arrive, chemin_suivi);
    printf("nombre de point trouvé %i \n", nb_points);

    if (nb_points > 0)
    {
        locomotion.trajectory(chemin_suivi, nb_points);
    }

    while (true)
    {
        // faire tourner l'acctionneur en boucle
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void strat_marchepas(void *arg){
    while (true){

        locomotion.moveBlocking(0, -5 * 2 * M_PI);
       vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
