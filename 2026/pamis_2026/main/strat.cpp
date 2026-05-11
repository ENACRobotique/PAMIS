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

void strat_fondation(void *arg)
{

    // wait to plug tirette
    while (gpio_get_level(FDC1))
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    locomotion.enableSteppers(true);

    // wait to unplug tirette
    while (!gpio_get_level(FDC1))
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI / 2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI / 2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI / 2);
    locomotion.moveBlocking(100, 0);
    locomotion.moveBlocking(0, M_PI / 2);

    while (true)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void strat_pami2026(void *ID)
{

    Position pos_depart;
    Position pos_arrive;
    uint32_t PAMI_ID = (uint32_t)ID;
    Position chemin_complet[200];
    int nb_points_inter = 0;
    Position mes_waypoints[10];

    printf("niveau FCD2 %d \n", gpio_get_level(FDC2));
    locomotion.set_speed(500, 4000);

    if (gpio_get_level(FDC2) == 1)
    {
        printf("Je suis jaune \n");
        switch (PAMI_ID)
        {
        case 1:
        {
            pos_depart = {100, 1600, M_PI};
            pos_arrive = {100, 950, -M_PI / 2.0};
            // mes_waypoints[0];
            // nb_points_inter = 0;
            break;
        }
        case 2:
        {
            pos_depart = {100, 1700, M_PI};
            pos_arrive = {700, 250, -M_PI / 2.0};
            // mes_waypoints[0];
            // nb_points_inter = 0;
            break;
        }
        case 3:
        {
            pos_depart = {100, 1800, M_PI};
            pos_arrive = {800, 950, -M_PI / 2.0};
            mes_waypoints[0] = {800, 1100, -M_PI / 2.0};
            nb_points_inter = 1;
            break;
        }
        case 4:
        {
            pos_depart = {100, 1900, M_PI};
            pos_arrive = {1150, 1400, M_PI};
            mes_waypoints[0] = {500, 1400, M_PI};
            nb_points_inter = 1;
            break;
        }
        case 5:
        {
            pos_depart = {300, 1900, -M_PI / 2.0};
            pos_arrive = {1350, 800, M_PI};
            mes_waypoints[0] = {500, 1400, 0};
            nb_points_inter = 1;
            break;
        }
        default:
            break;
        }
    }
    else
    {
        printf("Je suis bleu\n");
        switch (PAMI_ID)
        {
        case 1:
        {
            pos_depart = {2900, 1600, -M_PI};
            pos_arrive = {2850, 950, -M_PI / 2.0};
            // mes_waypoints[0];
            // nb_points_inter = 0;
            break;
        }
        case 2:
        {
            pos_depart = {2900, 1700, -M_PI};
            pos_arrive = {2300, 250, -M_PI / 2.0};
            // mes_waypoints[0];
            // nb_points_inter = 0;
            break;
        }
        case 3:
        {
            pos_depart = {2900, 1800, -M_PI};
            pos_arrive = {2200, 900, -M_PI / 2.0};
            mes_waypoints[0] = {2600, 1800, -M_PI / 2.0};
            mes_waypoints[1] = {2600, 1500, -M_PI / 2.0};
            nb_points_inter = 2;

            break;
        }
        case 4:
        {
            pos_depart = {2900, 1900, -M_PI};
            pos_arrive = {1800, 1475, -M_PI};
            mes_waypoints[0] = {2500, 1475, -M_PI};
            nb_points_inter = 1;
            break;
        }
        case 5:
        {
            pos_depart = {2700, 1900, 0};
            pos_arrive = {1650, 800, 0};
            mes_waypoints[0] = {2500, 1400, 0};
            nb_points_inter = 1;
            break;
        }
        default:
            break;
        }
    }

    int nb_points = gestion_point_intermediaire(pos_depart, pos_arrive, mes_waypoints, nb_points_inter, chemin_complet);
    printf("nombre de point trouvé %i \n", nb_points);

    // on regarde si on doit attendre
    // if(gpio_get_level(FDC2)){
    // wait to plug tirette
    while (gpio_get_level(FDC1))
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    locomotion.enableSteppers(true);
    // wait to unplug tirette
    while (!gpio_get_level(FDC1))
    {
        printf("on m'a enlever la tirette \n");
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    if (nb_points > 0)
    {
        locomotion.trajectory(chemin_complet, nb_points);
    }

    locomotion.setPos(pos_depart);

    // wait 85 secondes (pour etre sur de pas partir avant)
    // vTaskDelay(pdMS_TO_TICKS(85000));
    // }

    while (true)
    {
        // faire tourner l'acctionneur en boucle
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void strat_marchepas(void *arg)
{
    while (true)
    {
        locomotion.set_speed(500, 400);
        locomotion.moveBlocking(0, -5 * 2 * M_PI);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
