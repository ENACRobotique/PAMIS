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
#include "scs009.h"

extern uint8_t id_servo_queue;

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
    uint32_t strat_id = (uint32_t)ID;
    static Position chemin_complet[200];
    int nb_points_inter = 0;
    Position mes_waypoints[10];
    int attente_longue = 85000;

    printf("niveau FCD2 %d \n", gpio_get_level(FDC2));
    locomotion.set_speed(500, 2000);

    if (gpio_get_level(FDC2) == 0)
    {
        printf("Je suis jaune \n");
        switch (strat_id)
        {
        case 1:
        {
            pos_depart = {100, 1600, 0};
            pos_arrive = {700, 200, -M_PI / 2.0};
            mes_waypoints[0] = {400, 1500, -M_PI / 2.0};
            mes_waypoints[1] = {950, 1000, -M_PI / 2.0};
            mes_waypoints[2] = {950, 500, -M_PI / 2.0};
            nb_points_inter = 3;
            break;
        }
        case 2:
        {
            pos_depart = {100, 1700, 0};
            pos_arrive = {1500, 1000, -M_PI / 2.0};
            mes_waypoints[0] = {300, 1700, -M_PI / 2.0};
            mes_waypoints[1] = {300, 1500, -M_PI / 2.0};
            nb_points_inter = 2;
            attente_longue = 85500;
            break;
        }
        case 3:
        {
            pos_depart = {100, 1800, 0};
            pos_arrive = {800, 950, -M_PI / 2.0};
            mes_waypoints[0] = {300, 1800, -M_PI / 2.0};
            mes_waypoints[1] = {300, 1500, -M_PI / 2.0};
            nb_points_inter = 2;
            nb_points_inter = 2;
            attente_longue = 86000;
            break;
        }
        case 4:
        {
            pos_depart = {100, 1900, 0};
            pos_arrive = {1150, 1475, 0};
            mes_waypoints[0] = {300, 1900, M_PI};
            mes_waypoints[1] = {400, 1475, M_PI};
            nb_points_inter = 2;
            attente_longue = 86500;
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
        switch (strat_id)
        {
        case 1:
        {
            pos_depart = {2900, 1600, -M_PI};
            pos_arrive = {2300, 200, -M_PI / 2.0};
            mes_waypoints[0] = {2600, 1500, -M_PI / 2.0};
            mes_waypoints[1] = {2050, 1000, -M_PI / 2.0};
            mes_waypoints[2] = {2050, 500, -M_PI / 2.0};
            nb_points_inter = 3;
            break;
        }
        case 2:
        {
            pos_depart = {2900, 1700, -M_PI};
            pos_arrive = {1500, 900, -M_PI / 2.0};
            mes_waypoints[0] = {2700, 1700, -M_PI / 2.0};
            mes_waypoints[1] = {2700, 1500, -M_PI / 2.0};
            nb_points_inter = 2;
            attente_longue = 85500;
            break;
        }
        case 3:
        {
            pos_depart = {2900, 1800, -M_PI};
            pos_arrive = {2200, 900, -M_PI / 2.0};
            mes_waypoints[0] = {2700, 1800, -M_PI / 2.0};
            mes_waypoints[1] = {2700, 1500, -M_PI / 2.0};
            nb_points_inter = 2;
            attente_longue = 86000;
            break;
        }
        case 4:
        {
            pos_depart = {2900, 1900, -M_PI};
            pos_arrive = {1800, 1475, -M_PI};
            mes_waypoints[0] = {2700, 1900, -M_PI};
            mes_waypoints[1] = {2600, 1475, -M_PI};
            nb_points_inter = 2;
            attente_longue = 86500;
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

    locomotion.setPos(pos_depart);

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
    locomotion.setMatchStart();

    if (gpio_get_level(FDC3) == 1)
    {
        printf("j'attends longtemps \n");
        vTaskDelay(pdMS_TO_TICKS(attente_longue));
    }
    else
    {
        printf("j'attends pas longtemps \n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (nb_points > 0)
    {
        locomotion.trajectory(chemin_complet, nb_points);
    }

    bool a_gauche = true;
    const uint16_t POS_GAUCHE = 200; // Centre (512) - 112
    const uint16_t POS_DROITE = 500; // Centre (512) + 112

    while (locomotion.trajectoire_en_cours)
    {
        printf("je suis blouqer ici \n");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Bouger la queue
    while (1)
    {
        printf("je suis arrive pour le faire bouger \n");
        if (a_gauche)
        {
            scs009::move_scs(id_servo_queue, POS_GAUCHE);
        }
        else
        {
            scs009::move_scs(id_servo_queue, POS_DROITE);
        }
        a_gauche = !a_gauche;

        vTaskDelay(pdMS_TO_TICKS(400));
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
