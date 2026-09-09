#include "evitement.h"
#include "locomotion.h"
#include "radar_vl53.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "utils.h"
#include "config.h"
#include "scs009.h"
extern Locomotion locomotion;

#ifndef RADAR_NB
#define RADAR_NB 3
#endif // on aura 3 radar sur les PAMI_2026

const uint16_t SEUIL_STOP = 150;                                 // 15cm devant le robot
const float ANGLES_RADAR[3] = {-M_PI / 4.5f, 0.0f, M_PI / 4.5f}; // 40degres
const float DISTANCE_CENTRE_CAPTEUR = 50.0;
const float MARGE_MUR = 50.0;

extern int strat_id;

extern uint8_t id_servo_queue;

bool obstacle_fixe_detecte = false;
bool evitement_suspendu = false;

void danser()
{

    bool a_gauche = true;
    const uint16_t POS_GAUCHE = 200; // Centre (512) - 112
    const uint16_t POS_DROITE = 500; // Centre (512) + 1
    // printf("je suis arrive pour le faire bouger \n");
    locomotion.disableSteppers();
    while (1)
    {
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

bool est_un_obstacle_statique(float obs_x, float obs_y)
{
    // détection des mur de la carte
    if (obs_x < MARGE_MUR || obs_x > (3000.0f - MARGE_MUR))
        return true;
    if (obs_y < MARGE_MUR || obs_y > (2000.0f - MARGE_MUR))
        return true;
    // on vérifie le grenier la
    if (obs_x > 600.0f && obs_x < 2400.0f && obs_y > 1600.0f)
        return true;

    // si l'obstacle n'est pas la
    return false;
}

void task_evitement(void *ID)
{
    bool en_arret_urgence = false;
    int temps_sans_obstacle = 0;
    uint32_t strat_id = (uint32_t)ID;

    while (1)
    {
        uint32_t temps_ecoule_ms = (xTaskGetTickCount() - locomotion.match_start_time) * portTICK_PERIOD_MS;

        // printf("%ld \n", locomotion.match_start_time);
        if (temps_ecoule_ms >= 99000 && locomotion.match_start_time != 0)
        {
            danser();
        }
        // R Tryndamère :: invincible
        float x_robot = locomotion.pos.x;
        float y_robot = locomotion.pos.y;
        bool dans_zone_jaune = (x_robot < 450.0f && y_robot > 1600.0f);
        bool dans_zone_bleue = (x_robot > 2550.0f && y_robot > 1600.0f);
        bool non_initialise = (x_robot == 0.0f && y_robot == 0.0f);

        if (dans_zone_jaune || dans_zone_bleue || non_initialise)
        {
            if (en_arret_urgence)
            {
                printf("PURGE : Retrait de l'arret d'urgence dans le nid !\n");
                en_arret_urgence = false;
                temps_sans_obstacle = 0;
                locomotion.resumeTrajectory();
            }

            // On est dans le nid on regarde pas les obstacle
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        bool obstacle_detecte = false;

        // Lecture des radars
        for (uint16_t i = 0; i < RADAR_NB; i++)
        {
            int dist = get_distance(i);

            if (dist > 10 && dist < SEUIL_STOP)
            {
                float angle_absolu = normalise(locomotion.pos.theta + ANGLES_RADAR[i]);

                // 2. Calcul de la position (X, Y) de l'obstacle
                float distance_totale = dist + DISTANCE_CENTRE_CAPTEUR;
                float obs_x = locomotion.pos.x + distance_totale * cos(angle_absolu);
                float obs_y = locomotion.pos.y + distance_totale * sin(angle_absolu);

                // printf(" yellow = %i, strat_id == %u, i = %i ", gpio_get_level(FDC2), (unsigned int)strat_id, i);

                if (gpio_get_level(FDC2) == 0 && strat_id == 4 && i == 2)
                {
                    printf("je suis blouqer par le grenier jaune \n");
                }
                else if (gpio_get_level(FDC2) == 1 && strat_id == 4 && i == 0)
                {
                    printf("je suis blouqer par le grenier bleu \n");
                }
                else if (gpio_get_level(FDC2) == 0 && strat_id == 4 && i == 1)
                {
                    printf("je suis blouqer par le grenier jaune \n");
                }
                else if (gpio_get_level(FDC2) == 1 && strat_id == 4 && i == 1)
                {
                    printf("je suis blouqer par le grenier bleu \n");
                }
                else if (est_un_obstacle_statique(obs_x, obs_y))
                {
                    // C'est un mur de la table ou le grenier, on ne fait rien
                    // obstacle_detecte = true;
                    // break;
                }
                else
                {
                    obstacle_detecte = true;
                    break;
                }
            }
        }

        if (obstacle_detecte)
        {
            temps_sans_obstacle = 0;

            if (!en_arret_urgence)
            {
                locomotion.set_speed(500, 10000);
                // printf("v = %f  a = %f \n", locomotion.vitesse_pami, locomotion.acceleration_pami);
                printf("pause traj obstacle\n");
                locomotion.pauseTrajectory();
                locomotion.stop();

                en_arret_urgence = true;
            }
        }
        else
        {
            if (en_arret_urgence)
            {
                temps_sans_obstacle += 20;

                if (temps_sans_obstacle > 500)
                {
                    en_arret_urgence = false;
                    locomotion.set_speed(500, 2000);
                    printf(" reprise v = %f  a = %f \n", locomotion.vitesse_pami, locomotion.acceleration_pami);
                    locomotion.resumeTrajectory();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
