#include "evitement.h"
#include "locomotion.h"
#include "radar_vl53.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern Locomotion locomotion;

#ifndef RADAR_NB
#define RADAR_NB 3
#endif // on aura 3 radar sur les PAMI_2026

const uint16_t SEUIL_STOP = 150; // 15 cm devant le centre pas réllement devant le robot a régler avec les vrai pami

bool obstacle_fixe_detecte = false;
bool evitement_suspendu = false;

void task_evitement(void *arg)
{
    bool en_arret_urgence = false;
    int temps_sans_obstacle = 0;

    while (1)
    {
        bool obstacle_detecte = false;

        // Lecture des radars
        for (uint16_t i = 0; i < RADAR_NB; i++)
        {
            int dist = get_distance(i);

            if (dist < SEUIL_STOP)
            {
                obstacle_detecte = true;
                break;
            }
        }

        if (obstacle_detecte)
        {
            temps_sans_obstacle = 0;

            if (!en_arret_urgence)
            {
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
                    locomotion.resumeTrajectory();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void init_evitement()
{
    xTaskCreate(task_evitement, "evitement", configMINIMAL_STACK_SIZE + 1024, NULL, 3, NULL);
}