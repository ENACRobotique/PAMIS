#include "stratWallid.h"
#include "locomotion.h"
#include "stepper.h"
#include "sts3032.h"
#include "config.h"

TaskHandle_t ninja_handle = NULL;
TaskHandle_t manger_handle = NULL;
StratWallid stratWallid;

void manger(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    printf("MAAAAANNNGER !!!!!!!!!!!");

    while (true)
    {
        // remuer la queue
        stratWallid.danser();
    }
}

void match_end_cb(TimerHandle_t xTimer)
{
    printf("FIN DU MATCH  !!!!!!!!!!!");
    if (ninja_handle != NULL)
    {
        printf("stoping strat...");
        vTaskDelete(ninja_handle);
        // vTaskSuspend(ninja_handle);
    }

    if (manger_handle != NULL)
    {
        printf("notify manger...");
        xTaskNotifyGive(manger_handle);
    }
}

void strat_grenier(void *arg)
{

    ninja_handle = xTaskGetCurrentTaskHandle();

    xTaskCreate(manger, "manger", 4096, NULL, 1, &manger_handle);

    while (gpio_get_level(FDC1) == 0)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    auto timer = xTimerCreate("MatchEndTimer", pdMS_TO_TICKS(MATCH_TIME * 1000), pdFALSE, (void *)0, match_end_cb);
    xTimerStart(timer, portMAX_DELAY);

    int cote;
    int bras_mid;
    int bras_bas;
    if (gpio_get_level(FDC3))
    {
        cote = 1;
        bras_mid = 510;
        bras_bas = 350;
    }
    else
    {
        cote = -1;
        bras_mid = 1690;
        bras_bas = 1850;
    }
    Position test[2] = {
        {.x = 0, .y = 500, .theta = 0},
        {.x = 500, .y = 500, .theta = M_PI},
    };

    stratWallid.set_team(cote, bras_mid, bras_bas);
    sts3032::move(7, 3050);
    sts3032::move(1, 1100);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    locomotion.set_speed(500, 3000);
    stratWallid.vider_frigos();
    stratWallid.moveWallid(550, 0);
    stratWallid.taper_mur();
    locomotion.set_seuils(200, 0, 0, 0, 0);
    stratWallid.moveWallid(-290, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    stratWallid.moveWallid(0, -M_PI / 2);
    stratWallid.taper_mur();
    locomotion.set_seuils(200, 0, 0, 0, 0);
    stratWallid.moveWallid(-345, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    stratWallid.sortir_caisse();
    stratWallid.placer_frigo_1();
    stratWallid.moveWallid(480, 0);
    stratWallid.taper_mur();
    stratWallid.moveWallid(-240, 0);
    stratWallid.pousser_caisse();
    stratWallid.sortir_caisse();
    stratWallid.placer_frigo_2();
    stratWallid.moveWallid(110, 0);
    stratWallid.pousser_caisse();
    stratWallid.sortir_caisse();
    stratWallid.moveWallid(50, 0);
    stratWallid.moveWallid(0, M_PI / 2);
    stratWallid.moveWallid(260, 0);
    stratWallid.manger(1);
    stratWallid.danser();
}

int cote;
int bras_mid;
int bras_bas;
void StratWallid::set_team(int c, int bm, int bb)
{
    cote = c;
    bras_mid = bm;
    bras_bas = bb;
}

void StratWallid::moveWallid(float d, float alpha)
{
    locomotion.moveEvitement(d, cote * alpha);
}

void StratWallid::sortir_caisse()
{
    moveWallid(60, 0);
    sts3032::move(1, bras_mid);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    moveWallid(225, -0.12);
    sts3032::move(1, 1100);
    moveWallid(40, 0);
    taper_mur();
    locomotion.set_seuils(50, 0, 0, 0, 0);
    locomotion.set_speed(200, 3000);
    moveWallid(-45, 0);
    locomotion.set_speed(500, 3000);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(0, M_PI / 2);
    locomotion.set_speed(100, 500);
    moveWallid(60, 0);
    locomotion.set_speed(500, 3000);
}

void StratWallid::placer_frigo_1()
{
    sts3032::move(7, 2020);
    vTaskDelay(50);
    locomotion.set_speed(100, 500);
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-150, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(0, M_PI * 5 / 6);
    locomotion.set_speed(500, 3000);
    moveWallid(220, 0);
    sts3032::move(7, 3050);
    vTaskDelay(50);
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-50, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(0, M_PI / 2 + 0.5);
    locomotion.set_seuils(0, 50, 0, 0, 0);
    moveWallid(100, 0);
    locomotion.set_seuils(0, 0, 0, 0, 0);
    moveWallid(40, 0);
    taper_mur();
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-45, 0);
    moveWallid(0, M_PI / 2);
    locomotion.set_seuils(0, 200, 0, 0, 0);
}

void StratWallid::placer_frigo_2()
{
    sts3032::move(7, 2020);
    vTaskDelay(50);
    locomotion.set_speed(100, 500);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(-150, 0);
    locomotion.set_seuils(200, 0, 0, 0, 0);
    locomotion.set_speed(500, 3000);
    moveWallid(0, M_PI * 3 / 4);
    moveWallid(50, 0);
    sts3032::move(7, 3050);
    vTaskDelay(50);
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-75, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(0, M_PI * 1.8 / 2);
    moveWallid(40, 0);
    taper_mur();
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-25, 0);
    moveWallid(0, M_PI / 2);
    locomotion.set_seuils(0, 200, 0, 0, 0);
}

void StratWallid::pousser_caisse()
{
    moveWallid(0, -M_PI / 2);
    vTaskDelay(50);
    moveWallid(15, 0);
    taper_mur();
    locomotion.set_speed(100, 500);
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-350, 0);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    locomotion.set_speed(500, 3000);
}

void StratWallid::vider_frigos()
{
    moveWallid(420, 0);
    moveWallid(0, -M_PI / 2);
    vider_frigo1();
    moveWallid(0, M_PI / 2);
    moveWallid(110, 0);
    moveWallid(0, -M_PI / 2);
    vider_frigo2();
    locomotion.set_seuils(0, 40, 0, 0, 0);
    locomotion.set_speed(200, 3000);
    moveWallid(260, 0);
    locomotion.set_seuils(0, 0, 0, 0, 0);
    moveWallid(50, 0);
    taper_mur();
    locomotion.set_seuils(200, 0, 0, 0, 0);
    moveWallid(-45, 0);
    moveWallid(0, M_PI / 2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    locomotion.set_seuils(0, 200, 0, 0, 0);
}

void StratWallid::vider_frigo2()
{
    moveWallid(100, 0);
    sts3032::move(1, bras_bas);
    vTaskDelay(50);
    moveWallid(180, -0.2);
    sts3032::move(1, 1100);
    vTaskDelay(50);
    locomotion.set_seuils(50, 0, 0, 0, 0);
    locomotion.set_speed(200, 3000);
    moveWallid(-190, 0);
    locomotion.set_speed(500, 3000);
    locomotion.set_seuils(0, 200, 0, 0, 0);
    moveWallid(0, M_PI / 2);
    moveWallid(45, 0);
    moveWallid(0, -M_PI / 2);
    sts3032::move(1, bras_bas);
    vTaskDelay(50);
    moveWallid(220, -0.2);
    sts3032::move(1, 1100);
    moveWallid(0, M_PI);
}

void StratWallid::vider_frigo1()
{
    moveWallid(270, 0);
    locomotion.set_seuils(50, 0, 0, 0, 0);
    locomotion.set_speed(200, 3000);
    moveWallid(-270, 0);
    locomotion.set_speed(500, 3000);
    locomotion.set_seuils(0, 200, 0, 0, 0);
}

void StratWallid::taper_mur()
{
    locomotion.set_speed(100, 500);
    moveWallid(50, 0);
    locomotion.set_speed(500, 3000);
}

void StratWallid::danser()
{
    while (true)
    {
        sts3032::move(1, 1690);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        sts3032::move(1, 510);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void StratWallid::manger(int distance)
{
    // 1 = loin
    // 0 = proche
    int dist = 0;
    if (distance == 1)
    {
        dist = 870;
    }
    else if (distance == 0)
    {
        dist = 370;
    }

    locomotion.set_seuils(0, 0, 0, 0, 0);
    moveWallid(-15, 0);
    moveWallid(0, -M_PI / 2);
    taper_mur();
    moveWallid(-45, 0);
    moveWallid(0, M_PI);
    locomotion.set_seuils(0, 300, 0, 0, 0);
    moveWallid(dist, 0);
    sts3032::move(7, 2020);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    moveWallid(0, -M_PI / 4);
}

// // wait to plug tirette
// while (gpio_get_level(FDC1) == 0) {
//     vTaskDelay(50 / portTICK_PERIOD_MS);
// }
