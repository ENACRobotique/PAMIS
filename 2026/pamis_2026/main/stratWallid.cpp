#include "stratWallid.h"
#include "locomotion.h"
#include "stepper.h"
#include "sts3032.h"
#include "config.h"


StratWallid stratWallid;

void strat_grenier(void* arg) {

    while (gpio_get_level(FDC1) == 0) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    int cote;
    int bras_mid;
    int bras_bas;
    if (gpio_get_level(FDC3)){
        cote=1;
        bras_mid = 510;
        bras_bas = 350;
    }
    else{
        cote=-1;
        bras_mid = 1690;
        bras_bas = 1850;
    }
    Position test[2] = {
        {.x=0, .y=500, .theta=0},
        {.x=500, .y=500, .theta=M_PI},
    };


    stratWallid.set_team(cote, bras_mid, bras_bas);
    sts3032::move(7,3050);
    sts3032::move(1,1100);
    locomotion.set_seuils(0,200,0,0,0);
    locomotion.set_speed(3000,500);
    stratWallid.vider_frigos();
    stratWallid.moveWallid(570,0);
    stratWallid.taper_mur();
    locomotion.set_seuils(200,0,0,0,0);
    stratWallid.moveWallid(-290,0);
    locomotion.set_seuils(0,200,0,0,0);
    stratWallid.moveWallid(0,-M_PI/2);
    stratWallid.taper_mur();
    locomotion.set_seuils(200,0,0,0,0);
    stratWallid.moveWallid(-345,0);
    locomotion.set_seuils(0,200,0,0,0);
    stratWallid.sortir_caisse();
    stratWallid.placer_frigo_1();
    stratWallid.moveWallid(230,0);
    stratWallid.pousser_caisse();
    stratWallid.sortir_caisse();
    stratWallid.placer_frigo_2();
    stratWallid.moveWallid(110,0);
    stratWallid.pousser_caisse();
    stratWallid.sortir_caisse();
    stratWallid.moveWallid(50,0);
    stratWallid.moveWallid(0,M_PI/2);
    stratWallid.moveWallid(245,0);



}







int cote;
int bras_mid;
int bras_bas;
void StratWallid::set_team(int c, int bm, int bb){
    cote = c;
    bras_mid = bm;
    bras_bas = bb;
}

void StratWallid::moveWallid(float d,float alpha){
    locomotion.moveEvitement(d, cote*alpha);
}

void StratWallid::sortir_caisse(){
    moveWallid(60,0);
    sts3032::move(1,bras_mid);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    moveWallid(225,-0.12);
    sts3032::move(1,1100);
    moveWallid(0,M_PI/2);
    locomotion.set_speed(500,100);
    moveWallid(60,0);
    locomotion.set_speed(3000,500);
}

void StratWallid::placer_frigo_1(){
    sts3032::move(7,2020);
    vTaskDelay(50);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-150,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI*0.9);
    moveWallid(180,0);
    sts3032::move(7,3050);
    vTaskDelay(50);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-50,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI/2+0.4);
    moveWallid(110,0);
    taper_mur();
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-45,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI/2);

}

void StratWallid::placer_frigo_2(){
    sts3032::move(7,2020);
    vTaskDelay(50);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(-150,0);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(0,M_PI*3/4);
    moveWallid(50,0);
    sts3032::move(7,3050);
    vTaskDelay(50);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-75,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI*1.8/2);
    moveWallid(40,0);
    taper_mur();
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-25,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI/2);

}

void StratWallid::pousser_caisse(){
    moveWallid(0,-M_PI/2);
    vTaskDelay(50);
    moveWallid(15,0);
    taper_mur();
    locomotion.set_speed(500,100);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-350,0);
    locomotion.set_seuils(0,200,0,0,0);
    locomotion.set_speed(3000,500);
}

void StratWallid::vider_frigos(){
    moveWallid(420,0);
    moveWallid(0,-M_PI/2);
    vider_frigo1();
    moveWallid(0,M_PI/2);
    moveWallid(110,0);
    moveWallid(0,-M_PI/2);
    vider_frigo2();
    moveWallid(290,0);
    taper_mur();
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-45,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI/2);

}

void StratWallid::vider_frigo2(){
    moveWallid(100,0);
    sts3032::move(1,bras_bas);
    vTaskDelay(50);
    moveWallid(180,-0.2);
    sts3032::move(1,1100);
    vTaskDelay(50);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-190,0);
    locomotion.set_seuils(0,200,0,0,0);
    moveWallid(0,M_PI/2);
    moveWallid(45,0);
    moveWallid(0,-M_PI/2);
    sts3032::move(1,bras_bas);
    vTaskDelay(50);
    moveWallid(210,-0.2);
    moveWallid(0,M_PI);
    sts3032::move(1,1100);
}

void StratWallid::vider_frigo1(){
    moveWallid(270,0);
    locomotion.set_seuils(200,0,0,0,0);
    moveWallid(-270,0);
    locomotion.set_seuils(0,200,0,0,0);
}

void StratWallid::taper_mur(){
    locomotion.set_speed(500,100);
    moveWallid(50,0);
    locomotion.set_speed(3000,500);
}

    // // wait to plug tirette
    // while (gpio_get_level(FDC1) == 0) {
    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }