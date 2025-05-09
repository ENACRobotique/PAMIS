#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"

#define MM2STEP (20/3)
#define RAYON_PAMI 49.94 //48.33 //51.55
// (47.25*(12/11))

/****************
    x
    ^
    |
    |   
y<---

*****************/




typedef struct {
    float x;
    float y;
    float theta;
  }coord;
  
enum etat{
    INIT,
    TOURNE,
    TOURNE_FINI,
    TOUDRWA,
    TOUDRWA_FINI,
    FINITOPIPO,
    AVOIDING,
    AVOIDINGTOURNE,
    AVOIDINGTOURNEFINI,
    AVOIDINGTOUDRWA,
    AVOIDINGTOUDRWAFINI,
    SUIVILIGNES,
    SUIVILIGNES2,
    SUIVILIGNES25,
    SUIVILIGNESFINI,
};

enum sidE{
    DROITE,
    NONE,
    GAUCHE,
    DERRIERE,
};

enum angleRecommandation{
    N,
    ANGLE30,
    NN,
    ANGLE90,
};

class Locomotion {
public:
    Locomotion(AccelStepper* step_left, AccelStepper* step_right, pin_size_t en_left, pin_size_t en_right);

    void start();
    void doStep();
    void stop();
    
    int translateBlocking(long steps);
    int rotateBlocking(float angle);
    int moveBlocking(coord target);
    int move(coord * targets,int nb);
    void avoid(String where,angleRecommandation angle);
    void suiviLignes(sidE side);


    coord getPositon(){return current_coord;};
    void odometry();
    etat state = INIT;
    sidE side=NONE;
    angleRecommandation angleRecommande=ANGLE90;
    int i=0;
    

private:
    
    coord current_coord = {0,0,0};
    float old_pos_1 = 0;
    float old_pos_2 = 0;

    AccelStepper* step_left;
    AccelStepper* step_right;
    pin_size_t en_left;
    pin_size_t en_right;

    SemaphoreHandle_t mutex;

    bool stopped;
};

extern Locomotion locomotion;
