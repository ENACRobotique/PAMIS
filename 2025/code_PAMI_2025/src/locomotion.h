#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include "config.h"

#if defined (JOHNNY)
    #define MM2STEP (6.68*2*(90.0/83.0))
#else
    #define MM2STEP (6.68)
#endif

#define RAYON_PAMI 46 //49.94

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
    STOPPED,
};

enum sidE{
    GAUCHE,
    DROITE,
    DERRIERE,
    NONE,
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
    void resume();
    int translateBlocking(long steps);
    int rotateBlocking(float angle);
    int moveBlocking(coord target);
    int move(coord * targets,int nb);
    int superstar(coord* targets, int nb);
    void avoid(float angle);
    void suiviLignes(sidE side);
    void initPos(coord init){current_coord = init;};
    coord getPositon(){return current_coord;};
    void odometry();
    etat state = INIT;
    sidE side=NONE;
    float avoid_angle=0;
    int target_idx=0;
    bool finDuMatch=false;
    
    
    private:
    
    coord current_coord = {0,0,0};
    float old_pos_1 = 0;
    float old_pos_2 = 0;
    TaskHandle_t xHandle;
    AccelStepper* step_left;
    AccelStepper* step_right;
    pin_size_t en_left;
    pin_size_t en_right;

    SemaphoreHandle_t mutex;

    bool stopped;
};

extern Locomotion locomotion;
