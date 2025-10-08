#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"


constexpr float MM2STEP = ((8*200)/(M_PI*76.1));
#define RAYON_PAMI 67

/****************
    x
    ^
    |
    |   
y<---

*****************/

long convertMmToStep(float mms);


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
    
    int translateBlocking(long steps);
    int rotateBlocking(float angle);
    int moveBlocking(coord target);
    int move(coord * targets,int nb);
    int superstar(coord coords);
    void avoid(float angle);
    void suiviLignes(sidE side);
    void initPos(coord init){current_coord = init;};
    coord getPositon(){return current_coord;};
    void odometry();
    etat state = INIT;
    sidE side=NONE;
    float avoid_angle=0;
    int target_idx=0;
    

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
