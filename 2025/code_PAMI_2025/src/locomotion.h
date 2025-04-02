#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"

#define MM2STEP (20/3)
#define RAYON_PAMI 51.55
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


    coord getPositon(){return current_coord;};
    void odometry();
    etat state = INIT;
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
