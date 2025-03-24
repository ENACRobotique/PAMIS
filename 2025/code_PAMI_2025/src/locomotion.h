#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"

#define MM2STEP 20/3
#define RAYON_PAMI 47.25

typedef struct {
    float x;
    float y;
    float theta;
  }coord;
  

class Locomotion {
public:
    Locomotion(AccelStepper* step_left, AccelStepper* step_right, pin_size_t en_left, pin_size_t en_right);

    void start();
    void doStep();
    void stop();

    int translateBlocking(long steps);
    int rotateBlocking(long steps);
    int moveBlocking(coord target);


    coord getPositon(){return current_coord;};
    void odometry();
    int etat = 0;

private:
    
    coord current_coord;

    AccelStepper* step_left;
    AccelStepper* step_right;
    pin_size_t en_left;
    pin_size_t en_right;

    SemaphoreHandle_t mutex;

    bool stopped;
};

extern Locomotion locomotion;
