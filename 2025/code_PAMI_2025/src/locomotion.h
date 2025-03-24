#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <FreeRTOS.h>
#include "semphr.h"

#define STEP_PER_MM 6.68 //A change
#define RAYON_PAMI 4.725

typedef struct{
    float x;
    float y;
    float theta;
  } coord;
  

class Locomotion {
public:
    Locomotion(AccelStepper* step_left, AccelStepper* step_right, pin_size_t en_left, pin_size_t en_right);

    void start();
    void doStep();
    void stop();

    int translateBlocking(long steps);
    int rotateBlocking(long steps);

    coord getPositon(){return current_coord;};
    void odometry();

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
