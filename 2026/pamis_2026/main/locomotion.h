#pragma once
#include "stepper.h"

class Locomotion {
public:
    void init();

    void move(float lenght, float angle);
    void moveBlocking(float lenght, float angle);
    void stop();

    bool moving();

    /**
     * @return pdTRUE if move finished, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);

private:
    Stepper step_left;
    Stepper step_right;

};
