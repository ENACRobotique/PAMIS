#pragma once
#include "stepper.h"

struct Position {
    float x;
    float y;
    float theta;
};

class Locomotion {
public:
    void init();

    void move(float lenght, float angle);
    void moveBlocking(float lenght, float angle);
    void stop();

    void enableSteppers(bool enable);

    bool moving();

    Position getPos() {return pos;}

    /**
     * @return pdTRUE if move finished, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);

private:
    Stepper step_left;
    Stepper step_right;

    Position pos;

};
