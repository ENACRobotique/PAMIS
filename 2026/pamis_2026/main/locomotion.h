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
    void setPos(Position new_pos) {pos = new_pos;}

    float getPosLeft() {
        return step_left.getPositionMm();
    }

    float getPosRight() {
        return step_right.getPositionMm();
    }

    static void odometry_task(void* arg);

    /**
     * @return pdTRUE if move finished, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);

private:
    Stepper step_left;
    Stepper step_right;

    Position pos;

    float oldPosLeft;
    float oldPosRight;

};