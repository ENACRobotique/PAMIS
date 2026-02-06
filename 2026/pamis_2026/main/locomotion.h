#pragma once
#include "stepper.h"

struct Position {
    float x;
    float y;
    float theta;

};

enum mvm_status {
    IDLE,
    TURN,
    CRUISE,
    TURN_FINAL
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

    static void trajectory_task(void* arg);
    /**
     * @return pdTRUE if move finished, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);

    
    void trajectory(Position* dest, int nb_pts);
    
    int trajectory_movement();

    private:
    Stepper step_left;
    Stepper step_right;
    
    Position pos;
    
    float oldPosLeft;
    float oldPosRight;
    
    // mvm_status -> enum ?? -> mvm_status mvm_etat = IDLE
    // Il faut éviter d'avoir des doublons de IDLE : stepper et locomotion ??
    mvm_status mvm_etat = IDLE;

    SemaphoreHandle_t traj_sem;
    Position traj_points[10];
    int traj_length;
};
