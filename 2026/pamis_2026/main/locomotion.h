#pragma once
#include "stepper.h"
#include "radar_vl53.h"

struct Position {
    float x;
    float y;
    float theta;

};

struct DistancesRoues {
    float d1;
    float d2;
};

enum mvm_status {
    IDLE_mvm,
    TURN_mvm,
    CRUISE_mvm,
    TURN_FINAL_mvm,
};

class Locomotion {
public:
    void init();

    void move(float lenght, float angle);

    void moveBlocking(float lenght, float angle);
    DistancesRoues stop();
    void set_seuils(float a,float b,float c,float d, float e);
    bool danger();
    DistancesRoues leg(DistancesRoues d);
    void moveEvitement(float d,float alpha);
    void set_speed(float v, float a);

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
    void _move(float d1, float d2);
    Stepper step_left;
    Stepper step_right;
    
    float seuils[RADAR_NB];

    float vitesse_pami;
    float acceleration_pami;

    Position pos;

    
    
    float oldPosLeft;
    float oldPosRight;
    
    // mvm_status -> enum ?? -> mvm_status mvm_etat = IDLE
    // Il faut éviter d'avoir des doublons de IDLE : stepper et locomotion ??
    mvm_status mvm_etat = IDLE_mvm;

    SemaphoreHandle_t traj_sem;
    Position traj_points[200];
    int traj_length;
    TaskHandle_t traj_TaskHandle = NULL;
    bool is_aborted = true;
};


extern Locomotion locomotion;

