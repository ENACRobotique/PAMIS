#ifndef PILOT_H
#define PILOT_H

#include "Gpios.h"
#include "TeensyStep.h"

#define STEP_PER_MM 10.186
#define STEP_TO_DEGREES 10 // chercher 
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000
#define PULL_IN_SPEED 10 

class pilot
{
    private:
        Stepper * _stepper_left;
        Stepper * _stepper_right;
        StepControl _controller;
        
        Gpios::Signal _left_enable;
        Gpios::Signal _right_enable;

        int _current_pos;
        float _current_angle;

        int _current_step_left;
        int _current_step_right;
        
        int _target_step_left;
        int _target_step_right;

    public:
        pilot();
        pilot( 
            Stepper * left_STEP,Gpios::Signal left_ENABLE,
            Stepper * right_STEP,Gpios::Signal  right_ENABLE);

        void init();

        //basics
        void straight_line(int dist);
        void turn(float angle);
        void update();

        //odometry
        void step_pos();



        //TODO 
        void get_pose_2D(); // x, y , theta
        void parcour(); // pile de position successives
};


#endif