#include "pilot.h"

pilot::pilot(){}
pilot::pilot(
    Stepper * left_STEP,Gpios::Signal left_ENABLE,
    Stepper * right_STEP,Gpios::Signal  right_ENABLE)
{
    this->_stepper_left= left_STEP;
    this->_left_enable = left_ENABLE ;

    this->_stepper_right= right_STEP;
    this->_right_enable = right_ENABLE;
    
}

void pilot::init()
{
    // setup left stepper
    this->_stepper_left->setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(PULL_IN_SPEED)
                .setInverseRotation(true);
    gpios.write(this->_left_enable, LOW);       //enable motor

    // setup right stepper
    this->_stepper_right->setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(PULL_IN_SPEED)
                .setInverseRotation(true);
   
    gpios.write(this->_right_enable, LOW);       //enable motor
}


void pilot::straight_line(int dist) // dist in milimeters
{
    
    if (this->_controller.isRunning()){return;}

    this->_target_step_left = dist*STEP_PER_MM;
    this->_target_step_right = -dist*STEP_PER_MM ;

    this->_stepper_left->setTargetRel(this->_target_step_left);
    this->_stepper_right->setTargetRel(this->_target_step_right);
    
}

void pilot::turn(float angle) // angle in degrees, direction true for trigonometric
{  
    if (_controller.isRunning()){return;}
    
    this->_target_step_left = angle*STEP_TO_DEGREES;
    this->_target_step_right = -angle*STEP_TO_DEGREES;
    
    this->_stepper_left->setTargetRel(this->_target_step_left);
    this->_stepper_right->setTargetRel(this->_target_step_right);
}

void pilot::update()
{
    this->_controller.moveAsync(*(this->_stepper_left),*(this->_stepper_right));
}

void pilot::step_pos()
{
    this->_current_step_left = this->_stepper_left->getPosition();
    this->_current_step_right = this->_stepper_right->getPosition();
}