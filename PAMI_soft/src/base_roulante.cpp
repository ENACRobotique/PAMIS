#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Arduino.h>


#define MOT1_DIR PB4

// PA5 -> PB5
#define MOT2_DIR PA5
#define MOT1_PWM PA8
#define MOT2_PWM PA11
#define MOT1_STEP MOT1_PWM
#define MOT2_STEP MOT2_PWM

Stepper stepper_left(MOT1_STEP, MOT1_DIR);
Stepper stepper_right(MOT2_STEP, MOT2_DIR);
StepControl controller;

void Base_roulante::init(){
    cmd_a_executer = 0;
    cmd_ecrire = 0;
    nb_elem = 0;
    absc=0;
    ord=0;
    teta_0=0;
}


void Base_roulante::addCommand (command_t cmd){
    if (nb_elem<TAILLE_FILE){
        commands[cmd_ecrire]=cmd;
        cmd_ecrire = (cmd_ecrire + 1) % TAILLE_FILE;
        nb_elem ++;
    }
}

void Base_roulante::update_commands (){
    if (!(controller.isRunning()) && nb_elem>0){
        if (commands[cmd_a_executer].command_name == TRANSLATE){
            this->translate(commands[cmd_a_executer].value);
        }else{
            this->rotate(commands[cmd_a_executer].value);
        }
        cmd_a_executer = (cmd_a_executer + 1)% TAILLE_FILE;
        nb_elem --;
    }
}

void Base_roulante::rotate(float angle) {
    double temp  = RAYON_PAMI* (angle) * STEP_PER_MM;
    stepper_left.setTargetRel(temp);
    stepper_right.setTargetRel(temp);
    controller.moveAsync(stepper_left, stepper_right);
}
void Base_roulante::translate(float distance) {
    stepper_left.setTargetRel(distance * STEP_PER_MM);
    stepper_right.setTargetRel(-distance * STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);
}

void Base_roulante::move(float x,float y) {
    
    float dx= x- absc;
    float dy= y - ord;
    if (dx !=0 || dy !=0){
        double new_angle = atan2(dy,dx);
        float rotate_angle = new_angle-teta_0;
        if (rotate_angle > PI){rotate_angle -= 2*PI;}

        addCommand({ROTATE,rotate_angle});
        teta_0= new_angle;
        addCommand({TRANSLATE,sqrt(static_cast<float>(pow(dx,2)+pow(dy,2)))});
        absc=x;
        ord=y; 
    }
}
void Base_roulante::tour(float rayon){
    stepper_right.setTargetRel(-2*M_PI*(RAYON_PAMI+rayon)* STEP_PER_MM);
    stepper_left.setTargetRel(2*M_PI*(rayon-RAYON_PAMI)* STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);  
}

