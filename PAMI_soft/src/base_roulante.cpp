#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Arduino.h>


#define MOT1_DIR PA5

// PA5 -> PB5
#define MOT2_DIR PB4
#define MOT1_PWM PA11
#define MOT2_PWM PA8
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
    etat_evitement = false;
}


void Base_roulante::addCommand (command_t cmd){
    if (nb_elem<TAILLE_FILE){
        commands[cmd_ecrire]=cmd;
        cmd_ecrire = (cmd_ecrire + 1) % TAILLE_FILE;
        nb_elem ++;
    }
}

void Base_roulante::update_commands (coord* list){
    if (!(controller.isRunning()) && nb_elem>0 && !etat_evitement){
        if (commands[cmd_a_executer].command_name == TRANSLATE){
            this->translate(commands[cmd_a_executer].value);
        }else{
            this->rotate(commands[cmd_a_executer].value);
        }
        current_coord = list[cmd_a_executer]; //le nb d'elem ne depassera jamais TAILLE_FILE sinon ca marche pas
        last_command = commands[cmd_a_executer];
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

void Base_roulante::stop(){
    if (controller.isRunning()){
        controller.stopAsync();    
    }
}


void Base_roulante::evitement(int v_gauche, int v_droit){
    
    if (last_command.command_name == ROTATE){
        int rot_l = stepper_left.getPosition();
        current_coord.theta +=  rot_l/(RAYON_PAMI*STEP_PER_MM);
        stepper_left.setPosition(0);
        stepper_right.setPosition(0);
    }else{
        int trans_l = stepper_left.getPosition();
        float dist = trans_l/(STEP_PER_MM);
        current_coord.x = dist * cos(current_coord.theta);
        current_coord.y = dist * sin(current_coord.theta);
    }
    
    if (v_gauche < DISTANCE_EVITEMENT && v_droit > DISTANCE_EVITEMENT){
        etat_evitement_temp=1;
        if ((!etat_evitement) || (etat_evitement && !controller.isRunning())){
            etat_evitement = true;
            rotate(-0.4);
            current_coord.theta -= 0.4;
        }
        
            
        //evite a droite
    }else if (v_gauche > DISTANCE_EVITEMENT && v_droit < DISTANCE_EVITEMENT){
        etat_evitement_temp=2;
        if ((!etat_evitement) || (etat_evitement && !controller.isRunning())){
            etat_evitement = true;
            rotate(0.4);
            current_coord.theta += 0.4;
        }

    }else if (v_gauche < DISTANCE_EVITEMENT && v_droit < DISTANCE_EVITEMENT){
        etat_evitement_temp=3;
        if (!etat_evitement){
            etat_evitement = true;
            stop();
        }

    }else{
        etat_evitement_temp=0;
        if (etat_evitement){
            translate(400);
            etat_evitement = false;
            //move(last_move_x, last_move_y);
        }
        
    }
}


