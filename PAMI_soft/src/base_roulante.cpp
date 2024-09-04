#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define MOT1_DIR PB5 
#define MOT2_DIR PB4
#define MOT1_STEP PA11
#define MOT2_STEP PA8

Stepper stepper_left(MOT1_STEP, MOT1_DIR);
Stepper stepper_right(MOT2_STEP, MOT2_DIR);
StepControl controller;



void Base_roulante::init(){
    cmd_a_executer = 0;
    cmd_ecrire = 0;
    nb_elem = 0;
    current_coord = {0,0,0};
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

bool Base_roulante::commands_finished(){
    return nb_elem == 0 && !controller.isRunning();
}

void Base_roulante::translate_block(float distance){
    translate(distance);
    while(controller.isRunning()){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void Base_roulante::rotate_block(float angle){
    rotate(angle);
    while(controller.isRunning()){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}


void Base_roulante::rotate(float angle) {
    double temp  = RAYON_PAMI * angle * STEP_PER_MM;
    stepper_left.setTargetRel(-temp);
    stepper_right.setTargetRel(-temp);
    controller.moveAsync(stepper_left, stepper_right);
}

void Base_roulante::translate(float distance) {
    stepper_left.setTargetRel(distance * STEP_PER_MM);
    stepper_right.setTargetRel(-distance * STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);
}

void Base_roulante::move(float x,float y) {
    float dx= x- current_coord.x;
    float dy= y - current_coord.y;
    if (dx !=0 || dy !=0){
        double new_angle = atan2(dy,dx);
        float rotate_angle = new_angle-current_coord.theta;
        if (rotate_angle > PI){rotate_angle -= 2*PI;}

        addCommand({ROTATE,rotate_angle});
        addCommand({TRANSLATE,sqrt(static_cast<float>(pow(dx,2)+pow(dy,2)))});
    }
}

void Base_roulante::rotate_point(float x, float y){
    float dx= x - current_coord.x;
    float dy= y - current_coord.y;
    if (dx !=0 || dy !=0){
        double new_angle = atan2(dy,dx);
        float rotate_angle = new_angle-current_coord.theta;
        if (rotate_angle > PI){rotate_angle -= 2*PI;}

        addCommand({ROTATE,rotate_angle});
    }
}

void Base_roulante::translate_point(float x, float y){
    float dx= x- current_coord.x;
    float dy= y - current_coord.y;
    addCommand({TRANSLATE,sqrt(static_cast<float>(pow(dx,2)+pow(dy,2)))});
}

void Base_roulante::stop(){
    if (controller.isRunning()){
        controller.stopAsync();    
    }
}


void Base_roulante::odometry (){
  static float old_pos_1 = 0;
  static float old_pos_2 = 0;
  float pos_1 = stepper_left.getPosition();
  float pos_2 = stepper_right.getPosition();

  float dpos_1 = (pos_1 - old_pos_1) / STEP_PER_MM;
  float dpos_2 = (pos_2 - old_pos_2) / STEP_PER_MM; 

  current_coord.theta += (-dpos_1 - dpos_2) / (2 * RAYON_PAMI);
  current_coord.x += ((dpos_1 - dpos_2)/2) * cos (current_coord.theta);
  current_coord.y += ((dpos_1 - dpos_2)/2) * sin (current_coord.theta); 

  old_pos_1 = pos_1;
  old_pos_2 = pos_2;

}







