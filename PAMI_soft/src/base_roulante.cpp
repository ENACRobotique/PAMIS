#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include "Gpios.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"



#define TAILLE_FILE 10
#define STEP_PER_MM 6.68
constexpr float RAYON_PAMI = (152/2);
float absc=0;
float ord=0;
float teta_0=0;






Stepper stepper_left(MOT1_STEP, MOT1_DIR);
Stepper stepper_right(MOT2_STEP, MOT2_DIR);
StepControl controller;


enum CmdType {
    TRANSLATE,
    ROTATE,
};
enum Direction {
    Right,
    Left,
};

typedef struct{
    CmdType command_name;
    float value;
}command_t;





int cmd_a_executer = 0;
int cmd_ecrire = 0;
int nb_elem = 0;
command_t commands[TAILLE_FILE];
void addCommand(command_t cmd);
void update_commands();
void carre(float arete);
void tour(float rayon);

void addCommand (command_t cmd){
    if (nb_elem<10){
        commands[cmd_ecrire]=cmd;
        cmd_ecrire = (cmd_ecrire + 1) % TAILLE_FILE;
        nb_elem ++;
    }
}

void update_commands (){
    if (!(controller.isRunning()) && nb_elem>0){
        if (commands[cmd_a_executer].command_name == TRANSLATE){
            translate(commands[cmd_a_executer].value);
        }else{
            rotate(commands[cmd_a_executer].value);
        }
        cmd_a_executer = (cmd_a_executer + 1)% TAILLE_FILE;
        nb_elem --;
    }
}

void rotate(float angle) {
    stepper_left.setTargetRel(RAYON_PAMI* angle * STEP_PER_MM);
    stepper_right.setTargetRel(RAYON_PAMI* angle * STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);
}
void translate(float distance) {
    stepper_left.setTargetRel(distance * STEP_PER_MM);
    stepper_right.setTargetRel(-distance * STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);
}
void move(float x,float y) {
    
    float dx= x- absc;
    float dy= y - ord;
    if (dx !=0 and dy !=0){
        addCommand({ROTATE,(atan2(dy,dx)-teta_0)});
        teta_0=atan2(dy,dx);
        addCommand({TRANSLATE,sqrt(static_cast<float>(pow(dx,2)+pow(dy,2)))});
        absc=x;
        ord=y; 
    }
}
void tour(float rayon){
    stepper_right.setTargetRel(-2*M_PI*(RAYON_PAMI+rayon)* STEP_PER_MM);
    stepper_left.setTargetRel(2*M_PI*(rayon-RAYON_PAMI)* STEP_PER_MM);
    controller.moveAsync(stepper_left, stepper_right);  
}