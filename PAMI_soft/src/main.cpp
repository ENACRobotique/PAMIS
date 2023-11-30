#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include "Gpios.h"
#include <math.h>
#include <stdlib.h>

#define STEP_PER_MM 6.68
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000
#define TAILLE_FILE 10
constexpr float RAYON_PAMI = (152/2);

Stepper stepper_left(MOT1_STEP, MOT1_DIR);
Stepper stepper_right(MOT2_STEP, MOT2_DIR);
StepControl controller;



float absc=0;
float ord=0;
float teta_0=0;
void rotate(float angle);
void translate(float distance);

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
void tour(float arete);

void setup() {
    gpios.init();
    gpios.setMode(Gpios::LED, OUTPUT);
    Serial.begin(115200);
    delay(500);

    // setup left stepper
    stepper_left.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);
    gpios.write(Gpios::MOT1_ENABLE, LOW);      //enable motor

    // setup right stepper
    stepper_right.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);
   
    gpios.write(Gpios::MOT2_ENABLE, LOW);     //enable motor

    //addCommand({ROTATE, M_PI/2});
    //addCommand({TRANSLATE, 400});
    //addCommand({ROTATE, M_PI/2});
    carre(400);
   
    
}

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

uint32_t last_blink=0;


void carre(float arete){
    for (int i = 0; i < 4; ++i){
    addCommand({TRANSLATE, arete});
    addCommand({ROTATE, M_PI/2});
   }
    
}
void tour(float rayon){
    stepper_right.setTargetRel(2*M_PI*(RAYON_PAMI+rayon)* STEP_PER_MM);
    stepper_left.setTargetRel(2*M_PI*(rayon-RAYON_PAMI)* STEP_PER_MM);  
}

void loop(){
    if(millis() - last_blink > 200) {
        digitalToggle(LED_BUILTIN);
        last_blink = millis();
    }
    
    update_commands();

    
}



