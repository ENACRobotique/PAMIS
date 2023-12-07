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
StepControl controller;float absc=0;
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
void carre(float arete);
void tour(float rayon);

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


    
}




uint32_t last_blink=0;


void carre(float arete){
    for (int i = 0; i < 4; ++i){
    addCommand({TRANSLATE, arete});
    addCommand({ROTATE, M_PI/2});
   }
    
}


void loop(){
    if(millis() - last_blink > 200) {
        digitalToggle(LED_BUILTIN);
        last_blink = millis();
    }
   
    delay(500);
    
}



