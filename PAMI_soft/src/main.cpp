#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include "Gpios.h"

#define STEP_PER_MM 10.186
#define STEP_TO_DEGREES 10 // chercher 
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000

#define E 1
uint32_t status_time = 0;

Stepper stepper_left(MOT1_STEP, MOT1_DIR);
Stepper stepper_right(MOT2_STEP, MOT2_DIR);
StepControl controller;


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
    gpios.write(Gpios::MOT1_ENABLE, LOW);       //enable motor

    // setup right stepper
    stepper_right.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);
   
    gpios.write(Gpios::MOT2_ENABLE, LOW);       //enable motor

}
void straight_line(int dist) // dist in milimeters
{
    if (controller.isRunning()){return;}

    stepper_left.setTargetRel(dist*STEP_PER_MM);
    stepper_right.setTargetRel(-dist*STEP_PER_MM);
    controller.moveAsync(stepper_left,stepper_right);

}
void turn(float angle) // angle in degrees, direction true for trigonometric
{  
    if (controller.isRunning()){return;}

    stepper_left.setTargetRel(angle*STEP_TO_DEGREES);
    stepper_right.setTargetRel(-angle*STEP_TO_DEGREES);    
    controller.moveAsync(stepper_left,stepper_right);
}


void loop() {

    straight_line(10);

}
