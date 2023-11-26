#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include "Gpios.h"

#define STEP_PER_MM 10.186
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000

uint32_t status_time = 0;

Stepper stepper_left(MOT1_STEP, MOT1_DIR);
StepControl controller;

#define NB_TARGETS 2
float targets[NB_TARGETS] = {200, 0};
size_t target_idx = 0;
int32_t target_pos = 0;


void setup() {
    gpios.init();
    gpios.setMode(Gpios::LED, OUTPUT);
    Serial.begin(115200);
    delay(500);

    // setup stepper
    stepper_left.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);
    //gpios.setMode(Gpios::FDC1, INPUT_PULLUP);     // on statically configured TCA
    //gpios.setMode(Gpios::MOT1_ENABLE, OUTPUT);    // on statically configured TCA
    gpios.write(Gpios::MOT1_ENABLE, LOW);       //enable motor

    target_pos = 0;
    stepper_left.setTargetAbs(target_pos);
    controller.moveAsync(stepper_left);
}

void loop() {
    // toggle led and print stepper position
    if(millis() - status_time > 200) {
        float pos = stepper_left.getPosition() / STEP_PER_MM;
        Serial.println(pos);
        gpios.toggle(Gpios::LED);
        status_time = millis();
    }

    // si le stepper est proche de sa target, passer à la traget suivante.
    if(abs(target_pos - stepper_left.getPosition()) < 10 * STEP_PER_MM) {
        // delay(500);
        target_idx = (target_idx + 1) % NB_TARGETS;
        target_pos = targets[target_idx] * STEP_PER_MM;
        stepper_left.setTargetAbs(target_pos);
        controller.moveAsync(stepper_left);
    }
    
}
