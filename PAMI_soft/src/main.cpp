#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include "Gpios.h"
#include "pilot.h"



#define E 1
uint32_t status_time = 0;
Stepper left_step(MOT1_STEP, MOT1_DIR);
Stepper right_step(MOT2_STEP, MOT2_DIR);
pilot base_roulante(&left_step,Gpios::MOT1_ENABLE,
                    &right_step,Gpios::MOT2_ENABLE);



void setup() {
    gpios.init();
    gpios.setMode(Gpios::LED, OUTPUT);
    Serial.begin(115200);
    delay(500);

    base_roulante.init();

}


void loop() {


}
