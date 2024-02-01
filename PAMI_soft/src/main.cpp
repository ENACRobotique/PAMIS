#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Wire.h>
#include <VL53L0X.h>

//define IO PB04
#define MOT_ENABLE PA12
VL53L0X sensor;


//I2C1_SDA PB07
//I2C1_SCL PB06


void setup() {
    //gpios.init();
    //gpios.setMode(Gpios::LED, OUTPUT);
    Serial.begin(115200);
    delay(500);
    Serial.println("coucou ");

    pinMode(MOT_ENABLE, OUTPUT);
    digitalWrite(MOT_ENABLE, LOW);

    // setup left stepper
    stepper_left.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);
    //gpios.write(Gpios::MOT1_ENABLE, LOW);      //enable motor

    // setup right stepper
    stepper_right.setAcceleration(STEPPER_MAX_ACC)
                .setMaxSpeed(STEPPER_MAX_SPEED/2)
                .setPullInSpeed(10)
                .setInverseRotation(true);

    //gpios.write(Gpios::MOT2_ENABLE, LOW);     //enable motor


    Wire.begin();        // join i2c bus (address optional for master)
    
    pinMode(LED_BUILTIN, OUTPUT);

    sensor.init();
    sensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
    sensor.startContinuous();


}


Base_roulante base_roulante;
uint32_t last_blink=0;



void loop(){
    if(millis() - last_blink > 200) {
        digitalToggle(LED_BUILTIN);
        last_blink = millis();
    }
    //base_roulante.move(10,10);
    delay(50);
    base_roulante.update_commands();
    //stepper_left.setPosition(500);
    //Serial.println(stepper_left.getPosition());



    base_roulante.addCommand({TRANSLATE,100});
    delay(2000);   


    int distance =sensor.readRangeContinuousMillimeters();
    Serial.print("Distance: ");
    Serial.print(distance);
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();

    delay(500);
}


