#include <Arduino.h>
#include "config.h"
#include <AccelStepper.h>


AccelStepper stepper1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2, DIR2);

#define LEN_POS_OBJ 2
long pos_obj1[LEN_POS_OBJ] = {400, 0};
long pos_obj2[LEN_POS_OBJ] = {-400, 0};


void setup() {
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  digitalWrite(ENABLE1, LOW);
  digitalWrite(ENABLE2, LOW);

  Serial.begin(115200);

  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(10000.0);
  
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(10000.0);

  stepper1.moveTo(pos_obj1[0]);
  stepper2.moveTo(pos_obj2[0]);

}



void loop() {
  static uint32_t time_blink = 0;
  static int obji = 0;

  if(millis()-time_blink > 250) {
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
    time_blink = millis();
  }


  if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
    obji = (obji + 1) % LEN_POS_OBJ;
    stepper1.moveTo(pos_obj1[obji]);
    stepper2.moveTo(pos_obj2[obji]);
    
    Serial.print(pos_obj1[obji]);
    Serial.print(" ");
    Serial.println(pos_obj2[obji]);
  }

  stepper1.run();
  stepper2.run();

}

