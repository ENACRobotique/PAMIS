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

#define XSHUT_SENSOR2 PB0
#define XSHUT_SENSOR1 PB1

VL53L0X sensor1;
VL53L0X snensor2;


Base_roulante base_roulante;
uint32_t last_blink=0;

void setup() {

  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, HIGH);
  digitalWrite(XSHUT_SENSOR2, LOW);
  

  delay(1);

  Wire.begin();        // join i2c bus (address optional for master)

  sensor1.init();
  sensor1.setTimeout(500);

  sensor1.setAddress(0x30);

// Start continuous back-to-back mode (take readings as
// fast as possible).  To use continuous timed mode
// instead, provide a desired inter-measurement period in
// ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous();
  
  digitalWrite(XSHUT_SENSOR2, HIGH);
  delay(1);
  snensor2.init();
  snensor2.setTimeout(500);
  ///snensor2.setAddress(0x30);
  snensor2.startContinuous();



  Serial.begin(115200);
  delay(500);

  pinMode(MOT_ENABLE, OUTPUT);
  digitalWrite(MOT_ENABLE, LOW);

  // setup left stepper
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED/2)
              .setPullInSpeed(10)
              .setInverseRotation(true);
  

  // setup right stepper
  stepper_right.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED/2)
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  
  pinMode(LED_BUILTIN, OUTPUT);

  
  // base_roulante.move(250,0);
  // base_roulante.move(250,380);
  // base_roulante.move(950,380);
  // base_roulante.move(950,-600);
  // base_roulante.move(250,0);
  // base_roulante.move(0,0);

}






void loop(){
    if(millis() - last_blink > 200) {
        digitalToggle(LED_BUILTIN);
        last_blink = millis();
    }
     delay(50);




    

    base_roulante.update_commands();

    int distance1 =sensor1.readRangeContinuousMillimeters();
    int distance2 =snensor2.readRangeContinuousMillimeters();
    Serial.printf("Distance: %d,  %d\n", distance1, distance2);
    
    
    
    if (sensor1.timeoutOccurred() || snensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  delay(100);
}


