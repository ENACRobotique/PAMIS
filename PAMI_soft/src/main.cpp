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

VL53L0X sensor_right;
VL53L0X sensor_left;


Base_roulante base_roulante;
uint32_t last_blink=0;
uint32_t last_odo=0;

typedef struct{
  float x;
  float y;
  float theta;
} coord;

coord current_coord = {0,0,0};


void setup() {

  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, HIGH);
  digitalWrite(XSHUT_SENSOR2, LOW);
  

  delay(1);

  Wire.begin();        // join i2c bus (address optional for master)

  sensor_right.init();
  sensor_right.setTimeout(500);

  sensor_right.setAddress(0x30);

// Start continuous back-to-back mode (take readings as
// fast as possible).  To use continuous timed mode
// instead, provide a desired inter-measurement period in
// ms (e.g. sensor.startContinuous(100)).
  sensor_right.startContinuous();
  
  digitalWrite(XSHUT_SENSOR2, HIGH);
  delay(1);
  sensor_left.init();
  sensor_left.setTimeout(500);
  sensor_left.startContinuous();

  Serial.begin(115200);

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

  coord point0 = {0,0,0};
  coord point1 = {10000,0,0};
  coord list_point[2] = {point0,point1};
  //base_roulante.move(10000,0);
  // base_roulante.move(250,380);
  // base_roulante.move(950,380);
  // base_roulante.move(950,-600);
  // base_roulante.move(250,0);
  // base_roulante.move(0,0);
}


void odometry (){
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

uint32_t debug = 0;

void loop(){
  if(millis() - last_blink > 200) {
    digitalToggle(LED_BUILTIN);
    last_blink = millis();
  }

  if(millis() - last_odo > 50) {
    odometry();
    last_odo = millis();
  }

  
  

  int distance_right =sensor_right.readRangeContinuousMillimeters();
  int distance_left =sensor_left.readRangeContinuousMillimeters();

  if(millis() - debug > 200){
    // Serial.printf("etat de l'évitement : %d \n", base_roulante.etat_evitement_temp);
    // Serial.printf("Distance: %d,  %d\n", distance_right, distance_left);
    // if (sensor_right.timeoutOccurred() || sensor_left.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print(current_coord.x);
    Serial.print("  ");
    Serial.print(current_coord.y);
    Serial.print("  ");
    Serial.println(current_coord.theta);
    debug = millis();

  }

  //mettre l'evitement TOUJOURS avant l'update commande
  base_roulante.evitement(distance_right,distance_left);
  base_roulante.update_commands();

}


