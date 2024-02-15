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

#define DISTANCE_EVITEMENT 200

VL53L0X sensor_right;
VL53L0X sensor_left;


Base_roulante base_roulante;
uint32_t last_blink=0;
uint32_t last_odo=0;
uint32_t last_sensor = 0;
uint32_t debug = 0;


int distance_right;
int distance_left;


EtatRobot etat_robot = EtatRobot::ATTENTE_DEBUT;


#define NB_POINTS 3
coord list_point[NB_POINTS] = {{300,-300,0},{1300,-300,0},{1300,700,0}};
uint16_t index_point = 0;

void setup() {

  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, HIGH);
  digitalWrite(XSHUT_SENSOR2, LOW);
  

  delay(1); //pour que le sensor soit reveille

  Wire.begin();        // join i2c bus (address optional for master)

  sensor_right.init();
  sensor_right.setTimeout(500);
  sensor_right.setAddress(0x30);
  sensor_right.startContinuous();
  
  digitalWrite(XSHUT_SENSOR2, HIGH);
  delay(1); //pour que le sensor soit reveille
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

}





void run_comportement (){
  static int substate = 0;

  if ((distance_left < DISTANCE_EVITEMENT || distance_right < DISTANCE_EVITEMENT) && etat_robot != OBSTACLE_TOURNER){
    etat_robot = OBSTACLE_TOURNER;
    base_roulante.stop();
    substate = 0;
  }

  switch (etat_robot) {
    case ATTENTE_DEBUT:
      //si photoresistor allume avance
      if(millis() > 2000) {
        Serial.println("TOURNER");
        etat_robot = TOURNER;
        substate = 0;
      }
      break;
    case TOURNER:
      if (base_roulante.commands_finished()) {
        if(substate == 0) {
          // tourner vers le point
          float x = list_point[index_point].x;
          float y = list_point[index_point].y;
          base_roulante.rotate_point(x, y);
          substate = 1;
        }
        else if(substate == 1) {
          Serial.println("AVANCER");
          etat_robot = AVANCER;
          substate = 0;
        }
      }
      break;
    case AVANCER:
      if (base_roulante.commands_finished()) {
        if(substate == 0) {
          // avancer vers le point
          float x = list_point[index_point].x;
          float y = list_point[index_point].y;
          base_roulante.translate_point(x, y);
          substate = 1;
        }
        else if(substate == 1) {
          if(index_point >= (NB_POINTS - 1)) {
            // arrivé
            Serial.println("ETAT_FIN");
            etat_robot = ETAT_FIN;
            substate = 0;
          } else {
            // point suivant
            index_point += 1;
            Serial.println("TOURNER");
            etat_robot = TOURNER;
            substate = 0;
          }
        }
      }
      break;
    case OBSTACLE_TOURNER:
      if (base_roulante.commands_finished()){
        if (substate == 0){
          if (distance_left < DISTANCE_EVITEMENT && distance_right < DISTANCE_EVITEMENT){
              base_roulante.addCommand({ROTATE,-M_PI/6}); //peut etre a changer plus tard
          }else if (distance_left < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,-M_PI/6});
          }else{
            base_roulante.addCommand({ROTATE,M_PI/6});
          }
          substate = 1;
        }else if (substate == 1){
          if (distance_left > DISTANCE_EVITEMENT && distance_right > DISTANCE_EVITEMENT){
            etat_robot = OBSTACLE_AVANCER;
          }
          substate = 0;
        }
      }
      break;
    case OBSTACLE_AVANCER:
      if (base_roulante.commands_finished()){
        if (substate == 0){
          base_roulante.addCommand({TRANSLATE, 200});
          substate =1;
        }else if (substate == 1){
            etat_robot = TOURNER;
            substate = 0;
        }
      }  
      break;
    case ETAT_FIN:
      if (substate == 0){
        base_roulante.addCommand({ROTATE, M_PI});
        substate = 1;
        }
      break;
  }
}




void loop(){
  if(millis() - last_blink > 200) {
    digitalToggle(LED_BUILTIN);
    last_blink = millis();
  }

  if(millis() - last_odo > 50) {
    base_roulante.odometry();
    last_odo = millis();
  }

  if(millis() - last_sensor > 50) {
    distance_right =sensor_right.readRangeContinuousMillimeters();
    distance_left =sensor_left.readRangeContinuousMillimeters();
    run_comportement();
    last_sensor = millis();
  }

  if(millis() - debug > 200){
    coord current_coord =base_roulante.get_current_position();
    Serial.print(current_coord.x);
    Serial.print("  ");
    Serial.print(current_coord.y);
    Serial.print("  ");
    Serial.println(current_coord.theta);
    debug = millis();
  }

  base_roulante.update_commands();

}

//lire tension de batterie et faire clignoter beaucoups led quand plus de batterie  
//arreter moteur quand y'a plus de batterie et arreter moteur quand pas besoin de rouler (au debut et a la fin)
