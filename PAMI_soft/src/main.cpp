#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Servo.h>

#defin E
#define LEVIER PA6
#define PHOTORESISTOR PA4
#define MIN_U_BATTERY 12
int photoresistor_lim;



#define GODET PA3 //PA0
Servo godet;

//define IO PB04
#define MOT_ENABLE PA12



#define XSHUT_SENSOR1 PB0
#define XSHUT_SENSOR2 PB1
#define BUZZER PA0 //PA3
#define BATTERYLVL PA7
constexpr float F_BAT= 0.14838709677419354;

uint32_t timeout = 10000;
uint32_t time_game = 0;

#define MAX_EVITEMENT 3


#if defined(MAMAMIA)
VL53L0X sensor_right;
VL53L0X sensor_left;
VL53L0X sensor_middle;


#elif defined(MILO)
VL53L0X sensor_middle;
VL53L0X sensor_left;
VL53L1X sensor_right;

#elif defined(MONA)
VL53L0X sensor_middle;
VL53L1X sensor_left;
VL53L1X sensor_right;

#else
#error "Le PAMI n'est pas défini! (MIA, PAMI1, PAMI2)"
#endif


EtatRobot etat_robot = EtatRobot::AVANCER;



Base_roulante base_roulante;

uint32_t last_blink=0;
uint32_t last_sensor = 0;



uint32_t blink_period = 2000;
int distance_right;
int distance_left;
int distance_middle;
int lvl;
float u_battery;


/*
      x
      ^
      |
      |
      |
y<-----

*/





void deploie_godet(){
  #if defined (MILO)
  godet.writeMicroseconds(2800);
  #elif defined(MAMAMIA)
  godet.writeMicroseconds(2800);
  #endif
}

void setup() {
  pinMode(BUZZER,OUTPUT);
  pinMode(LEVIER, OUTPUT);
 digitalWrite(LEVIER,HIGH);


  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, LOW);
  digitalWrite(XSHUT_SENSOR2, LOW);
  
  #if defined(MAMAMIA)
  godet.attach(GODET);
  delay(500);
  godet.writeMicroseconds(2800);
  delay(500);
  #endif





  delay(100); //pour que le sensor soit reveille

  Wire.begin();        // join i2c bus (address optional for master)
  bool is_ok = sensor_middle.init();
  if (is_ok){
    sensor_middle.setTimeout(500);
    sensor_middle.setAddress(0x40);
    sensor_middle.startContinuous(50);
  }else{
    sensor_middle.setAddress(0x40);
  } 


  digitalWrite(XSHUT_SENSOR1, HIGH);
  delay(100);
  sensor_right.init();
  sensor_right.setTimeout(500);
  sensor_right.setAddress(0x50);
  sensor_right.startContinuous(50);
  
  digitalWrite(XSHUT_SENSOR2, HIGH);
  delay(100); //pour que le sensor soit reveille
  sensor_left.init();
  sensor_left.setTimeout(500);
  sensor_left.startContinuous(50);

  Serial.begin(115200);

  pinMode(MOT_ENABLE, OUTPUT);
  digitalWrite(MOT_ENABLE, HIGH);

  // setup left stepper
              #if defined(MONA)
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              #else
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              #endif
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  // // setup right stepper
              #if defined(MONA)
              stepper_right.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              #else
 stepper_right.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              #endif
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  digitalWrite(MOT_ENABLE, LOW);


  base_roulante.init();

  delay (500);

}




void battery_checking(){
  lvl=analogRead(PA7);
  u_battery=lvl*3.3/(1023.*F_BAT);
  //Serial.println(u_battery);
  if (u_battery<MIN_U_BATTERY){
     digitalWrite(BUZZER,HIGH);
     //digitalWrite(MOT_ENABLE, HIGH);
  }
  else{
    digitalWrite(BUZZER,LOW);
  }
}





void run_comportement (){
static int substate = 0;

  if ((distance_left < DISTANCE_EVITEMENT || distance_right < DISTANCE_EVITEMENT || distance_middle < DISTANCE_EVITEMENT) && etat_robot != OBSTACLE_TOURNER){
    etat_robot = OBSTACLE_TOURNER;
    base_roulante.stop();
  }

  switch (etat_robot) {

    case AVANCER:
      if (base_roulante.commands_finished()) {
          // avancer
          base_roulante.addCommand({TRANSLATE,10000000});;
          substate = 1;
        }
      break;

    case OBSTACLE_TOURNER:
      if (base_roulante.commands_finished()){
        if (substate == 0){
          if (distance_left < DISTANCE_EVITEMENT && distance_right < DISTANCE_EVITEMENT){
            //signe en fonction de ou l'on sera sur la table (ou vers la cible en fonction du temps que ca prend)
            base_roulante.addCommand({ROTATE,-M_PI/2});
          }else if (distance_left < DISTANCE_EVITEMENT && distance_middle < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,-M_PI/3});
          }else if (distance_right < DISTANCE_EVITEMENT && distance_middle < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,M_PI/3});
          }else if (distance_left < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,-M_PI/6});
          }else if (distance_right < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,-M_PI/6});
          }else if (distance_middle < DISTANCE_EVITEMENT){
            base_roulante.addCommand({ROTATE,-M_PI/4});
          }
            substate = 1;
          }else if (substate == 1){
          if (distance_left > DISTANCE_EVITEMENT && distance_middle && distance_right > DISTANCE_EVITEMENT){
            etat_robot = AVANCER;
          }
          substate = 0;
        }
      }
      break;
  }
}






void loop(){
  
  if(millis() - last_blink > blink_period) {
    last_blink = millis();
    battery_checking();
    
  }


  if(millis() - last_sensor > 50) {
    distance_right =sensor_right.readRangeContinuousMillimeters();
    distance_left =sensor_left.readRangeContinuousMillimeters();
    distance_middle = sensor_middle.readRangeContinuousMillimeters();
    //Serial.printf("G: %d, M: %d, R: %d \n", distance_left, distance_middle, distance_right);
    //Serial.printf("G: %d, R: %d \n", distance_left, distance_right);
    run_comportement();
    last_sensor = millis();
  }

  base_roulante.update_commands();

}


