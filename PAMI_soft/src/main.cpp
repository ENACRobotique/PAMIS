#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"

#define LEVIER PA6
#define MIN_U_BATTERY 12

#define MOT_ENABLE PA12


coord depart =  {0,0,0}; 
#define NB_POINTS 4
coord list_point[NB_POINTS] = {{100, 0, 0},{750,800,0},{1200, 800,0},{1600,950,0}};


EtatRobot etat_robot = EtatRobot::RECEPTION_FINISHED;
Base_roulante base_roulante;
uint32_t last_odo=0;
int mot_allumer = 0;

uint32_t last_blink=0;
uint32_t blink_period = 2000;

uint32_t debug = 0;





uint16_t index_point = 0;



void setup() {
  pinMode(LEVIER, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  pinMode(MOT_ENABLE, OUTPUT);
  // setup left stepper
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              .setPullInSpeed(10)
              .setInverseRotation(false);
  // setup right stepper
  stepper_right.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              .setPullInSpeed(10)
              .setInverseRotation(false);

   base_roulante.init();

   int current_levier_position = digitalRead(LEVIER);
   Serial.printf("DEBUT");

   while (digitalRead(LEVIER) == current_levier_position){

  }  
}







void run_comportement (){

  switch (etat_robot) {
    case RECEPTION_FINISHED:
        base_roulante.set_current_position(depart);
        Serial.println("RECEPT");
        digitalWrite(MOT_ENABLE, LOW);
        etat_robot = TOURNER;
        mot_allumer = 1;
      break;

    case TOURNER:
      if (base_roulante.commands_finished()) {
        // tourner vers le point
        Serial.println("TOURNER");
        float x = list_point[index_point].x;
        float y = list_point[index_point].y;
        base_roulante.rotate_point(x, y);
        etat_robot = AVANCER;
      }
      break;
    case AVANCER:
      if (base_roulante.commands_finished()) {
        // avancer vers le point
        Serial.println("AVANCER");
        float x = list_point[index_point].x;
        float y = list_point[index_point].y;
        base_roulante.translate_point(x, y);
        if(index_point >= (NB_POINTS - 1)) {
          etat_robot = ETAT_FIN;
        }else{
          // point suivant
          index_point ++;
          etat_robot = TOURNER;;
          }
        }
      break;
   
    case ETAT_FIN:
      if (base_roulante.commands_finished() && mot_allumer){
        Serial.println("FIN");
        digitalWrite(MOT_ENABLE, HIGH);
        mot_allumer = 0;
        }
      break;
  }
}





void loop(){
  
  if(millis() - last_blink > blink_period) {
    digitalToggle(LED_BUILTIN);
    last_blink = millis();  
  }

  if(millis() - last_odo > 50) {
    base_roulante.odometry();
    last_odo = millis();
    run_comportement();
  }


  base_roulante.update_commands();

}

