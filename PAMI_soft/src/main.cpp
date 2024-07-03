#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Wire.h>
#include "vision.h"
#include <Servo.h>

#define LEVIER PA6
#define PHOTORESISTOR PA4
#define MIN_U_BATTERY 12
static int photoresistor_lim = 700;



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
coord depart[2] = {
  {1500 - 225,175,M_PI/2}, 
  {1500 + 225,175,M_PI/2}
  };
#define NB_POINTS 3
coord list_point[2][NB_POINTS] =  {
  {{1500-225,275,0}, {500, 275,0},{0,550,0}}, 
  {{1500 + 225, 275,0}, {2500, 275, 0},{3000,550,0}}
};
#elif defined(MILO)
coord depart[2] = {
  {1500 - 75, 175, M_PI/2},
  {1500 + 75, 175, M_PI/2}};
#define NB_POINTS 3
coord list_point[2][NB_POINTS] =  {{{1500 - 75, 275, 0},{1500-737,275,0}, {1500-737, 175,0}}, 
                                  {{1500 + 75, 275,0},{1500+737,275,0}, {1500+737, 175, 0}}};
#elif defined(MONA)
coord depart[2] = { 
  {1500 - 400,175,M_PI/2}, 
  {1500 + 400,175,M_PI/2}};
#define NB_POINTS 5
coord list_point[2][NB_POINTS] = {{{1500 - 400, 275, 0},{700,550,0}, {700, 2000-550,0},{400,1700,0},{100,2000,0}}, 
                                  {{1500 + 400, 275,0},{2300,550,0}, {2300, 2000-550, 0},{2600,1700,0},{2900,2000,0}}};
#else
#error "Le PAMI n'est pas défini! (MIA, PAMI1, PAMI2)"
#endif


EtatRobot etat_robot = EtatRobot::RECEPTION_FINISHED;



Base_roulante base_roulante;
Vision vision;

uint32_t last_blink=0;
uint32_t last_odo=0;
uint32_t last_sensor = 0;
uint32_t last_led_intensity;
uint32_t debug = 0;


uint32_t blink_period = 2000;
int current_led_intensity = 0;
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

int team; // 1 = blue ; 2 = yellow

bool dernier_point = false;


int current_levier_position;
int moyenne_led_intensity = 0;
int moyenne_place_intensity = 0;

uint16_t index_point = 0;


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
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PHOTORESISTOR, INPUT);

  


  base_roulante.init();

  current_levier_position = digitalRead(LEVIER);
 
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
  static int nb_evitement = 0; //nb d'évitement utlisiser avant d'atteindre le point suivant

  if (etat_robot != RECEPTION_FINISHED && (millis() - time_game) > timeout){
    digitalWrite(MOT_ENABLE, HIGH);
  }

  if (((distance_left < DISTANCE_EVITEMENT || distance_right < DISTANCE_EVITEMENT) && etat_robot != OBSTACLE_TOURNER)&& etat_robot != RECEPTION_FINISHED && etat_robot != RECEPTION_FINISHED && etat_robot != ETAT_FIN && !dernier_point){
    etat_robot = OBSTACLE_TOURNER;
    base_roulante.stop();
    substate = 0;
  }


    Serial.printf("%d \n", index_point);

  switch (etat_robot) {

      //on leve moyeu un peu

    case RECEPTION_FINISHED:
      //si photoresistor allume commence
      //Serial.printf("current_led_intensity: %d \n", current_led_intensity);
      //Serial.printf("photoresistor_lim : %d", photoresistor_lim);
      if(current_led_intensity > photoresistor_lim) {
        team = !digitalRead(LEVIER);
        base_roulante.set_current_position(depart[team]);
        //Serial.println("TOURNER");
        digitalWrite(MOT_ENABLE, LOW);
        etat_robot = TOURNER;
        substate = 0;
        time_game = millis();
        #if defined(MILO)
            delay(4000);
        #elif defined(MAMAMIA)
            delay(1000);
        #endif
      }
      break;
    case TOURNER:
      if (base_roulante.commands_finished()) {
        if(substate == 0) {
          // tourner vers le point
          float x = list_point[team][index_point].x;
          float y = list_point[team][index_point].y;
          base_roulante.rotate_point(x, y);
          substate = 1;
        }
        else if(substate == 1) {
          //Serial.println("AVANCER");
          etat_robot = AVANCER;
          substate = 0;
        }
      }
      break;
    case AVANCER:
      if (base_roulante.commands_finished()) {
        if(substate == 0) {
          // avancer vers le point
          float x = list_point[team][index_point].x;
          float y = list_point[team][index_point].y;
          base_roulante.translate_point(x, y);
          substate = 1;
        }
        else if(substate == 1) {
          if(index_point >= (NB_POINTS - 1)) {
            // arrivé
            //Serial.println("ETAT_FIN");
            
            #if defined(MILO) or defined(MAMAMIA)
            deploie_godet();
            #endif

            etat_robot = ETAT_FIN;
            substate = 0;
          } else if (index_point >= (NB_POINTS - 2)){
             // point suivant
            nb_evitement = 0;
            index_point ++;
            //Serial.println("TOURNER");
            etat_robot = TOURNER;
            substate = 0;
            dernier_point = true;
          }else{
            // point suivant
            nb_evitement = 0;
            index_point ++;
            //Serial.println("TOURNER");
            etat_robot = TOURNER;
            substate = 0;
          }
        }
      }
      break;
    case OBSTACLE_TOURNER:
      if (base_roulante.commands_finished()){
        if (nb_evitement > MAX_EVITEMENT && index_point < NB_POINTS-1 ){
          index_point ++;
          nb_evitement = 0;
          etat_robot = TOURNER;
          substate = 0;

        }else{
          if (substate == 0){
            nb_evitement ++; // on vient de faire un evitement
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
              //signe en fonction de ou l'on sera sur la table (ou vers la cible en fonction du temps que ca prend)
              base_roulante.addCommand({ROTATE,-M_PI/4});
            }
              substate = 1;
            }else if (substate == 1){
            if (distance_left > DISTANCE_EVITEMENT && distance_right > DISTANCE_EVITEMENT){
              etat_robot = OBSTACLE_AVANCER;
            }
            substate = 0;
          }
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
        digitalWrite(MOT_ENABLE, HIGH);
        //base_roulante.addCommand({ROTATE, M_PI});
        substate = 1;
        }
      break;
  }
}





void loop(){
  
  if(millis() - last_blink > blink_period) {
    digitalToggle(LED_BUILTIN);
    last_blink = millis();
    battery_checking();
    
  }

  if(millis() - last_odo > 50) {
    base_roulante.odometry();
    last_odo = millis();
    run_comportement();
  }

  if(millis() - last_sensor > 50) {
    distance_right = vision.get_dist_right();
    distance_left = vision.get_dist_left();
    distance_middle = vision.get_dist_right();
    //Serial.printf("G: %d, M: %d, R: %d \n", distance_left, distance_middle, distance_right);
    //Serial.printf("%d \n", analogRead(PHOTORESISTOR));
    //Serial.printf("G: %d, R: %d \n", distance_left, distance_right);
    //run_comportement();
    last_sensor = millis();
  }

  if (millis() - last_led_intensity > 50){
     current_led_intensity = analogRead(PHOTORESISTOR);
     //Serial.printf("valeur : %d \n", current_led_intensity);
     last_led_intensity = millis();
  }

  base_roulante.update_commands();


}

