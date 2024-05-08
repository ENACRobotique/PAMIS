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

#if defined(MIA)
VL53L0X sensor_right;
VL53L0X sensor_left;
VL53L0X sensor_middle;
#elif defined(MONA)
VL53L0X sensor_middle;
VL53L1X sensor_left;
VL53L1X sensor_right;
#elif defined(MILO)
VL53L0X sensor_middle;
VL53L0X sensor_left;
VL53L1X sensor_right;
#else
#error "Le PAMI n'est pas défini! (MIA, PAMI1, PAMI2)"
#endif


EtatRobot etat_robot = EtatRobot::RECEPTION_FINISHED;
;


Base_roulante base_roulante;

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




#define MAX_EVITEMENT 3
#define NB_POINTS 3




/*
      x
      ^
      |
      |
      |
y<-----

*/

//coord list_point[NB_POINTS];
int team; // 1 = blue ; 2 = yellow
coord depart;
coord list_point[2][NB_POINTS] = {{{1375,175,0},{450,1550,0},{200,1800,0}}, {{1875,175,0},{2550,1550,0},{2800,1800,0}}};
bool dernier_point = false;


int current_levier_position;
int moyenne_led_intensity = 0;
int moyenne_place_intensity = 0;

uint16_t index_point = 0;


void setup() {
  pinMode(BUZZER,OUTPUT);
  pinMode(LEVIER, OUTPUT);
  digitalWrite(LEVIER,HIGH);


  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, LOW);
  digitalWrite(XSHUT_SENSOR2, LOW);
  

 //godet.attach(GODET,0,170,30);



  delay(100); //pour que le sensor soit reveille

  Wire.begin();        // join i2c bus (address optional for master)
  bool is_ok = sensor_middle.init();
  if (is_ok){
    sensor_middle.setTimeout(500);
    sensor_middle.setAddress(0x40);
    sensor_middle.startContinuous(50);
  }else{
    sensor_middle.setAddress(0x40);
    // uint8_t readreg = sensor_middle.readReg(VL53L0X::IDENTIFICATION_MODEL_ID);
    // if (readreg != 0xEE) { 
    //   blink_period = 100;
    //   }
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
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED/2)
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  // // setup right stepper
  stepper_right.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED/2)
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PHOTORESISTOR, INPUT);

  //   if  (digitalRead(LEVIER)){
  //   depart = {1875,75,M_PI/2};
  //   team = 1;
  // } else{
  //   depart = {1375,75,M_PI/2};
  //   team = 0;
  // }


  base_roulante.init(depart);
  //base_roulante.rotate(M_PI*20);

  current_levier_position = digitalRead(LEVIER);
  Serial.printf("DEBUT");
  Serial.print(1);
  while (digitalRead(LEVIER) == current_levier_position){

  }
  current_levier_position = digitalRead(LEVIER);
  Serial.printf("RECEPTION_LED_INTENSITY \n");
  delay(1000);
  //on prend la moyenne de l'intensite sur 4 secondes
  for (int i=0; i<50; i++){
    moyenne_led_intensity += analogRead(PHOTORESISTOR);
    Serial.printf("moy = %d \n", moyenne_led_intensity);
    delay(30);
  }
  moyenne_led_intensity = moyenne_led_intensity/50;
  //on leve le godet un peu
  Serial.printf("RECEPTION_LED_FINISHED \n");
  while (digitalRead(LEVIER) == current_levier_position){

  }
  current_levier_position = digitalRead(LEVIER);
  Serial.printf("RECEPTION_PLACE_INTENSITY \n");
  delay(1000);
  //on prend la moyenne de l'intensite sur 4 secondes
   for (int i=0; i<50; i++){
    moyenne_place_intensity += analogRead(PHOTORESISTOR);
    delay(30);
  }
  moyenne_place_intensity = moyenne_led_intensity / 50;
  //apres le temps on calcul somme des deux intensité sur 2 et on le met à la place de PHOTORESISTOR_INTENSITY_DEM
  photoresistor_lim = (moyenne_led_intensity + moyenne_place_intensity)/2;

  //lever godet
  delay(1000);
}




void battery_checking(){
  lvl=analogRead(PA7);
  u_battery=lvl*3.3/(1023.*F_BAT);
  //Serial.println(u_battery);
  if (u_battery<MIN_U_BATTERY){
     digitalWrite(BUZZER,HIGH);
     digitalWrite(MOT_ENABLE, HIGH);
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



  switch (etat_robot) {


      //on leve moyeu un peu

    case RECEPTION_FINISHED:
      //on choisit l'équipe

      //si photoresistor allume commence
      Serial.printf("current_led_intensity: %d \n", current_led_intensity);
      Serial.printf("photoresistor_lim : %d", photoresistor_lim);
      if(current_led_intensity > photoresistor_lim) {

        
        //Serial.println("TOURNER");
        digitalWrite(MOT_ENABLE, LOW);
        etat_robot = TOURNER;
        substate = 0;
        time_game = millis();
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

void deploie_godet(){
  godet.write(170);
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
  }

  if(millis() - last_sensor > 50) {
    distance_right =sensor_right.readRangeContinuousMillimeters();
    distance_left =sensor_left.readRangeContinuousMillimeters();
    distance_middle = sensor_middle.readRangeContinuousMillimeters();
    Serial.printf("G: %d, M: %d, R: %d \n", distance_left, distance_middle, distance_right);
    Serial.printf("%d \n", analogRead(PHOTORESISTOR));
    //Serial.printf("G: %d, R: %d \n", distance_left, distance_right);
    run_comportement();
    last_sensor = millis();
  }

  if (millis() - last_led_intensity > 50){
     current_led_intensity = analogRead(PHOTORESISTOR);
     //Serial.printf("valeur : %d \n", current_led_intensity);
     last_led_intensity = millis();
  }

  base_roulante.update_commands();

  // if(millis() - debug > 200){
  //   coord current_coord =base_roulante.get_current_position();
    
  //   Serial.printf("la photoresistor : %d \n", current_led_intensity);
  //   Serial.print(current_coord.x);
  //   Serial.print("  ");
  //   Serial.print(current_coord.y);
  //   Serial.print("  ");
  //   Serial.println(current_coord.theta);
  //   debug = millis();
  // }



  
   

}

//lire tension de batterie et faire clignoter beaucoups led quand plus de batterie  
//arreter moteur quand y'a plus de batterie et arreter moteur quand pas besoin de rouler (au debut et a la fin)
