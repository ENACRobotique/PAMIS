#include <Arduino.h>
#include "TeensyStep.h"
#include "PinLayout.h"
#include <math.h>
#include <stdlib.h>
#include "base_roulante.h"
#include <Wire.h>
#include "vision.h"
#include <Servo.h>

#include <STM32FreeRTOS.h>
#include <task.h>

#define LEVIER PA6
#define PHOTORESISTOR PA4
#define MIN_U_BATTERY 12

// #define GODET PA3 //PA0
// Servo godet;

//define IO PB04
#define MOT_ENABLE PA12

#define XSHUT_SENSOR1 PB0
#define XSHUT_SENSOR2 PB1
#define BUZZER PA0 //PA3
#define BATTERYLVL PA7
constexpr float F_BAT= 0.14838709677419354;

uint32_t timeout = 10000;
uint32_t time_game = 0;



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

TaskHandle_t TaskAvancer;
TaskHandle_t TaskTourner;
TaskHandle_t TaskVoirObsable;
float dist = 1000.0;
float angle = M_PI / 2;
bool isAdvancing = false;
bool isTurning = false;

CmdType prochaine_commande = TRANSLATE;

void vTask_Avancer(void* param){
  base_roulante.translate(*(float*)param);
  Serial.println("Avancer");
  vTaskDelay(pdMS_TO_TICKS(4000)); // Délai pour avancer
  //prochaine_commande = ROTATE;
  vTaskDelete(NULL); // Supprimer la tâche après l'exécution
}

void vTask_Tourner(void* param){
  base_roulante.rotate(*(float*)param);
  Serial.println("Tourner");
  vTaskDelay(pdMS_TO_TICKS(4000)); // Délai pour tourner
  prochaine_commande = TRANSLATE;
  vTaskDelete(NULL); // Supprimer la tâche après l'exécution
}

void vTask_VoirObstacle (void* param){
  for (;;){
    Serial.println("Voir");
    distance_right = vision.get_dist_right();
    distance_left = vision.get_dist_left();
    distance_middle = vision.get_dist_right();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() {
  pinMode(BUZZER,OUTPUT);
  pinMode(LEVIER, OUTPUT);
  digitalWrite(LEVIER,HIGH);

  pinMode(XSHUT_SENSOR1, OUTPUT);
  pinMode(XSHUT_SENSOR2, OUTPUT);
  digitalWrite(XSHUT_SENSOR1, LOW);
  digitalWrite(XSHUT_SENSOR2, LOW);
  
  delay(100); //pour que le sensor soit reveille

  Wire.begin(); // join i2c bus (address optional for master)
  bool is_ok = sensor_middle.init();
  if (is_ok){
    sensor_middle.setTimeout(500);
    sensor_middle.setAddress(0x40);
    sensor_middle.startContinuous(50);
  } else {
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
  stepper_left.setAcceleration(STEPPER_MAX_ACC)
              .setMaxSpeed(STEPPER_MAX_SPEED)
              .setPullInSpeed(10)
              .setInverseRotation(true);
  
  // setup right stepper
  stepper_right.setAcceleration(STEPPER_MAX_ACC)
               .setMaxSpeed(STEPPER_MAX_SPEED)
               .setPullInSpeed(10)
               .setInverseRotation(true);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PHOTORESISTOR, INPUT);
  
  base_roulante.init();
  digitalWrite(MOT_ENABLE, LOW);

  current_levier_position = digitalRead(LEVIER);

  vTaskStartScheduler();


  xTaskCreate(vTask_VoirObstacle, "VoirTask", 1000, NULL, 1, &TaskVoirObsable);
}

void battery_checking(){
  lvl = analogRead(PA7);
  u_battery = lvl * 3.3 / (1023. * F_BAT);
  if (u_battery < MIN_U_BATTERY){
     digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void loop() {
  if (millis() - last_blink > blink_period) {
    digitalToggle(LED_BUILTIN);
    last_blink = millis();
    battery_checking();   
  }

  if (millis() - last_odo > 50) {
    base_roulante.odometry();

    if (distance_left < 150 || distance_middle < 150 || distance_right < 150){
      Serial.printf("stop");
      //prochaine_commande = AUCUN;
    }
  }



  if (prochaine_commande == TRANSLATE) {
    xTaskCreate(vTask_Avancer, "AvancerTask", 1000, &dist, 2, &TaskAvancer);
  }

  if (prochaine_commande == ROTATE) {
    xTaskCreate(vTask_Tourner, "TournerTask", 1000, &angle, 1, &TaskTourner);
  }


}
