#include <Arduino.h>
#include "config.h"
#include "VL53L1X_api.h"
#include <FreeRTOS.h>
#include "task.h"
#include "Wire.h"
#include "locomotion.h"
#include "radar.h"
#define DISTANCEEVITEMENT 200
coord startPos={2925,1625,M_PI};

// task qui fait clignoter une LED en continu.
static void blinker( void *arg ) {
  while(true) {
    digitalWrite(LED1, !digitalRead(LED1));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void test_avance( void *arg ) {
  while(true) {
    if(locomotion.translateBlocking(4000)) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    if(locomotion.translateBlocking(-4000)) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
  }
}

static void test_rotate(void * args) {
  locomotion.rotateBlocking(40*M_PI);
  while(true) {
    // if(locomotion.rotateBlocking(1000)) {
    //   while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
    //   }
    // }
  } 
}

static void test_moveB(void * args) {
  while(true) {
    //locomotion.moveBlocking({-2000,2000,0});
    if(locomotion.moveBlocking({1000,1000,0})){
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    if(locomotion.moveBlocking({1000,0,0})) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
  } 
}

  static void move(void * args) {
    coord target[1] = {{1150,1400,0}};
    while(true) {
      locomotion.move(target, 1);
    } 
  }


void odometry(void*){
  while (-1){
    locomotion.odometry();
    coord current_coord=locomotion.getPositon();
    // Serial.print("pos x : ");
    //Serial.println(current_coord.x);
    // Serial.print(", pos y : ");
    // Serial.print(current_coord.y);
    // Serial.print(", theta : ");

    // char buffer[15]; // Un buffer suffisamment grand pour contenir le nombre formaté
    // dtostrf(current_coord.theta, 13, 7, buffer); // Convertit le flottant en chaîne avec 7 chiffres après la virgule
    // Serial.println(buffer); // Affiche la chaîne formatée
    // Serial.printf("x: %f, y: %f, theta: %f \n",current_coord.x, current_coord.y, current_coord.theta);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}



static void radar_alert_cb() {
  digitalWrite(LED2, !digitalRead(LED2));

  bool doit = 
    locomotion.state==INIT ||
    locomotion.state==TOURNE ||
    locomotion.state==TOURNE_FINI ||
    locomotion.state==TOUDRWA ||
    locomotion.state==TOUDRWA_FINI ||
    locomotion.state==FINITOPIPO ||
    locomotion.state==AVOIDINGTOUDRWA ||
    locomotion.state==AVOIDINGTOUDRWAFINI ||
    locomotion.state==SUIVILIGNES25 ||
    locomotion.state==SUIVILIGNESFINI;

  if(!doit){
    return;
  }

  locomotion.stop();

  bool front_close = radar.getDistance(RADAR_FRONT,NULL) < DISTANCEEVITEMENT;
  bool left_close = radar.getDistance(RADAR_LEFT,NULL) < DISTANCEEVITEMENT;
  bool right_close = radar.getDistance(RADAR_RIGHT,NULL) < DISTANCEEVITEMENT;
  bool front_far = radar.getDistance(RADAR_FRONT,NULL) < 500;
  bool left_far = radar.getDistance(RADAR_LEFT,NULL) < 150;
  bool right_far = radar.getDistance(RADAR_RIGHT,NULL) < 150;
  
  if(front_close){
    if(left_far && !right_far){
      locomotion.avoid(M_PI/2);
    } else {
      locomotion.avoid(-M_PI/2);
    }
  } 
  else if(left_close and !(locomotion.state==SUIVILIGNES25 || locomotion.state==SUIVILIGNESFINI)){
    // locomotion.stop();
    if(front_far){
      locomotion.suiviLignes(GAUCHE);
    } else {
      locomotion.avoid(-M_PI/6);
    }
  } 
  else if(right_close and !(locomotion.state==SUIVILIGNES25 || locomotion.state==SUIVILIGNESFINI)){
    // locomotion.stop();
    if(front_far){
      locomotion.suiviLignes(DROITE);
    } else {
      locomotion.avoid(M_PI/6);
    }
  }
  else{
    locomotion.avoid(M_PI/2);
  }
}

void setup() {
  // LEDs OUTPUT
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  Serial.begin(115200);

  // countdown to start
  vTaskDelay(pdMS_TO_TICKS(100));
  for(int i=3; i>0; i--) {
    Serial.println(i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }


  // init I2C
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.setClock(100000);
  Wire.begin();
  Serial.println("begin");

  if(radar.init()) {
    Serial.println("[ERROR] initializing radar.");
  } else {
    Serial.println("[OK] Radar initialized.");
  }
  radar.setAlertDistances(DISTANCEEVITEMENT, DISTANCEEVITEMENT, DISTANCEEVITEMENT);
  radar.setAlertCallback(radar_alert_cb);
  radar.start();

  locomotion.start();
  locomotion.initPos(startPos);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // create blinker task
  xTaskCreate(
    blinker, "blinker", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );


  // create move test task
  // xTaskCreate(
  //   test_move, "test_move", configMINIMAL_STACK_SIZE,
  //   NULL, tskIDLE_PRIORITY + 1, NULL
  // );
  // xTaskCreate(
  //   test_rotate, "test_rotate", configMINIMAL_STACK_SIZE,
  //   NULL, tskIDLE_PRIORITY + 1, NULL
  // );

  xTaskCreate(
    move, "move", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );

  xTaskCreate(
    odometry, "odometry", configMINIMAL_STACK_SIZE,
    NULL, 0, NULL
  );

}



void loop() {
  static float w = 0;
  vTaskDelay(pdMS_TO_TICKS(2000));
  //Serial.println(d);
}
