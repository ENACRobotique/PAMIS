#include <Arduino.h>
#include "config.h"
#include "VL53L1X_api.h"
#include <FreeRTOS.h>
#include "task.h"
#include "Wire.h"
#include "locomotion.h"
#include "radar.h"


// task qui fait clignoter une LED en continu.
static void blinker( void *arg ) {
  while(true) {
    digitalWrite(LED1, !digitalRead(LED1));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void test_move( void *arg ) {
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
  while(true) {
    if(locomotion.rotateBlocking(1000)) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    if(locomotion.rotateBlocking(-1000)) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
  } 
}

static void test_moveB(void * args) {
  while(true) {
    if(locomotion.moveBlocking({200,200,0}))
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    if(locomotion.moveBlocking({200,200,0})) {
      while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
  } 


// static void test_moveOnce( void *arg ) {
//   while(true) {
//     locomotion.translateBlocking(10000);
//     vTaskDelay(pdMS_TO_TICKS(2000));
//   }
// }

// static void move_correctly(void * args){
  
// }


void odometry(void*){
  while (-1){
    locomotion.odometry();
    // coord position=locomotion.getPositon();
    // Serial.print("pos x : ");
    // Serial.print(position.x);
    // Serial.print(", pos y : ");
    // Serial.print(position.y);
    // Serial.print(", theta : ");

    // char buffer[15]; // Un buffer suffisamment grand pour contenir le nombre formaté
    // dtostrf(position.theta, 13, 7, buffer); // Convertit le flottant en chaîne avec 7 chiffres après la virgule
    // Serial.println(buffer); // Affiche la chaîne formatée
    //Serial.printf("x: %f, y: %f, theta: %f \n",position.x, position.y, position.theta);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}



static void radar_alert_cb() {
  digitalWrite(LED2, !digitalRead(LED2));
  locomotion.stop();
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
  

  if(radar.init()) {
    Serial.println("[ERROR] initializing radar.");
  } else {
    Serial.println("[OK] Radar initialized.");
  }

  radar.setAlertDistances(100, 100, 100);
  radar.setAlertCallback(radar_alert_cb);
  radar.start();

  locomotion.start();


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
    test_moveB, "test_move", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );

  xTaskCreate(
    odometry, "odometry", configMINIMAL_STACK_SIZE,
    NULL, 0, NULL
  );

}



void loop() {
  static float w = 0;
  vTaskDelay(pdMS_TO_TICKS(50));
  uint16_t d = radar.getDistance(RADAR_LEFT, NULL);
  Serial.print(locomotion.etat);
  //Serial.println(d);
}
