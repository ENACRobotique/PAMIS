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
    coord target[3] = {{1000,0,0},{1000,1000,0},{0,0,0}};
    while(true) {
      //locomotion.moveBlocking({-2000,2000,0});
      // if(locomotion.move(target, 2)){
      // }
      locomotion.move(target,2);
        // while(radar.getDistance(RADAR_LEFT, NULL) < 100 ) {
        //   while(!(locomotion.state==AVOIDINGTOURNE || locomotion.state==AVOIDINGTOURNEFINI || locomotion.state==AVOIDINGTOUDRWA || locomotion.state==AVOIDINGTOUDRWAFINI)) {
        //     locomotion.avoid();
        //     Serial.print(locomotion.state);
        //     vTaskDelay(pdMS_TO_TICKS(200));
        //   }
        // }
      // if(locomotion.moveBlocking({1000,0,0})) {
      //   while(radar.getDistance(RADAR_LEFT, NULL) < 100) {
      //     vTaskDelay(pdMS_TO_TICKS(200));
      //   }
      // }
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
  while(!(locomotion.state==AVOIDINGTOURNE || locomotion.state==AVOIDINGTOURNEFINI || locomotion.state==AVOIDINGTOUDRWA || locomotion.state==AVOIDINGTOUDRWAFINI)) {
    if(!(radar.getDistance(RADAR_FRONT, NULL) < 100 and radar.getDistance(RADAR_LEFT, NULL) < 100 and radar.getDistance(RADAR_RIGHT, NULL) < 100)){
      if(radar.getDistance(RADAR_LEFT, NULL) < 100 and !(radar.getDistance(RADAR_RIGHT, NULL) < 100)){
        locomotion.avoid("droite");
      }
      //if(radar.getDistance(RADAR_RIGHT, NULL) < 100)
      else {
        locomotion.avoid("gauche");
      };
    };
  }
  //locomotion.stop();
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
  vTaskDelay(pdMS_TO_TICKS(50));
  uint16_t d = radar.getDistance(RADAR_LEFT, NULL);
  //Serial.print(locomotion.etat);
  //Serial.println(d);
}
