#include <Arduino.h>
#include "config.h"
#include "VL53L1X_api.h"
#include <FreeRTOS.h>
#include "task.h"
#include "Wire.h"
#include "locomotion.h"


// task qui fait clignoter une LED en continu.
static void blinker( void *arg ) {
  while(true) {
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}


void setup() {
  // LEDs OUTPUT
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  Serial.begin(115200);

  // init I2C
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();

  // create blinker task
  xTaskCreate(
    blinker, "blinker", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );

  locomotion.start();

}



void loop() {
  locomotion.translateBlocking(400);
  vTaskDelay(pdMS_TO_TICKS(200));
  locomotion.translateBlocking(-400);
  vTaskDelay(pdMS_TO_TICKS(1000));
}
