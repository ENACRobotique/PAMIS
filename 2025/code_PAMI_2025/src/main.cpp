#include <Arduino.h>
#include "config.h"
#include "VL53L1X_api.h"
#include <FreeRTOS.h>
#include "task.h"
#include "Wire.h"
#include "locomotion.h"
#include "radar.h"
#include "time.h"
#include "Servo.h"
#define DISTANCE_FIN 150
#define BLUE 1
#define YELLOW 2
#define TIRETTE_IN LOW
#define TIRETTE_OUT HIGH
Servo servobras;


// #if defined(BOWIE)
// #define NB_TARGET 3
// int hDecale = 0;
// coord startPos={75,1840,0};
// coord target[NB_TARGET] = {{400,1840,0},{650,1400,0},{1850,1400,0}};
#if defined(JIMMY)
#define NB_TARGET 2
coord startPos={75,1615,0};
coord target[NB_TARGET] = {{300,1615,0},{900,1500,0}};
int hDecale = 4;
//startPos={(500,500,0)};
// coord target[1] = {{1000,0,0}};
#elif defined(STEVE)
#define NB_TARGET 3
int hDecale=0;
coord startPos={75,1750,0};
coord target[NB_TARGET] = {{300,1750,0},{650,1400,0},{1220,1330,0}};
#elif defined(JOHNNY)
#define NB_TARGET 4
int hDecale=0;
coord startPos={75,1910,0};
coord target[NB_TARGET] = { {600,1910,0}, {1280,1910,0}, {1280, 2030, M_PI}, {1280,1670,0}};
#endif
// coord target[1] = {{0,0,0}};
// int nb=1;
// task qui fait clignoter une LED en continu.
static void blinker( void *arg ) {
  while(true) {
    digitalWrite(LED1, !digitalRead(LED1));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void move(void * args) {
    // coord target[1] = {{1150,1400,0}};
    while(true) {
      #if defined(JOHNNY)
      locomotion.superstar(target, NB_TARGET);
      #else
      locomotion.move(target, NB_TARGET);
      #endif
      //coord current_coords=locomotion.getPositon();
      // Serial.print(", pos x : ");
      // Serial.println(current_coords.x);
      // Serial.print(", pos y : ");
      // Serial.print(current_coords.y);
      // Serial.print(", theta : ");
      // Serial.print(current_coords.theta);
    } 
  }


void odometry(void*){
  while (-1){
    locomotion.odometry();
    coord current_coord=locomotion.getPositon();
    // Serial.print("pos x : ");
    // Serial.println(current_coord.x);
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

void clock(void *){
  time_t t=time(NULL);
  while (!locomotion.finDuMatch){
    // Serial.print(difftime(time(NULL),t));
    if (time(NULL)-t >=15){
      locomotion.finDuMatch=true;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  
  while(true){
    servobras.writeMicroseconds(2000);
    vTaskDelay(pdMS_TO_TICKS(750));
    servobras.writeMicroseconds(1000);
    vTaskDelay(pdMS_TO_TICKS(750));
  }


  // int i=0;
  // time_t t=time(NULL);
  // while (i<15) {
  //   if (time(NULL)-t>1){
  //     i+=1;
  //     t=time(NULL);
  //     Serial.print((int)i);void resume();
  // }
}

static void radar_alert_cb() {
  digitalWrite(LED2, !digitalRead(LED2));
  #if defined(JOHNNY)
  locomotion.stop();
  locomotion.avoid(0);
  #else
  
  // Serial.print("HE IS ALIVE");
  #if not defined(JOHNNY)
  coord tgt_pos = target[NB_TARGET-1];
  float x = locomotion.getPositon().x;
  float y = locomotion.getPositon().y;
  float dist = sqrt(pow((x-tgt_pos.x), 2) + pow((y-tgt_pos.y), 2));
  if(dist <= DISTANCE_FIN){
    locomotion.stop();
    return;} // si on est proche du point d'arrivé et qu'on veut esquiver, on s'arrête
  #endif
  bool doit = 
    locomotion.state==INIT ||
    locomotion.state==TOUDRWA ||
    locomotion.state==TOUDRWA_FINI ||
    locomotion.state==FINITOPIPO ||
    locomotion.state==AVOIDINGTOUDRWA ||
    locomotion.state==AVOIDINGTOUDRWAFINI ||
    locomotion.state==SUIVILIGNES25 ||
    locomotion.state==STOPPED||
    locomotion.state==SUIVILIGNESFINI;

  if(!doit){
    return;
  }

  bool front_close = radar.getDistance(RADAR_FRONT,NULL) < DISTANCEEVITEMENT;
  bool left_close = radar.getDistance(RADAR_LEFT,NULL) < DISTANCEEVITEMENT;
  bool right_close = radar.getDistance(RADAR_RIGHT,NULL) < DISTANCEEVITEMENT;
  bool front_far = radar.getDistance(RADAR_FRONT,NULL) < 500;
  
  int equipe;
  if(digitalRead(FDC1)==LOW){
    equipe=BLUE;
  } else {
    equipe=YELLOW;
  }

  // if ((front_close or right_close or left_close))// and (dist <=DISTANCE_FIN)){
  // {
  //   locomotion.stop();
  // }
  // else 
  // {
  //   locomotion.resume();
  // }

  // else 
  if(front_close){
    if(equipe == BLUE){
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
    // locomotion.avoid(M_PI/2);
  }
  #endif

  
}


void setup() {
  // LEDs OUTPUT
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(FDC1,INPUT_PULLUP);
  pinMode(TIRETTE,INPUT_PULLUP);
  
  xTaskCreate(
    blinker, "blinker", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );
  // init I2C
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.setClock(100000);
  Wire.begin();
  Serial.begin(115200);
  // countdown to start
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("begin");
  
  if(radar.init()) {
    Serial.println("[ERROR] initializing radar.");
  } else {
    Serial.println("[OK] Radar initialized.");
  }
  #if defined(JOHNNY)
  radar.setAlertDistances(DISTANCEEVITEMENT_JHONNY, DISTANCEEVITEMENT, DISTANCEEVITEMENT_JHONNY);
  #else
  radar.setAlertDistances(DISTANCEEVITEMENT, DISTANCEEVITEMENT, DISTANCEEVITEMENT);
  #endif
  radar.setAlertCallback(radar_alert_cb);
  
  locomotion.start();
  // coord startPos={0,0,0};
  if(digitalRead(FDC1)==LOW){
    startPos={3000-startPos.x,startPos.y,(float)M_PI-startPos.theta}; // equipe bleue si FDC1 est LOW
    for (int i =0; i<NB_TARGET; i++){
      target[i] = {3000-target[i].x,target[i].y,target[i].theta};
    }
    Serial.println("equipe bleue");
  } else {
    // startPos={2925,1625,M_PI}; // equipe jaune par défaut
    Serial.println("equipe jaune");
  }
  servobras.attach(SERVO);
  #if defined(JOHNNY)
  servobras.writeMicroseconds(1800);
  #else
  servobras.writeMicroseconds(1000);
  #endif
  vTaskDelay(pdMS_TO_TICKS(500));
  //servobras.writeMicroseconds(1000);
  // Serial.println(startPos.x);
  // Serial.println(startPos.y);
  // Serial.println(startPos.theta);
  locomotion.initPos(startPos);
  // coord Robert=locomotion.getPositon();
  // Serial.println(Robert.x);
  // Serial.println(Robert.y);
  // Serial.println(Robert.theta);
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // if (digitalRead(FDC2)==TIRETTE_IN){
  //   Serial.println("tirrette detected");
  // } else if(digitalRead(FDC2)==TIRETTE_OUT){
  //   Serial.println("tirrette NOT detected");
  // }

  while(digitalRead(TIRETTE)==TIRETTE_OUT){
    servobras.writeMicroseconds(2000);
    vTaskDelay(pdMS_TO_TICKS(750));
    servobras.writeMicroseconds(1000);
    vTaskDelay(pdMS_TO_TICKS(750));
    Serial.println("waiting for tirrette");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  while(digitalRead(TIRETTE)==TIRETTE_IN){
    Serial.println("waiting for tirrette in");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  // vTaskDelay(pdMS_TO_TICKS(85000));
  
  for(int i=85; i>0; i--) {
    Serial.println(i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  xTaskCreate(
    clock, "clock", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 2, NULL
  );


  if(hDecale){

    for(int i=hDecale; i>0; i--) {
      Serial.println(i);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  
  xTaskCreate(
    move, "move", configMINIMAL_STACK_SIZE,
    NULL, tskIDLE_PRIORITY + 1, NULL
  );
  radar.start();

  xTaskCreate(
    odometry, "odometry", configMINIMAL_STACK_SIZE,
    NULL, 0, NULL
  );


}



void loop() {
  vTaskDelay(pdMS_TO_TICKS(2000));
  //Serial.println(d);
}
