#include "deplacement.h"
#include "base_roulante.h"

extern int distance_left;
extern int distance_middle;
extern int distance_right;
extern Base_roulante base_roulante;


void vTask_Deplacement (void* param){ 
  while (1){
    if (distance_left < DISTANCE_EVITEMENT || distance_middle < DISTANCE_EVITEMENT ||distance_right < DISTANCE_EVITEMENT){
      base_roulante.stop();
      Serial.println("trop pres d'un obstacle");
      sleep(50);
    }else{
      base_roulante.translate(200);
      sleep(2000);
      base_roulante.rotate(M_PI/2);
      sleep(1000);
      Serial.println("deplacement");
    }
  }
}