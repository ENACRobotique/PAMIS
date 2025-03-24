#include "locomotion.h"
#include "config.h"
#include "FreeRTOSConfig.h"
#include "math.h"



long convertMmToStep(long mms){ return mms * MM2STEP;}
long convertAngleToStep(long angle){ return RAYON_PAMI * angle * MM2STEP;}

AccelStepper stepper1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2, DIR2);

Locomotion locomotion(&stepper1, &stepper2, ENABLE1, ENABLE2);





static void locomotion_run( void *arg );


Locomotion::Locomotion(AccelStepper* step_left, AccelStepper* step_right, pin_size_t en_left, pin_size_t en_right):
    step_left(step_left), step_right(step_right), en_left(en_left), en_right(en_right){
    step_left->setAcceleration(5000.0);
    step_left->setMaxSpeed(1000.0);
    
    step_right->setAcceleration(5000.0);
    step_right->setMaxSpeed(1000.0);

    mutex = xSemaphoreCreateMutex();
}

void Locomotion::start(){
    pinMode(en_left, OUTPUT);
    pinMode(en_right, OUTPUT);
    digitalWrite(en_left, LOW);
    digitalWrite(en_right, LOW);

    TaskHandle_t xHandle;
    xTaskCreate(
        locomotion_run, "stepper_run", configMINIMAL_STACK_SIZE,
        &locomotion, tskIDLE_PRIORITY + 3, &xHandle );

    // run task on core 1
    UBaseType_t uxCoreAffinityMask = (1 << 1 );  
    vTaskCoreAffinitySet( xHandle, uxCoreAffinityMask ); 
}

void Locomotion::doStep(){
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        step_left->run();
        step_right->run();
        xSemaphoreGive(mutex);
    }
}


int Locomotion::translateBlocking(long steps){
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        stopped = false;
        step_left->move(-steps);
        step_right->move(steps);
        xSemaphoreGive(mutex);
    }

    while(true) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            return 0;
        }
        if(stopped) {
            return -1;
        }
        taskYIELD();
    }
}

int Locomotion::rotateBlocking(long steps){

    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        stopped = false;
        step_left->move(steps);
        step_right->move(steps);
        xSemaphoreGive(mutex);
    }

    while(true) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            return 0;
        }
        if(stopped) {
            return 1;
        }
        taskYIELD();
    }

}



int Locomotion::moveBlocking(coord target){
    float dx=target.x-current_coord.x;
    float dy=target.y-current_coord.y;
    float dtheta=atan2(dy,dx)-current_coord.theta;
    Serial.println(convertAngleToStep(dtheta));
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        stopped = false;
        step_left->move(convertAngleToStep(dtheta));
        step_right->move(convertAngleToStep(dtheta));
        xSemaphoreGive(mutex);
    }
    etat =1;
    while(etat == 1) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            etat =2;
        }
        if(stopped) {
            return 1;
        }
        taskYIELD();
    }
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        stopped = false;
        step_left->move(-sqrt(dy*dy+dx*dx));
        step_right->move(sqrt(dy*dy+dx*dx));
        xSemaphoreGive(mutex);
    }
    etat = 3;
    while(etat==3) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            etat=4;
        }
        if(stopped) {
            return -1;
        }
        taskYIELD();
    }
    return 0;    
}




void Locomotion::odometry(){
        static float old_pos_1 = 0;
        static float old_pos_2 = 0;
        float pos_1 = step_left->currentPosition(); //return current position in step
        float pos_2 = step_right->currentPosition();
    
        float dpos_1 = (pos_1 - old_pos_1) / MM2STEP;
        float dpos_2 = (pos_2 - old_pos_2) / MM2STEP; 
    
        current_coord.theta += (-dpos_1 - dpos_2) / (2 * RAYON_PAMI);
        current_coord.x += ((dpos_1 - dpos_2)/2) * cos (current_coord.theta);
        current_coord.y += ((dpos_1 - dpos_2)/2) * sin (current_coord.theta); 
    
        old_pos_1 = pos_1;
        old_pos_2 = pos_2;
  
 }

void Locomotion::stop(){
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        step_left->stop();
        step_right->stop();
        stopped = true;
        // odometry();
        xSemaphoreGive(mutex);
    }
}

static void locomotion_run( void *arg ) {
    Locomotion* loco = (Locomotion*) arg;
    while(true) {
        loco->doStep();
        taskYIELD();
    }
    
}
