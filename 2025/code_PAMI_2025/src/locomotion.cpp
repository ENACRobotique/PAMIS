#include "locomotion.h"
#include "config.h"
#include "FreeRTOSConfig.h"
#include "math.h"


long convertMmToStep(float mms){ return mms * MM2STEP;}
float convertStepToMm(long step){ return step / MM2STEP;}
long convertAngleToStep(float angle){ return RAYON_PAMI * angle * MM2STEP;}
float convertStepToAngle(long step){ return step / (RAYON_PAMI * MM2STEP);}

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

int Locomotion::rotateBlocking(float angle){

    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        stopped = false;
        step_left->move(convertAngleToStep(angle));
        step_right->move(convertAngleToStep(angle));
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
    while (state == INIT){
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            stopped = false;
            step_left->move(convertAngleToStep(dtheta));
            step_right->move(convertAngleToStep(dtheta));
            xSemaphoreGive(mutex);
            state=TOURNE;
        }
    }
    while(state == TOURNE) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            state =TOURNE_FINI;
        }
        if(stopped) {
            return 1;
        }
        taskYIELD();
    }
    while(state==TOURNE_FINI){
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            stopped = false;
            step_left->move(-convertMmToStep(sqrt(dy*dy+dx*dx)));
            step_right->move(convertMmToStep(sqrt(dy*dy+dx*dx)));
            xSemaphoreGive(mutex);
            state=TOUDRWA;
        }
    }   
    while(state==TOUDRWA) {
        bool ended = false;
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
        }
        if(ended) {
            state=TOUDRWA_FINI;
        }
        if(stopped) {
            return -1;
        }
        taskYIELD();
    }
    while(state == AVOIDING) {
        state = INIT;
        Serial.print("pos x : ");
        Serial.print(current_coord.x);
        Serial.print(", pos y : ");
        Serial.println(current_coord.y);
    }
    return 0;    
}

int Locomotion::move(coord * targets,int nb){
        if (i>2){return 0;}
        Serial.println("testtest");
        float dx=targets[i].x-current_coord.x;
        float dy=targets[i].y-current_coord.y;
        float dtheta=atan2(dy,dx)-current_coord.theta;
        while(state==INIT){
            if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                stopped = false;
                step_left->move(convertAngleToStep(dtheta));
                step_right->move(convertAngleToStep(dtheta));
                xSemaphoreGive(mutex);
                state=TOURNE;
            }
        }
        while(state == TOURNE) {
            bool ended = false;
            if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
                xSemaphoreGive(mutex);
            }
            if(ended) {
                state =TOURNE_FINI;
            }
            if(stopped) {
                return 1;
            }
            taskYIELD();
        }
        while(state==TOURNE_FINI){
            if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                stopped = false;
                step_left->move(-convertMmToStep(sqrt(dy*dy+dx*dx)));
                step_right->move(convertMmToStep(sqrt(dy*dy+dx*dx)));
                xSemaphoreGive(mutex);
                state=TOUDRWA;
            }
        }   
        while(state==TOUDRWA) {
            bool ended = false;
            if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
                xSemaphoreGive(mutex);
            }
            if(ended) {
                state=TOUDRWA_FINI;
            }
            if(stopped) {
                return -1;
            }
            taskYIELD();
        }
        if (state==TOUDRWA_FINI){
            i+=1;
            Serial.println(i);
            Serial.println(state);
            Serial.println("test");
            state=INIT;
        }
        while(state == AVOIDINGTOURNE || state==AVOIDINGTOURNEFINI) {
            Serial.println(state);
            while(state==AVOIDINGTOURNE){
                if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                    stopped = false;
                    step_left->move(convertAngleToStep(M_PI/2));
                    step_right->move(convertAngleToStep(M_PI/2));
                    xSemaphoreGive(mutex);
                    state=AVOIDINGTOURNEFINI;
                    Serial.println(state);

                }
            }
            while(state==AVOIDINGTOURNEFINI){
                bool ended = false;
                    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                        ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
                        xSemaphoreGive(mutex);
                    }
                    if(ended) {
                        state =AVOIDINGTOUDRWA;
                        Serial.println(state);

                    }
                    if(stopped) {
                        return 1;
                    }
                    taskYIELD();
            }
        }
            while(state==AVOIDINGTOUDRWA || state==AVOIDINGTOUDRWAFINI){
                while(state==AVOIDINGTOUDRWA){
                    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                        stopped = false;
                        step_left->move(-convertMmToStep(150));
                        step_right->move(convertMmToStep(150));
                        xSemaphoreGive(mutex);
                        state=AVOIDINGTOUDRWAFINI;
                        Serial.println(state);
                }
            }
            Serial.println("CALMBEFORETHESTORM");
                while(state==AVOIDINGTOUDRWAFINI){
                    Serial.print("testavoidingtoudrwafini");
                    bool ended = false;
                    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                        ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
                        xSemaphoreGive(mutex);
                    }
                    if(ended) {
                        state=INIT;
                        Serial.println(state);
                        Serial.println("INIT done");

                    }
                    if(stopped) {
                        return -1;
                    }
                    taskYIELD();
                    Serial.println("TASKYIELDED");
                }
                Serial.println("CALMAFTERTHESTORM");
            }
            
            
        return 0;    
    }
    

int Locomotion::avoid(){
    // Serial.println(state);
    state=AVOIDINGTOURNE;
    return 0;
}


void Locomotion::odometry(){

    float pos_1 = -step_left->currentPosition(); //return current position in step
    float pos_2 = step_right->currentPosition();



    float dpos_1 = (pos_1 - old_pos_1); 
    float dpos_2 = (pos_2 - old_pos_2) ; 

    current_coord.theta += convertStepToAngle((-dpos_1 + dpos_2) / 2 ); //en radian
    current_coord.x += convertStepToMm((((dpos_1 + dpos_2)/2) * cos (current_coord.theta))); //in mm
    current_coord.y += convertStepToMm(((dpos_1 + dpos_2)/2) * sin (current_coord.theta)); //in mm

    old_pos_1 = pos_1;
    old_pos_2 = pos_2;
 }



void Locomotion::stop(){
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        if(!(state==AVOIDINGTOURNE || state==AVOIDINGTOURNEFINI || state==AVOIDINGTOUDRWA || state==AVOIDINGTOUDRWAFINI)){

        }
        step_left->stop();
        step_right->stop();
        stopped = true;
        // odometry();
        // state = AVOIDING;
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
