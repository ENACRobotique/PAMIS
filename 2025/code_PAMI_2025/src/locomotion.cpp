#include "locomotion.h"
#include "config.h"
#include "FreeRTOSConfig.h"
#include "math.h"
#include "radar.h"

#if defined(JOHNNY)
#define ACCELMAX 5000.0
#define VMAX 4000 
#else
#define ACCELMAX 15000.0
#define VMAX 2500
#endif



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
    step_left->setAcceleration(ACCELMAX);
    step_left->setMaxSpeed(VMAX);
    
    step_right->setAcceleration(ACCELMAX);
    step_right->setMaxSpeed(VMAX);

    mutex = xSemaphoreCreateMutex();
}

void Locomotion::start(){
    pinMode(en_left, OUTPUT);
    pinMode(en_right, OUTPUT);
    digitalWrite(en_left, LOW);
    digitalWrite(en_right, LOW);

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
            Serial.println("gonna tourne");
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
            Serial.println("tourne");
        }
        if(ended) {
            state =TOURNE_FINI;
            Serial.println("tourne fini");

        }
        if(stopped) {
            return 1;
        }
        taskYIELD();
    }
    while(state==TOURNE_FINI){
        if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
            stopped = false;
            Serial.println("gonna toudrwa");
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
            Serial.println("toudrwa");

        }
        if(ended) {
            state=TOUDRWA_FINI;
            Serial.println("toudrwafini");

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

float angleLegal(float angle){
    if (angle>M_PI){
        angle = angleLegal(angle-2*M_PI);
    }else if (angle < -M_PI){
        angle = angleLegal(angle+2*M_PI);
    }
    return angle;
}

int Locomotion::move(coord* targets,int nb){
        if(finDuMatch || (target_idx >= nb)){
            vTaskDelay(pdMS_TO_TICKS(200));
            Serial.printf("fin: %d\n", finDuMatch);
            return 0;
        }
        float dx=targets[target_idx].x-current_coord.x;
        float dy=targets[target_idx].y-current_coord.y;
        float dtheta=angleLegal(atan2(dy,dx)-current_coord.theta);
        while(state==INIT) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            stopped = false;
            step_left->move(convertAngleToStep(dtheta));
            step_right->move(convertAngleToStep(dtheta));
            Serial.println("gonna tourne");
            xSemaphoreGive(mutex);
            state=TOURNE;
        }
        while(state == TOURNE) {
            bool ended = false;
            xSemaphoreTake(mutex, portMAX_DELAY);
            ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
            //Serial.println("tourne");
            if(ended) {
                state =TOURNE_FINI;
                Serial.println("tournefini");

            }
            if(stopped) {
                return 1;
            }
            taskYIELD();
        }
        while(state==TOURNE_FINI){
            xSemaphoreTake(mutex, portMAX_DELAY);
            stopped = false;
            step_left->move(-convertMmToStep(sqrt(dy*dy+dx*dx)));
            step_right->move(convertMmToStep(sqrt(dy*dy+dx*dx)));
            Serial.println("gonna toudrwa");
            xSemaphoreGive(mutex);
            state=TOUDRWA;
        }   
        while(state==TOUDRWA) {
            bool ended = false;
            xSemaphoreTake(mutex, portMAX_DELAY);
            ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
            xSemaphoreGive(mutex);
            if(ended) {
                state=TOUDRWA_FINI;
                Serial.println("toudrwafini");

            }
            if(stopped) {
                return -1;
            }
            taskYIELD();
        }
        if (state==TOUDRWA_FINI){
            target_idx+=1;
            // Serial.println(target_idx);
            // Serial.println(state);
            // Serial.println("test");
            state=INIT;
        }
        while(state == AVOIDINGTOURNE || state==AVOIDINGTOURNEFINI) {
            while(state==AVOIDINGTOURNE){
                xSemaphoreTake(mutex, portMAX_DELAY);
                stopped = false;
                Serial.printf(" esquive %d \n",side);
                step_left->move(convertAngleToStep(avoid_angle));
                step_right->move(convertAngleToStep(avoid_angle));
                xSemaphoreGive(mutex);
                state=AVOIDINGTOURNEFINI;
            }
            while(state==AVOIDINGTOURNEFINI){
                bool ended = false;
                    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
                        ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
                        xSemaphoreGive(mutex);
                    }
                    if(ended) {
                        state =AVOIDINGTOUDRWA;

                    }
                    if(stopped) {
                        return 1;
                    }
                    taskYIELD();
            }
        }
            while(state==AVOIDINGTOUDRWA || state==AVOIDINGTOUDRWAFINI){
                while(state==AVOIDINGTOUDRWA){
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    stopped = false;
                    step_left->move(-convertMmToStep(200));
                    step_right->move(convertMmToStep(200));
                    xSemaphoreGive(mutex);
                    state=AVOIDINGTOUDRWAFINI;
                
            }
                while(state==AVOIDINGTOUDRWAFINI){
                    bool ended = false;
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
                    xSemaphoreGive(mutex);
                    if(ended) {
                        state=INIT;

                    }
                    if(stopped) {
                        return -1;
                    }
                    taskYIELD();
                }
            }
        while(state==SUIVILIGNES || state==SUIVILIGNES2 || state==SUIVILIGNES25 || state==SUIVILIGNESFINI){

            while(state==SUIVILIGNES){
                eRadar radarEnQuestion;
                if(side==GAUCHE){radarEnQuestion=RADAR_LEFT;}
                else if(side==DROITE){radarEnQuestion=RADAR_RIGHT;}
                xSemaphoreTake(mutex, portMAX_DELAY);
                stopped = false;
                double angle=atan2(sin(ANGLERF2RL),((radar.getDistance(RADAR_FRONT,NULL)+RF2CENTER)/(radar.getDistance(radarEnQuestion,NULL)+RL2CENTER))-cos(ANGLERF2RL));
                if(side==GAUCHE){angle=-angle;}
                step_left->move(convertAngleToStep(angle));
                step_right->move(convertAngleToStep(angle));
                xSemaphoreGive(mutex);
                state=SUIVILIGNES2;
                    
            }
            while(state==SUIVILIGNES2){
                bool ended = false;
                xSemaphoreTake(mutex, portMAX_DELAY);
                ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
                xSemaphoreGive(mutex);
                if(ended) {
                    state=SUIVILIGNES25;
                    
                }
                if(stopped) {
                    return -1;
                }
                taskYIELD();
            }
            while(state==SUIVILIGNES25){
                eRadar radarEnQuestion;
                if(side==GAUCHE){radarEnQuestion=RADAR_LEFT;}
                else if(side==DROITE){radarEnQuestion=RADAR_RIGHT;}
                int dist_radar=radar.getDistance(radarEnQuestion,NULL);
                while(dist_radar<200){
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    stopped = false;
                    double distanceToEndOfWall=cos(ANGLERF2RL)*dist_radar+RADARTOROUES;
                    // Serial.println("distanceToEndOfWall");
                    // Serial.println(convertMmToStep(distanceToEndOfWall));
                    step_left->move(-convertMmToStep(distanceToEndOfWall));
                    step_right->move(convertMmToStep(distanceToEndOfWall));
                    xSemaphoreGive(mutex);
                    dist_radar=radar.getDistance(radarEnQuestion,NULL);
                }
                state=SUIVILIGNESFINI;
                while(state==SUIVILIGNESFINI){
                    bool ended = false;
                    xSemaphoreTake(mutex, portMAX_DELAY);
                        ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
                        xSemaphoreGive(mutex);
                    if(ended) {
                        state=INIT;
                        
                    }
                    if(stopped) {
                        return -1;
                    }
                    taskYIELD();
                }
            }
        }
        
            
        return 0;    
    }


int Locomotion::superstar(coord* targets, int nb){
    if(finDuMatch || (target_idx >= nb)){
        vTaskDelay(pdMS_TO_TICKS(200));
        return 0;
    }
    float dx=targets[target_idx].x-current_coord.x;
    float dy=targets[target_idx].y-current_coord.y;
    float dtheta=angleLegal(atan2(dy,dx)-current_coord.theta);

    while(state==INIT) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        stopped = false;
        if (targets[target_idx].theta != 0){
            dtheta = angleLegal(dtheta + M_PI);
            }
        step_left->move(convertAngleToStep(dtheta));
        step_right->move(convertAngleToStep(dtheta));
        Serial.println("gonna tourne");
        xSemaphoreGive(mutex);
        state=TOURNE;
    }
    while(state == TOURNE) {
        bool ended = false;
        xSemaphoreTake(mutex, portMAX_DELAY);
        ended = step_left->distanceToGo()==0 && step_right->distanceToGo() == 0;
        xSemaphoreGive(mutex);
        //Serial.println("tourne");
        if(ended) {
            state =TOURNE_FINI;
            Serial.println("tournefini");

        }
        if(stopped) {
            return 1;
        }
        taskYIELD();
    }
    while(state==TOURNE_FINI){
        xSemaphoreTake(mutex, portMAX_DELAY);
        stopped = false;
        long dist = convertMmToStep(sqrt(dy*dy+dx*dx));
        if (targets[target_idx].theta != 0){
            dist = -dist;
            }
        step_left->move(-dist);
        step_right->move(dist);
        Serial.println("gonna toudrwa");
        xSemaphoreGive(mutex);
        state=TOUDRWA;
    }   
    while(state==TOUDRWA) {
        bool ended = false;
        xSemaphoreTake(mutex, portMAX_DELAY);
        ended = step_left->distanceToGo() == 0 && step_right->distanceToGo() == 0;
        xSemaphoreGive(mutex);
        if(ended) {
            state=TOUDRWA_FINI;
            Serial.println("toudrwafini");

        }
        if(stopped) {
            return -1;
        }
        taskYIELD();
    }
    if (state==TOUDRWA_FINI){
        target_idx+=1;
        // Serial.println(target_idx);
        // Serial.println(state);
        // Serial.println("test");
        state=INIT;
    }
    while(state==AVOIDINGTOURNE){
        state=INIT;
        vTaskDelay(pdMS_TO_TICKS(200));
        taskYIELD();
    }  
    return 0;    
}

void Locomotion::avoid(float angle){
    state=AVOIDINGTOURNE;
    avoid_angle=angle;
}

void Locomotion::suiviLignes(sidE siDe){
    state=SUIVILIGNES;
    side=siDe;
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
    step_left->setAcceleration(5*ACCELMAX);
    step_right->setAcceleration(5*ACCELMAX);
    
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        step_left->stop();
        step_right->stop();
        stopped = true;
        state = STOPPED;
        xSemaphoreGive(mutex);
    }
    step_left->setAcceleration(ACCELMAX);
    step_right->setAcceleration(ACCELMAX);
}

static void locomotion_run( void *arg ) {
    Locomotion* loco = (Locomotion*) arg;
    while(!loco->finDuMatch) {
        loco->doStep();
        taskYIELD();
    }
}


void Locomotion:: resume()
{
    state=INIT;
}