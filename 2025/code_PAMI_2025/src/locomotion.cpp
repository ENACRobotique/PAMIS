#include "locomotion.h"
#include "config.h"
#include "FreeRTOSConfig.h"




AccelStepper stepper1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2, DIR2);

Locomotion locomotion(&stepper1, &stepper2, ENABLE1, ENABLE2);


static void locomotion_run( void *arg );


Locomotion::Locomotion(AccelStepper* step_left, AccelStepper* step_right, pin_size_t en_left, pin_size_t en_right):
    step_left(step_left), step_right(step_right), en_left(en_left), en_right(en_right)
{
    step_left->setAcceleration(5000.0);
    step_left->setMaxSpeed(1000.0);
    
    step_right->setAcceleration(5000.0);
    step_right->setMaxSpeed(1000.0);

    mutex = xSemaphoreCreateMutex();
}

void Locomotion::start()
{
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

void Locomotion::doStep()
{
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        step_left->run();
        step_right->run();
        xSemaphoreGive(mutex);
    }
}


int Locomotion::translateBlocking(long steps)
{
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

void Locomotion::stop()
{
    if(xSemaphoreTake(mutex, portMAX_DELAY)  == pdTRUE) {
        step_left->stop();
        step_right->stop();
        stopped = true;
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
