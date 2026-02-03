#include "locomotion.h"
#include "stepper.h"
#include "config.h"
#include "utils.h"

// 1.8° per step, wheel 3inch diameter
constexpr double STEPS_PER_MM = (360.0/1.8) / (M_PI * 76.2);

constexpr float WHEELBASE = 132.55; // mm

Stepper_config_t step_cfg = {
    .stepPin = 39,
    .dirPin = 38,
    .enPin = 40,
    .miStep = MICROSTEP_8,
    .stepAngle = 1.8
};

Stepper_config_t step2_cfg = {
    .stepPin = 36,
    .dirPin = 35,
    .enPin = 40,
    .miStep = MICROSTEP_8,
    .stepAngle = 1.8
};


void Locomotion::init() {
    step_left.config(&step_cfg);
    step_left.init();
    step_left.setStepsPerMm(STEPS_PER_MM);
    step_left.setSpeedMm(1000, 4000, 1000);
    step_right.config(&step2_cfg);
    step_right.init();
    step_right.setStepsPerMm(STEPS_PER_MM);
    step_right.setSpeedMm(1000, 4000, 1000);

    oldPosLeft = getPosLeft();
    oldPosRight = getPosRight();

    xTaskCreate(odometry_task, "Blinker", configMINIMAL_STACK_SIZE+1024, this, 2, NULL);

}

void Locomotion::stop() {
    step_left.stopSlow();
    step_right.stopSlow();
    waitFinishedTimeout(1000 / portTICK_PERIOD_MS);
}

void Locomotion::move(float lenght, float angle) {
    float lenght_left = lenght - angle * WHEELBASE / 2.0f;
    float lenght_right = lenght + angle * WHEELBASE / 2.0f;
    step_left.runPosMm(lenght_left);
    step_right.runPosMm(lenght_right);
}

void Locomotion::moveBlocking(float lenght, float angle) {
    move(lenght, angle);
    waitFinishedTimeout(portMAX_DELAY);
}

void Locomotion::enableSteppers(bool enable) {
    if(enable) {
        step_left.enableMotor();
    } else {  
        step_left.disableMotor();
    }
}

bool Locomotion::moving() {
    bool left_moving = step_left.getState() != motor_status::IDLE && step_left.getState() != motor_status::DISABLED;
    bool right_moving = step_right.getState() != motor_status::IDLE && step_right.getState() != motor_status::DISABLED;
    return left_moving || right_moving;
}

void Locomotion::odometry_task(void* arg)
{
    Locomotion* that = static_cast<Locomotion*>(arg);
    
    while(true) {
        float pos_left = that->getPosLeft();
        float pos_right = that->getPosRight();
        
        float dLeft = pos_left - that->oldPosLeft;
        float dRight = pos_right - that->oldPosRight;
        float dTheta = (dLeft - dRight)/WHEELBASE/4;
        float d = (dRight + dLeft)/2;
        
        // ancienne position
        Position pos = that->getPos();

        pos.x += d * cos(pos.theta+dTheta/2);
        pos.y += d * sin(pos.theta+dTheta/2);
        pos.theta += dTheta;
        pos.theta = normalise(pos.theta);

        that->setPos(pos);

        // sleep jusqu'à la prochaine période
        vTaskDelay(ODOMETRY_PERIOD_MS / portTICK_PERIOD_MS);

    }

}

BaseType_t Locomotion::waitFinishedTimeout(TickType_t xTicksToWait) {
    return step_left.waitFinishedTimeout(xTicksToWait) && step_right.waitFinishedTimeout(xTicksToWait);
}


