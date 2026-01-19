#include "locomotion.h"
#include "stepper.h"

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


bool Locomotion::moving() {
    bool left_moving = step_left.getState() != motor_status::IDLE && step_left.getState() != motor_status::DISABLED;
    bool right_moving = step_right.getState() != motor_status::IDLE && step_right.getState() != motor_status::DISABLED;
    return left_moving || right_moving;
}

BaseType_t Locomotion::waitFinishedTimeout(TickType_t xTicksToWait) {
    return step_left.waitFinishedTimeout(xTicksToWait) && step_right.waitFinishedTimeout(xTicksToWait);
}