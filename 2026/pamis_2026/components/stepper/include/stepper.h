
#pragma once

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "hal/gpio_ll.h"

#define TIMER_F 1000000ULL

enum motor_status
{
    DISABLED,
    IDLE,
    ACC,
    COAST,
    DEC
};

enum dir
{
    CW,
    CCW
};

typedef enum {
    MICROSTEP_1 = 0x1,
    MICROSTEP_2,
    MICROSTEP_4 = 0x4,
    MICROSTEP_8 = 0x8,
    MICROSTEP_16 = 0x10,
    MICROSTEP_32 = 0x20,
    MICROSTEP_64 = 0x40,
    MICROSTEP_128 = 0x80,
    MICROSTEP_256 = 0x100,
} microStepping_t;

/**
 * @brief Configuration structure
 */
typedef struct
{
    uint8_t stepPin;                /** step signal pin */
    uint8_t dirPin;                 /** dir signal pin */
    uint8_t enPin;                  /** enable signal pin */
    microStepping_t miStep;         /** microstepping configured on driver - used in distance calculation */
    float stepAngle;                /** one step angle in degrees (usually 1.8deg), used in steps per rotation calculation */
} Stepper_config_t;

typedef struct
{
    uint32_t steps_done = 0;   // step counter
    uint32_t steps_total = 0; // steps we need to take
    double v_max = 500;      // speed in step/s
    double acc = 1000;        // acceleration in step*second^-2
    //float dec = 100;        // decceleration in step*second^-2
    uint8_t status = DISABLED;
    bool dir = CW;
    // bool runInfinite = false;
    double stepsPerMm = 0; /** Steps per one milimiter, if the motor is performing linear movement */
    
    uint32_t n;
    uint32_t n_acc;          // nombre de steps d'accélération
    uint32_t n_dec;          // nombre de steps de décélération
    uint32_t T;                 // période courante (s)
    uint32_t T_min;             // période min (vitesse max)
} ctrl_var_t;

class Stepper
{
private:
    Stepper_config_t conf;
    ctrl_var_t ctrl;
    TaskHandle_t enTask;
    gptimer_handle_t timer_handle;  /** timer handle */
    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0
    };
    int64_t currentPos = 0; // absolute position

    SemaphoreHandle_t bsem;

    /** @brief sets En GPIO
     *  @param state 0-LOW,1-HIGH
     *  @return void
     */
    void setEn(bool);

    /** @brief sets Dir GPIO
     *  @param state 0-CW 1-CCW
     */
    void setDir(bool);

    /** @brief static wrapper for ISR function
     *  @param _this Stepper* this pointer
     *  @return bool
     */
    static bool xISRwrap(gptimer_t* timer, const gptimer_alarm_event_data_t* data,void *_this)
    {
        return static_cast<Stepper *>(_this)->xISR(timer,data);
    }

    /** @brief enableMotor wrapper
     */
    static void _disableMotor(void *_this)
    {
        static_cast<Stepper *>(_this)->disableMotor();
    }

    bool xISR(gptimer_t* timer, const gptimer_alarm_event_data_t* data);

public:
    /** @brief Costructor - conf variables to be passed later
     */
    Stepper();

    /** @brief Configuration of library, used with constructor w/o params
     *  @param config Stepper_config_t structure pointer - can be freed after this call
     */
    void config(Stepper_config_t *config);

    /** @brief initialize GPIO and Timer peripherals
     *  @param stepP step pulse pin
     *  @param dirP direction signal pin
     *  @param enP enable signal Pin
     *  @param group timer group to use (0 or 1)
     *  @param index which timer to use (0 or 1)
     *  @param microstepping microstepping performed by the driver, used for more accuracy
     *  @param stepsPerRot how many steps it takes for the motor to move 2Pi rads. this can be also used instead of microstepping parameter
     */
    void init(uint8_t, uint8_t, uint8_t, microStepping_t microstep);

    /** @brief initialize GPIO and Timer peripherals, class must be configured beforehand with @attention config()
     */
    void init();

    /** @brief runs motor to relative position in steps
     *  @param relative number of steps to run, negative is reverse
     */
    esp_err_t runPos(int32_t relative);

    /** @brief runs motor to relative position in mm
     *  @param relative number of mm to run, negative is reverse
     */
    esp_err_t runPosMm(int32_t relative);

    // /** @brief sets motor speed
    //  *  @param speed speed in steps per second
    //  *  @param accT acceleration time in ms
    //  *  @param decT deceleration time in ms
    //  */
    // void setSpeed(uint32_t speed, uint16_t accT, uint16_t decT);

    /** @brief sets motor speed and accel in millimeters/second
     *  @param speed speed mm*s^-1
     *  @param accT acceleration speed mm*s^-2
     *  @param accT deceleration speed mm*s^-2
     */
    void setSpeedMm(double speed, double acc, double dec);

    /**
     * @brief Set steps per 1 mm of linear movement
     *
     * @param steps steps needed to move one millimiter
     */
    void setStepsPerMm(double steps);

    /**
     * @brief get steps per 1mm settings
     *
     */
    float getStepsPerMm();

    /** @brief set EN pin 1, stops movement
     */
    void disableMotor();

    /** @brief set EN pin to 0, enables movement
     */
    void enableMotor();

    /** @brief returns current state
     *  @return motor_status enum
     */
    uint8_t getState();

    /** @brief returns current absolute position
     *  @return current absolute postion in steps
     */
    int64_t getPosition();

    /** @brief returns current absolute position
     *  @return current absolute postion in steps
     */
    float getPositionMm();

    /** @brief resets absolute pos to 0
     */
    void resetAbsolute();

    /** @brief
     *
     */
    void runInf(bool direction);

    /** @brief returns current speed in steps per second
     */
    uint16_t getSpeed();


    /** @brief returns current target speed in steps per second
     */
    uint16_t getTargetSpeed();


    /** @brief returns current acceleration time in ms
     */
    float getAcc();

    /** @brief returns current deceleration time in ms
     */
    float getDec();

    /** @brief stops the motor dead, but stays enabled
     */
    void stop();

    void stopSlow();

    /**
     * @return pdTRUE if move finished, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);
};
