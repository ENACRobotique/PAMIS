
#pragma once

#include <limits>
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
} Stepper_config_t;

typedef struct
{
    uint32_t steps_done = 0;   // step counter
    uint32_t steps_total = 0; // steps we need to take
    double v_max = 500;      // speed in step/s
    double acc = 1000;        // acceleration in step*second^-2
    uint8_t status = DISABLED;
    bool dir = CW;
    double stepsPerMm = 0; /** Steps per one milimiter, if the motor is performing linear movement */
    
    uint32_t n;
    uint32_t n_ramp;     // nombre de pas prévu pour chacune des deux rampes
    uint32_t T;                 // période courante en ticks, format Q18.14
    uint32_t T_min;             // période minimale en ticks, format Q18.14
} ctrl_var_t;

class Stepper
{
private:
    Stepper_config_t conf;
    volatile ctrl_var_t ctrl;
    portMUX_TYPE control_lock = portMUX_INITIALIZER_UNLOCKED;
    gptimer_handle_t timer_handle;  /** timer handle */
    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0
    };
    int64_t currentPos = 0; // absolute position, protected by control_lock

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

    /** @brief immediate stop; control_lock must already be held
     *  @return true if an active movement was stopped
     */
    bool emergencyStopLocked();

    /** @brief static wrapper for ISR function
     *  @param _this Stepper* this pointer
     *  @return bool
     */
    static bool xISRwrap(gptimer_t* timer, const gptimer_alarm_event_data_t* data,void *_this)
    {
        return static_cast<Stepper *>(_this)->xISR(timer,data);
    }

    bool xISR(gptimer_t* timer, const gptimer_alarm_event_data_t* data);

public:
    /** @brief Constructor - conf variables to be passed later
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
     *  @param microstepping microstepping performed by the driver, used for more accuracy
     */
    void init(uint8_t stepP, uint8_t dirP, uint8_t enP, microStepping_t microstepping);

    /** @brief initialize GPIO and Timer peripherals, class must be configured beforehand with @attention config()
     */
    void init();

    /** @brief runs motor to relative position in steps
     *  @param relative number of STEP pulses, negative is reverse; maximum magnitude 1000000
     */
    esp_err_t runPos(int32_t relative);

    /** @brief runs motor to relative position in mm
     *  @param relative number of mm to run, negative is reverse; limited to 1000000 STEP pulses
     */
    esp_err_t runPosMm(int32_t relative);

    /** @brief sets target speed and shared acceleration/deceleration
     *  @param speed speed mm*s^-1
     *  @param acc acceleration and deceleration mm*s^-2
     *  @note Unset conversion factor or non-finite/non-positive converted values
     *        are logged and ignored; both previous settings are preserved.
     */
    void setSpeedMm(double speed, double acc);

    /**
     * @brief Set full motor steps per mm, before microstepping
     *
     * @param steps full motor steps needed to move one millimeter
     * @note The value after microstepping must be finite and positive;
     *       otherwise the error is logged and the previous setting is preserved.
     */
    void setStepsPerMm(double steps);

    /**
     * @brief get full motor steps per mm, before microstepping
     *
     */
    float getStepsPerMm();

    /** @brief set EN high and mark disabled; does not stop an active timer
     *  @attention Call only when no movement is active.
     */
    void disableMotor();

    /** @brief set EN low and mark idle
     *  @attention Call only when no movement is active.
     */
    void enableMotor();

    /** @brief returns current state
     *  @return motor_status enum
     */
    uint8_t getState();

    /** @brief returns current absolute position
     *  @return counted position in STEP pulses
     */
    int64_t getPosition();

    /** @brief returns current absolute position
     *  @return counted position in millimeters
     */
    float getPositionMm();

    /** @brief resets absolute pos to 0
     */
    void resetAbsolute();

    /** @brief returns target speed in STEP pulses per second, converted to uint16_t
     */
    uint16_t getSpeed();


    /** @brief returns target speed in STEP pulses per second, converted to uint16_t
     */
    uint16_t getTargetSpeed();


    /** @brief returns shared acceleration/deceleration in STEP pulses per second squared
     */
    float getAcc();

    /** @brief stops the motor dead, but stays enabled
     */
    void emergencyStop();

    /** @brief brakes within the requested maximum distance, including when already in DEC
     *  @param braking_distance optional non-negative maximum distance in mm, independent of direction;
     *         rounded up to a STEP pulse. A shorter planned braking ramp is preserved.
     *         Zero requests an immediate stop. When omitted, the normal braking ramp is used.
     *  @return signed distance removed from the original target, in mm;
     *          zero if inactive or arguments are invalid
     */
    float stop(double braking_distance = std::numeric_limits<double>::max());

    /**
     * @param xTicksToWait timeout in FreeRTOS ticks; a timeout does not stop the motor
     * @return pdTRUE if move finished or motor inactive, or pdFALSE on timeout
     */
    BaseType_t waitFinishedTimeout(TickType_t xTicksToWait);
};
