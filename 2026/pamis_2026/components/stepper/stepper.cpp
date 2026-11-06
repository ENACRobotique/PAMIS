#include <stdio.h>
#include <cfloat>
#include "stepper.h"

#include "esp_log.h"

#define STEP_DEBUG

#ifdef STEP_DEBUG
#define STEP_LOGI(...) ESP_LOGI(__VA_ARGS__)
#define STEP_LOGW(...) ESP_LOGW(__VA_ARGS__)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#else
#define STEP_LOGI(...) while (0)
#define STEP_LOGW(...) while (0)
#define STEP_LOGE(...) ESP_LOGE(__VA_ARGS__)
#endif

// Periods use unsigned Q18.14. At 1 MHz, one timer tick is one microsecond.
static constexpr uint32_t PERIOD_SCALE = 1U << 14;
static constexpr uint32_t MAX_PERIOD_US = 100000; // 100 ms; 2*T still fits in 32 bits
static constexpr uint32_t PERIOD_MAX = MAX_PERIOD_US * PERIOD_SCALE;
static constexpr int32_t MAX_MOVE_STEPS = 1000000; // also keeps 4*n+1 within 32 bits
// Compensates the first-step error of Austin's approximate recurrence.
// Reference: Microchip AVR446, section 2.3.1 (see the component README).
static constexpr double INITIAL_PERIOD_CORRECTION = 0.676;

// Convert timer ticks to fixed-point (rounded when cast to uint32_t).
#define Q(n) ((n) * PERIOD_SCALE + 0.5)

// Convert fixed-point to whole timer ticks.
#define F(bfp) ((bfp) / PERIOD_SCALE)

static uint32_t periodToFixed(double ticks)
{
    // The setters guarantee a positive speed and acceleration.
    // Limit the resulting period to the timer's supported range: 1 us to 100 ms.
    if (ticks < 1) {
        ticks = 1;
    } else if (ticks > MAX_PERIOD_US) {
        ticks = MAX_PERIOD_US;
    }
    return (uint32_t)Q(ticks);
}


// PUBLIC definitions

Stepper::Stepper()
{
}

void Stepper::config(Stepper_config_t *config)
{
    memcpy(&conf, config, sizeof(conf));
}

void Stepper::disableMotor()
{
    setEn(true);
    STEP_LOGI("Stepper", "Disabled");
    ctrl.status = DISABLED;
}

void Stepper::enableMotor()
{
    setEn(false);
    ctrl.status = IDLE;
    STEP_LOGI("Stepper", "Enabled");
}

void Stepper::init(uint8_t stepP, uint8_t dirP, uint8_t enP, microStepping_t microstepping = MICROSTEP_1)
{
    conf.stepPin = stepP;
    conf.dirPin = dirP;
    conf.enPin = enP;
    conf.miStep = microstepping;
    ctrl.status = 0;
    init();
}

void Stepper::init()
{
    currentPos = 0;
    bsem = xSemaphoreCreateBinary();

    // Set STEP, DIR, and ENABLE pins as OUTPUT.
    uint64_t mask = (1ULL << conf.stepPin) | (1ULL << conf.dirPin) | (1ULL << conf.enPin); // put output gpio pins in bitmask
    gpio_config_t gpio_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    gptimer_config_t timer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_F,
    };

    timer_conf.flags.intr_shared = false;


    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &timer_handle));

    gptimer_event_callbacks_t cb_group;
    cb_group.on_alarm = xISRwrap;
    alarm_cfg.flags.auto_reload_on_alarm = 1;
    ESP_ERROR_CHECK(
        gptimer_register_event_callbacks(timer_handle, &cb_group, this)
    );
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
}

esp_err_t Stepper::runPos(int32_t nb_steps)
{
    if (nb_steps == 0) {
        return ESP_OK;
    }

    // Refuse a new move while the previous one is still active.
    if (ctrl.status > IDLE)
    {
        STEP_LOGW("Stepper", "Finising previous move, this command will be ignored");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Reject oversized moves before abs() and the ramp calculations.
    if (nb_steps < -MAX_MOVE_STEPS || nb_steps > MAX_MOVE_STEPS) {
        STEP_LOGE("Stepper", "Move exceeds one million STEP pulses");
        return ESP_ERR_INVALID_ARG;
    }
    // Compensate the approximation error accumulated by the period recurrence.
    const double initial_period = INITIAL_PERIOD_CORRECTION * sqrt(2.0 / ctrl.acc) * TIMER_F;
    const double min_period = TIMER_F / ctrl.v_max;

    if (ctrl.status == DISABLED) { // if motor is disabled, enable it
        enableMotor();
    }

    ctrl.status = ACC;
    setDir(nb_steps < 0); // set CCW if <0, else set CW

    xSemaphoreTake(bsem, 0);

    ctrl.steps_total = abs(nb_steps);
    ctrl.n = 1;
    // Initial period (step 1)
    // Convert ctrl.T to fixed-point representation, so later operations are more precise
    ctrl.T = periodToFixed(initial_period);

    // period at max speed
    ctrl.T_min = periodToFixed(min_period);
    if (ctrl.T < ctrl.T_min) {
        ctrl.T = ctrl.T_min;
    }

    // number of steps to reach v_max
    const double n_vmax = (ctrl.v_max * ctrl.v_max) / (2 * ctrl.acc);
    // Bound before conversion: also keeps 4*n+1 within uint32_t in the ISR.
    const uint32_t half_steps = ctrl.steps_total / 2;
    if (n_vmax < half_steps) {
        ctrl.n_ramp = (uint32_t)n_vmax;
    } else {
        ctrl.n_ramp = half_steps;
    }

    // Initial period. Convert back from fixed-point to normal number.
    alarm_cfg.alarm_count  = F(ctrl.T);
    esp_err_t err = gptimer_set_raw_count(timer_handle, 0);
    if (err != ESP_OK) {
        ctrl.status = IDLE;
        STEP_LOGE("Stepper", "Failed to reset timer: %s", esp_err_to_name(err));
        return err;
    }

    err = gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    if (err != ESP_OK) {
        ctrl.status = IDLE;
        STEP_LOGE("Stepper", "Failed to set timer alarm: %s", esp_err_to_name(err));
        return err;
    }

    err = gptimer_start(timer_handle);
    if (err != ESP_OK) {
        ctrl.status = IDLE;
        STEP_LOGE("Stepper", "Failed to start timer: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t Stepper::runPosMm(int32_t relative)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("Stepper", "Steps per millimeter not set, cannot move!");
        return ESP_ERR_INVALID_ARG;
    }
    const double steps = relative * ctrl.stepsPerMm;
    // Check before converting double to int32_t.
    if (fabs(steps) > MAX_MOVE_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }
    return runPos((int32_t)steps);
}


void Stepper::setSpeedMm(double speed, double acc)
{
    const double steps_per_mm = ctrl.stepsPerMm;
    const double v_max = speed * steps_per_mm;
    const double acceleration = acc * steps_per_mm;
    if (!isfinite(v_max) || v_max <= 0 ||
        !isfinite(acceleration) || acceleration <= 0) {
        STEP_LOGE("Stepper", "Speed and acceleration must be finite and positive");
        return;
    }
    ctrl.v_max = v_max;
    ctrl.acc = acceleration;
    STEP_LOGI("Stepper", "Speed set: v=%02f mm/s a=%02f mm/s^2", speed, acc);
}

void Stepper::setStepsPerMm(double steps)
{
    const double steps_per_mm = steps * (int)conf.miStep;
    if (!isfinite(steps_per_mm) || steps_per_mm <= 0) {
        STEP_LOGE("Stepper", "Steps per millimeter must be finite and positive");
        return;
    }
    ctrl.stepsPerMm = steps_per_mm;
}

float Stepper::getStepsPerMm()
{
    return ctrl.stepsPerMm / (int)conf.miStep;
}

uint8_t Stepper::getState()
{
    return ctrl.status;
}

int64_t Stepper::getPosition()
{
    portENTER_CRITICAL(&control_lock);
    const int64_t position = currentPos;
    portEXIT_CRITICAL(&control_lock);
    return position;
}

float Stepper::getPositionMm()
{
    return getPosition() / ctrl.stepsPerMm;
}

void Stepper::resetAbsolute()
{
    portENTER_CRITICAL(&control_lock);
    currentPos = 0;
    portEXIT_CRITICAL(&control_lock);
}

uint16_t Stepper::getSpeed()
{
    return ctrl.v_max;
}

uint16_t Stepper::getTargetSpeed()
{
    return ctrl.v_max;
}

float Stepper::getAcc()
{
    return ctrl.acc;
}

void Stepper::emergencyStop()
{
    portENTER_CRITICAL(&control_lock);
    const bool stopped = emergencyStopLocked();
    portEXIT_CRITICAL(&control_lock);

    if (stopped) {
        xSemaphoreGive(bsem);
    }
}

bool Stepper::emergencyStopLocked()
{
    if (ctrl.status <= IDLE)
    {
        return false;
    }
    gptimer_stop(timer_handle); // stop the timer
    ctrl.status = IDLE;
    ctrl.steps_done = 0;
    gpio_set_level((gpio_num_t)conf.stepPin, 0);
    return true;
}

// PRIVATE definitions

void Stepper::setEn(bool state)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.enPin, state));
}

void Stepper::setDir(bool state)
{
    ctrl.dir = state;
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)conf.dirPin, state));
}

/* Timer callback, used for generating pulses and calculating speed profile in real time */
bool Stepper::xISR(gptimer_t *timer, const gptimer_alarm_event_data_t *data)
{
    portENTER_CRITICAL_ISR(&control_lock);
    // An alarm callback may already be pending when stop(0) stops the timer.
    if (ctrl.status == IDLE || ctrl.status == DISABLED) {
        portEXIT_CRITICAL_ISR(&control_lock);
        return 0;
    }
    gpio_ll_set_level(&GPIO, conf.stepPin, 1);
    
    // add one step
    ctrl.steps_done++;

    // update current position
    if (ctrl.dir == CW)
    {
        currentPos++;
    }
    else
    {
        currentPos--;
    }


    // There is no next period to calculate after the final pulse.
    if (ctrl.steps_done >= ctrl.steps_total) {
        gptimer_stop(timer_handle);
        ctrl.status = IDLE;
        ctrl.steps_done = 0;
        gpio_ll_set_level(&GPIO, conf.stepPin, 0);
        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(bsem, &task_woken);
        portEXIT_CRITICAL_ISR(&control_lock);
        return task_woken == pdTRUE;    // return true if some other task was waiting on the semaphore
    }

    uint32_t steps_left = ctrl.steps_total - ctrl.steps_done;

    // Select the next pulse's phase before updating its period.
    // Preserve DEC when braking was requested by stop().
    if (ctrl.status == ACC || ctrl.status == COAST) {
        if (steps_left <= ctrl.n_ramp) {
            ctrl.status = DEC;
        } else if (ctrl.steps_done >= ctrl.n_ramp) {
            ctrl.status = COAST;
        }
    }

    switch (ctrl.status)
    {
    case ACC:
        // Use fixed-point arithmetic to retain sub-tick period changes.
        ctrl.T -= (2 * ctrl.T) / (4 * ctrl.n + 1);
        if (ctrl.T < ctrl.T_min) {
            ctrl.T = ctrl.T_min;
        }
        ctrl.n++;
        break;

    case DEC: {
        const uint32_t period = ctrl.T;
        const uint32_t delta = (2 * period) / (4 * ctrl.n + 1);
        if (delta > PERIOD_MAX - period) {  // if(period+delta > PERIOD_MAX)
            ctrl.T = PERIOD_MAX;
        } else {
            ctrl.T = period + delta;
        }
        if (ctrl.n > 0) {
            ctrl.n--;
        }
        break;
    }

    case COAST:
    default:
        break;
    }

    gpio_ll_set_level(&GPIO, conf.stepPin, 0);

    // convert back from fixed-point to normal value
    alarm_cfg.alarm_count = F(ctrl.T);
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    portEXIT_CRITICAL_ISR(&control_lock);
    return false;   // No high priority task woken
}

float Stepper::stop()
{
    // No distance limit: keep the normal braking ramp selected by stop(distance).
    return stop(DBL_MAX);
}

float Stepper::stop(double braking_distance) {
    const double steps_per_mm = ctrl.stepsPerMm;
    if (!isfinite(braking_distance) || braking_distance < 0 ||
        steps_per_mm == 0) {
        STEP_LOGE("Stepper", "Invalid braking distance or steps per millimeter");
        return 0;
    }
    // Update the remaining distance and recurrence index as one operation
    // relative to the pulse ISR.
    portENTER_CRITICAL(&control_lock);
    if (ctrl.status <= IDLE) {
        portEXIT_CRITICAL(&control_lock);
        return 0;
    }

    const uint32_t steps_done = ctrl.steps_done;
    const uint32_t steps_left = ctrl.steps_total - steps_done;
    uint32_t planned_braking_steps;
    if (ctrl.status == ACC) {
        planned_braking_steps = ctrl.n;
    } else if (ctrl.status == COAST) {
        planned_braking_steps = ctrl.n_ramp;
    } else {
        // A braking ramp is already in progress: keep its current target.
        planned_braking_steps = steps_left;
    }
    if (planned_braking_steps > steps_left) {
        planned_braking_steps = steps_left;
    }

    // braking_distance is a maximum: never make the normal braking ramp longer.
    uint32_t braking_steps = planned_braking_steps;
    if (braking_distance < planned_braking_steps / steps_per_mm) {
        braking_steps = (uint32_t)ceil(braking_distance * steps_per_mm);
    }

    double removed_distance = (steps_left - braking_steps) / steps_per_mm;
    if (ctrl.dir == CCW) {
        removed_distance = -removed_distance;
    }
    if (braking_steps == 0) {
        const bool stopped = emergencyStopLocked();
        portEXIT_CRITICAL(&control_lock);
        if (stopped) {
            xSemaphoreGive(bsem);
        }
        return removed_distance;
    }

    // Keep the current period: no instantaneous speed change. A shorter
    // recurrence index produces a steeper slowdown over fewer pulses.
    ctrl.n = braking_steps;
    ctrl.steps_total = steps_done + braking_steps;
    ctrl.status = DEC;
    portEXIT_CRITICAL(&control_lock);
    return removed_distance;
}

BaseType_t Stepper::waitFinishedTimeout(TickType_t xTicksToWait) {
    if(ctrl.status == IDLE || ctrl.status == DISABLED) {
        return pdTRUE;
    } else {
        return xSemaphoreTake(bsem, xTicksToWait);
    }
}
