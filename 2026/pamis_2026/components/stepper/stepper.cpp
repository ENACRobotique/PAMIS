#include <stdio.h>
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

bool state = 0;

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
    bsem = xSemaphoreCreateBinary();

    uint64_t mask = (1ULL << conf.stepPin) | (1ULL << conf.dirPin) | (1ULL << conf.enPin); // put output gpio pins in bitmask
    gpio_config_t gpio_conf = {
        // config gpios
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // set the gpios as per gpio_conf
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
    gptimer_register_event_callbacks(timer_handle, &cb_group, this);
}

esp_err_t Stepper::runPos(int32_t nb_steps)
{
    if (nb_steps == 0) {
        return ESP_OK;
    }

    // TODO que faire ?fonction appelée alors que le mouvement précédent n'est pas termminé
    if (ctrl.status > IDLE)
    { // we are running, we need to adjust steps accordingly, for now just stop the movement
        STEP_LOGW("Stepper", "Finising previous move, this command will be ignored");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (ctrl.status == DISABLED) { // if motor is disabled, enable it
        enableMotor();
    }
    ctrl.status = ACC;
    setDir(nb_steps < 0); // set CCW if <0, else set CW

    xSemaphoreTake(bsem, 0);

    ctrl.steps_total = abs(nb_steps);
    ctrl.status = ACC;
    ctrl.n = 1;
    /* Période initiale (step 1) */
    ctrl.T = sqrtf(2.0f / ctrl.acc);

    /* Vitesse max */
    ctrl.T_min = 1.0f / ctrl.v_max;

    /* Steps nécessaires pour atteindre v_max */
    uint32_t n_vmax = (uint32_t)((ctrl.v_max * ctrl.v_max) / (2.0f * ctrl.acc));

    if (2 * n_vmax < ctrl.steps_total) {
        /* Trapézoïdal */
        ctrl.n_acc = n_vmax;
        ctrl.n_dec = n_vmax;
    } else {
        /* Triangulaire */
        ctrl.n_acc = ctrl.steps_total / 2;
        ctrl.n_dec = ctrl.n_acc;
    }


    //calc(abs(relative)); // calculate velocity profile

    alarm_cfg.alarm_count  = ctrl.T * TIMER_F;
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    gptimer_enable(timer_handle);
    return gptimer_start(timer_handle);
}

esp_err_t Stepper::runPosMm(int32_t relative)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("Stepper", "Steps per millimeter not set, cannot move!");
    }
    return runPos(relative * ctrl.stepsPerMm);
}


void Stepper::setSpeedMm(double speed, double acc, double dec)
{
    if (ctrl.stepsPerMm == 0)
    {
        STEP_LOGE("Stepper", "Steps per millimeter not set, cannot set the speed!");
    }
    ctrl.v_max = speed * ctrl.stepsPerMm;
    ctrl.acc = acc * ctrl.stepsPerMm;
    //ctrl.dec = dec * ctrl.stepsPerMm;
    STEP_LOGI("Stepper", "Speed set: v=%02f mm/s t+=%02f s t-=%02f s", speed, acc, dec);
}

void Stepper::setStepsPerMm(double steps)
{
    ctrl.stepsPerMm = steps * conf.miStep;
}

double Stepper::getStepsPerMm()
{
    return ctrl.stepsPerMm / conf.miStep;
}

uint8_t Stepper::getState()
{
    return ctrl.status;
}

uint64_t Stepper::getPosition()
{
    return currentPos;
}

uint64_t Stepper::getPositionMm()
{
    return getPosition() / ctrl.stepsPerMm;
}

void Stepper::resetAbsolute()
{
    currentPos = 0;
}

void Stepper::runInf(bool direction)
{
    
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

float Stepper::getDec()
{
    return ctrl.acc;
    //return ctrl.dec;
}

void Stepper::stop()
{
    if (ctrl.status <= IDLE)
    {
        return;
    }
    // ctrl.runInfinite = false;
    gptimer_stop(timer_handle); // stop the timer
    ctrl.status = IDLE;
    ctrl.steps_done = 0;
    gptimer_disable(timer_handle);
    gpio_set_level((gpio_num_t)conf.stepPin, 0);
    xSemaphoreGive(bsem);
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

//static bool plop = 0;
/* Timer callback, used for generating pulses and calculating speed profile in real time */
bool Stepper::xISR(gptimer_t *timer, const gptimer_alarm_event_data_t *data)
{
    //GPIO.out_w1ts = (1ULL << conf.stepPin);
    GPIO.out1_w1ts.val = 1 << (conf.stepPin - 32);
    
    // add and substract one step

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


    uint32_t steps_left = ctrl.steps_total - ctrl.steps_done;


    switch (ctrl.status)
    {
    case ACC:
        if (ctrl.n >= ctrl.n_acc) {
            ctrl.status = COAST;
            break;
        }

        ctrl.T -= (2.0 * ctrl.T) / (4.0 * ctrl.n + 1);
        if (ctrl.T < ctrl.T_min) {
            ctrl.T = ctrl.T_min;
        }
        ctrl.n++;
        break;

    case COAST:
        if (steps_left <= ctrl.n_dec) {
            ctrl.status = DEC;
        }
    break;
    case DEC:
        ctrl.T += (2.0 * ctrl.T) / (4.0 * ctrl.n + 1);
        ctrl.n--;
        break;
    
    default:
        break;
    }

    if (ctrl.steps_done >= ctrl.steps_total) {
        gptimer_stop(timer_handle); // stop the timer
        ctrl.status = IDLE;
        ctrl.steps_done = 0;
        gptimer_disable(timer_handle);
        //GPIO.out_w1tc = (1ULL << conf.stepPin);
        GPIO.out1_w1tc.val = 1 << (conf.stepPin - 32);
        xSemaphoreGiveFromISR(bsem, NULL);
        return 0;
    }

    GPIO.out1_w1tc.val = 1 << (conf.stepPin - 32);

    alarm_cfg.alarm_count = ctrl.T * TIMER_F;
    gptimer_set_alarm_action(timer_handle, &alarm_cfg);
    return 1;
}

void Stepper::stopSlow() {
    if(ctrl.status == ACC or ctrl.status == COAST) {
        ctrl.status = DEC;
        uint32_t n = ctrl.n < ctrl.n_acc ? ctrl.n: ctrl.n_acc;

        uint32_t v = sqrt(2 * ctrl.acc * n);
        uint32_t n_dec = (uint32_t)((v*v) / (2.0f * ctrl.acc));

        ctrl.steps_total = ctrl.steps_done + n_dec;
    }
}

BaseType_t Stepper::waitFinishedTimeout(TickType_t xTicksToWait) {
    if(ctrl.status == IDLE || ctrl.status == DISABLED) {
        return pdTRUE;
    } else {
        return xSemaphoreTake(bsem, xTicksToWait);
    }
}
