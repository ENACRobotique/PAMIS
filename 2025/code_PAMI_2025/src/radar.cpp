#include "radar.h"
#include "VL53L1X_api.h"
#include "config.h"
#include "Wire.h"
#include <FreeRTOS.h>
#include "task.h"
#include <Arduino.h>


#define RADAR_LEFT_ADDR 0x32
#define RADAR_FRONT_ADDR 0x31
#define RADAR_RIGHT_ADDR 0x30

VL53L1_Dev_t vl53_left = {
    .addr = VL53L1_DEFAULT_ADDR,
    .shutdown_pin = SHUTDOWN3,
    .wire = &Wire,
    .alert_dist = 200,
    .last_dist = 0,
    .timestamp = 0,
};
VL53L1_Dev_t vl53_front = {
    .addr = VL53L1_DEFAULT_ADDR,
    .shutdown_pin = SHUTDOWN2,
    .wire = &Wire,
    .alert_dist = 200,
    .last_dist = 0,
    .timestamp = 0,
};
VL53L1_Dev_t vl53_right = {
    .addr = VL53L1_DEFAULT_ADDR,
    .shutdown_pin = SHUTDOWN1,
    .wire = &Wire,
    .alert_dist = 200,
    .last_dist = 0,
    .timestamp = 0,
};

Radar radar(&vl53_left, &vl53_front, &vl53_right);

// Radar radar(&vl53_left,&vl53_front, NULL);

static void radar_run( void *arg );
static uint8_t sensor_get_distance(VL53L1_Dev_t* dev, uint16_t* distance);
static VL53L1X_ERROR sensor_init(VL53L1_Dev_t* dev, uint8_t new_addr);



int Radar::init()
{
    //turn off sensors
    if(radar_left) {
        pinMode(radar_left->shutdown_pin, OUTPUT);
        digitalWrite(radar_left->shutdown_pin , LOW);
    }
    #if not defined(JOHNNY)
    if(radar_front) {
        pinMode(radar_front->shutdown_pin, OUTPUT);
        digitalWrite(radar_front->shutdown_pin , LOW);
    }
    #endif
    if(radar_right) {
        pinMode(radar_right->shutdown_pin, OUTPUT);
        digitalWrite(radar_right->shutdown_pin , LOW);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));

    VL53L1X_ERROR status = 0;

    if(radar_left) {
        status |= sensor_init(radar_left, RADAR_LEFT_ADDR);
    }
    #if not defined(JOHNNY)
    if(radar_front) {
        status |= sensor_init(radar_front, RADAR_FRONT_ADDR);
    }
    #endif
    if(radar_right) {
        status |= sensor_init(radar_right, RADAR_RIGHT_ADDR);
    }

    return status;
}

void Radar::start()
{
    TaskHandle_t xHandle;
    xTaskCreate(
        radar_run, "radar_run", configMINIMAL_STACK_SIZE*2,
        &radar, tskIDLE_PRIORITY + 2, &xHandle );
}

static void radar_run( void *arg ) {
    Radar* radar = (Radar*) arg;
    while(true) {
        radar->radarLoop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void Radar::radarLoop()
{
    bool alert = false;

    if(radar_left) {
        radar_measure(radar_left);
        alert |= radar_left->last_dist < radar_left->alert_dist;
    }
    #if not defined(JOHNNY)
    if(radar_front) {
        radar_measure(radar_front);
        alert |= radar_front->last_dist < radar_front->alert_dist;
    }
    #endif
    if(radar_right) {
        radar_measure(radar_right);
        alert |= radar_right->last_dist < radar_right->alert_dist;
    }

    if(alert && alert_cb) {
        alert_cb();
    }

    // Serial.printf("gauche : %u, mid : %u, droite : %u \n", radar.getDistance(RADAR_LEFT, NULL), radar.getDistance(RADAR_FRONT, NULL), radar.getDistance(RADAR_RIGHT, NULL));
    //Serial.print(locomotion.etat);
}

void Radar::radar_measure(VL53L1_Dev_t* dev)
{
    uint16_t distance;
    uint8_t ret = sensor_get_distance(dev, &distance);
    if(ret == 0) {
        dev->last_dist = distance;
        dev->timestamp = xTaskGetTickCount();
    } else if(ret == 4) {
        // crazy long distance
        dev->last_dist = 10000;
        dev->timestamp = xTaskGetTickCount();
    } else {
        // Serial.printf("VL53L1 ranging error: %d\n", ret);
        dev->last_dist = 10000;
     //   dev->timestamp = xTaskGetTickCount();

    }
}



uint16_t Radar::getDistance(enum eRadar sensor_id, uint32_t* timestamp)
{
    switch (sensor_id)
    {
    case RADAR_LEFT:
        if(timestamp) {*timestamp = radar_left->timestamp;}
        return radar_left->last_dist;
        break;
    
    #if not defined(JOHNNY)
    case RADAR_FRONT:
    if(timestamp) {*timestamp = radar_front->timestamp;}
    return radar_front->last_dist;
        break;
    #endif
    case RADAR_RIGHT:
        if(timestamp) {*timestamp = radar_right->timestamp;}
        return radar_right->last_dist;
        break;
    
    default:
        return 1<<16-1;
        break;
    }
    return 0;
}




void Radar::setAlertDistances(uint16_t dl, uint16_t df, uint16_t dr) {
    if(radar_left) {
        radar_left->alert_dist = dl;
    }
    #if not defined(JOHNNY)
    if(radar_front) {
        radar_front->alert_dist = df;
    }
    #endif
    if(radar_right) {
        radar_right->alert_dist = dr;
    }
}

static VL53L1X_ERROR sensor_init(VL53L1_Dev_t* dev, uint8_t new_addr) {
    // turn ON sensor
    digitalWrite(dev->shutdown_pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t sensorState = 0;
    VL53L1X_ERROR status;
    while(sensorState==0) {
        status = VL53L1X_BootState(dev, &sensorState);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Chip booted

    /* This function must to be called to initialize the sensor with the default setting  */
    status = VL53L1X_SensorInit(dev);

    if(!VL53L1X_SetI2CAddress(dev, new_addr*2)) {
        dev->addr = new_addr;
    }

    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
    status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
    status = VL53L1X_SetInterMeasurementInMs(dev, 100); /* in ms, IM must be > = TB */
    // status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
    // status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
    // status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
    // status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    
    status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
    return status;
}


static uint8_t sensor_get_distance(VL53L1_Dev_t* dev, uint16_t* distance) {
    uint8_t dataReady = 0;
    VL53L1X_ERROR status;
    VL53L1X_ERROR status2;
    VL53L1X_ERROR status3;
    VL53L1X_ERROR status4;
    VL53L1X_ERROR status5;
    VL53L1X_ERROR status6;


    TickType_t time_start = xTaskGetTickCount();

    while(pdTICKS_TO_MS(xTaskGetTickCount() - time_start) < 20) {
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        if(!status && dataReady) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if(!dataReady) {
        return -1;
    }

    if(status)
        Serial.printf("data pourrie, %d", status);

    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum; 
    uint8_t RangeStatus;

    status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
    status2 = VL53L1X_GetDistance(dev, distance);
    status3 = VL53L1X_GetSignalRate(dev, &SignalRate);
    status4 = VL53L1X_GetAmbientRate(dev, &AmbientRate);
    status5 = VL53L1X_GetSpadNb(dev, &SpadNum);
    status6 = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
    //Serial.printf("RangeStatus%d Distance%d SignalRate%d  AmbientRate%d SpadNb%d\n", status, status2, status3, status4, status5);
    // if(RangeStatus == 0)
    //     Serial.printf("D:%d\t SR:%d\tAR: %d\tSN:%d\tCI: %d\n", *distance, SignalRate, AmbientRate, SpadNum, status6);


 
    return RangeStatus; 
}
