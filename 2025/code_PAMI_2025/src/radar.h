#pragma once
#include "VL53L1X_api.h"
#include <FreeRTOS.h>
#include "semphr.h"

#define NBRADAR 3
#define RF2CENTER 58.69358
#define RL2CENTER 58.33013
#define ANGLERF2RL (M_PI/6)
#define RADARTOROUES 60

typedef void(*radar_cb)();

enum eRadar {
    RADAR_LEFT,
    RADAR_FRONT,
    RADAR_RIGHT,
};

class Radar {
public:
    Radar(VL53L1_Dev_t* radar_left, VL53L1_Dev_t* radar_front, VL53L1_Dev_t* radar_right):
    radar_left(radar_left), radar_front(radar_front), radar_right(radar_right), alert_cb(nullptr) {}

    /**
     * Init sensors.
     * Wire must be already initialized before calling this function
     */
    int init();

    /**
     * Start periodic measurements
     */
    void start();

    void setAlertCallback(radar_cb cb) {
        alert_cb = cb;
    }

    void setAlertDistances(uint16_t dl, uint16_t df, uint16_t dr);

    void radarLoop();

    uint16_t getDistance(enum eRadar sensor_id, uint32_t* timestamp);

private:
    void radar_measure(VL53L1_Dev_t* dev);

    VL53L1_Dev_t* radar_left;
    VL53L1_Dev_t* radar_front;
    VL53L1_Dev_t* radar_right;

    radar_cb alert_cb;
};

extern Radar radar;
