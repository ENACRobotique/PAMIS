#include "vision.h"
#include <Arduino.h>



#define XSHUT_SENSOR1 PB0
#define XSHUT_SENSOR2 PB1

#if defined(MAMAMIA)
    VL53L0X sensor_right;
    VL53L0X sensor_left;
    VL53L0X sensor_middle;
#elif defined(MILO)
    VL53L0X sensor_middle;
    VL53L0X sensor_left;
    VL53L1X sensor_right;
#elif defined(MONA)
    VL53L0X sensor_middle;
    VL53L1X sensor_left;
    VL53L1X sensor_right;
#else
    #error "Le PAMI n'est pas défini! (MIA, PAMI1, PAMI2)"
#endif


int Vision::get_dist_left (){
    return sensor_left.readRangeContinuousMillimeters();
}

int Vision::get_dist_right (){
    return sensor_right.readRangeContinuousMillimeters();
}

int Vision::get_dist_middle (){
    return sensor_middle.readRangeContinuousMillimeters();
}

std::array<int,3> Vision::get_all_dist(){
    return {Vision::get_dist_left(),Vision::get_dist_middle(),Vision::get_dist_right()};
}