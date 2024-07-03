#pragma once

#include <VL53L0X.h>
#include <VL53L1X.h>


#if defined(MAMAMIA)
    extern VL53L0X sensor_right;
    extern VL53L0X sensor_left;
    extern VL53L0X sensor_middle;
#elif defined(MILO)
    extern VL53L0X sensor_middle;
    extern VL53L0X sensor_left;
    extern VL53L1X sensor_right;
#elif defined(MONA)
    extern VL53L0X sensor_middle;
    extern VL53L1X sensor_left;
    extern VL53L1X sensor_right;
#else
    #error "Le PAMI n'est pas défini! (MIA, PAMI1, PAMI2)"
#endif


class Vision{
    private:

    public:
    int get_dist_left();
    int get_dist_right();
    int get_dist_middle();
    std::array<int,3> get_all_dist();
};