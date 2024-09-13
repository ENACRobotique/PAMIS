#pragma once

#include <STM32FreeRTOS.h>

inline void sleep(int duration){
    vTaskDelay(pdMS_TO_TICKS(duration));
}

void vTask_Deplacement (void* param);