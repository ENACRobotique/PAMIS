#if defined(STM32F4xx) || defined(STM32L432xx)
#include "TimerField.h"
int TimerField::instances = 0;
TIM_TypeDef* TimerField::timer_mapping[MAX_TIMERS] = { TIM1, TIM2, TIM6, TIM7, TIM15, TIM16 };
// TIM3 and TIM13 used by HAL/FreeRTOS? doesn't work well to use.
#endif
