#ifndef KEY_H
#define KEY_H

#include "main.h"
uint16_t KEY_Scan(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // 中断回调函数

#endif
