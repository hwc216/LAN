#ifndef _PWM_H
#define _PWM_H

HAL_StatusTypeDef PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t dutycycle);
HAL_StatusTypeDef PWM_Set_Frequency(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t frequency);

#endif
