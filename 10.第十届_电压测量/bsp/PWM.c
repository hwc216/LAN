#include "stm32g4xx.h" // Device header
#include "PWM.h"
#include "main.h"

HAL_StatusTypeDef PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t dutycycle)
{
    // 计算新的比较匹配值
    uint32_t compare = ((htim->Instance->ARR + 1) * dutycycle) / 100;

    // 设置新的比较匹配值
    __HAL_TIM_SET_COMPARE(htim, Channel, compare);

    // 如果需要立即更新PWM输出，则可以调用下面的函数
    // HAL_TIM_PWM_Start(htim, Channel); // 不需要在此处启动，只需设置占空比即可

    return HAL_OK;
}

/* 修改PWM频率
 * htim : 定时器句柄, 例如 &htim3 如果你使用的是 TIM3
 * Channel : 定时器通道, 例如 TIM_CHANNEL_1
 * frequency : 新的频率值
 */
HAL_StatusTypeDef PWM_Set_Frequency(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t frequency)
{
    uint32_t timer_clock = 80000000; // 系统时钟频率为80MHz
    uint32_t prescaler = (timer_clock / frequency) / (htim->Instance->ARR + 1) - 1;

    // 设置预分频器
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(htim, timer_clock / (prescaler + 1) / frequency - 1);

    // 更新定时器设置
    HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

    // 重新启动PWM
    HAL_TIM_PWM_Start(htim, Channel);

    return HAL_OK;
}
