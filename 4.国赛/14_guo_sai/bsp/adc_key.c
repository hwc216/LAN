#include "stm32g4xx.h" // Device header
#include "adc_key.h"
#include "badc.h"


uint8_t Scan_Btn(uint16_t ADC_Value)
{
    // 更新后的阈值计算和判断方法
    const uint16_t noPressThreshold = 4020;

    // 没有按键按下时的情况
    if (ADC_Value >= noPressThreshold)
        return 0; // 假设没有按键按下时ADC值接近或超过4020

    // 计算每个按键的阈值
    uint16_t step = (noPressThreshold / 8);

    for (uint8_t i = 1; i <= 8; i++)
    {
        if (ADC_Value < step * i)
        {
            return i;
        }
    }

    return 0; // 防止任何意外情况，返回0
}

