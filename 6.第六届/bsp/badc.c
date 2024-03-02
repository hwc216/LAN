#include "stm32g4xx.h" // Device header
#include "badc.h"

double Get_ADC(ADC_HandleTypeDef *pin)
{
   uint16_t adc;
   HAL_ADC_Start(pin);
   adc = HAL_ADC_GetValue(pin);
   return adc * 3.3 / 4096;
}

double Get_ADC_Average(ADC_HandleTypeDef *hadc, uint8_t samples)
{
   uint32_t adc_accumulated = 0; // 用于累加ADC值
   uint16_t adc_value;

   // 清除之前的ADC值
   HAL_ADC_Start(hadc); // 启动ADC

   // 获取指定次数的ADC值并累加
   for (uint8_t i = 0; i < samples; ++i)
   {
      adc_value = HAL_ADC_GetValue(hadc); // 读取ADC值
      adc_accumulated += adc_value;       // 累加到总和中
   }

   // 计算平均值并返回
   return (double)adc_accumulated / samples * 3.3 / 4096;
}

// 使用该函数时，可以这样调用：
// double adc_avg = Get_ADC_Average(&hadc1, 10);
