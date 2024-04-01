#ifndef _badc_h
#define _badc_h

#include "main.h"
#include "adc.h"

typedef struct
{
    uint16_t AdcChannel13Value; // PA5, ADC2 Channel 13 //拓展版ADC_KEY ADC值 或者拓展版右可调电阻
    uint16_t AdcChannel17Value; // PA4, ADC2 Channel 17 //拓展版光敏电阻值    或者拓展版左可调电阻 
} ADC2_ChannelValues;

double Get_ADC(ADC_HandleTypeDef *pin);
double Get_ADC_Average(ADC_HandleTypeDef *hadc, uint8_t samples);
ADC2_ChannelValues Read_ADC2_Channels(void);
double Return_3_3(uint16_t ADC_Value);

#endif
