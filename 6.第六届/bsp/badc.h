#ifndef _badc_h
#define _badc_h

#include "main.h"

double Get_ADC(ADC_HandleTypeDef *pin);
double Get_ADC_Average(ADC_HandleTypeDef *hadc, uint8_t samples);

#endif
