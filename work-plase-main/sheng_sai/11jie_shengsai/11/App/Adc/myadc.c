#include "myadc.h"


/*******************************************************************
* ��������:��ȡADC��ֵ
* ����������
* 			ADC_HandleTypeDef *hadc��ADC��ͨ��ֵ
* ��������ֵ��
* 			double:ת�����ADCֵ
*******************************************************************/
double getADC(ADC_HandleTypeDef *hadc)
{
	unsigned int value = 0;
	//����ת��ADC���һ�ȡֵ
	HAL_ADC_Start(hadc);
	value = HAL_ADC_GetValue(hadc);
	//ADCֵ��ת�� 3.3V�ǵ�ѹ 4096��ADC�ľ���Ϊ12λҲ����2^12=4096
	return value*3.3/4096;
}
