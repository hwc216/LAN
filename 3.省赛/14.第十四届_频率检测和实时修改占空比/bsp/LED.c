#include "stm32g4xx.h" // Device header
#include "led.h"

void LED_Init(void) // LED��ʼ������ȫ��
{

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_Disp(uint8_t c) // ��ĳһ���ֵ�
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // ����LEDϨ��lСд��
	HAL_GPIO_WritePin(GPIOC, c << 8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_ON(uint16_t LED_Decide) // ֻ��ĳһ����
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // ����LEDϨ��lСд��
	LED_Decide += 7;
	HAL_GPIO_WritePin(GPIOC, 0x01 << LED_Decide, GPIO_PIN_RESET);
	LED_Decide = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_OFF(uint16_t LED_Decide) // �ر�ĳһ����
{
	uint8_t i;
	i = 0x00;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // ����LEDϨ��lСд��
	LED_Decide += 7;
	HAL_GPIO_WritePin(GPIOC, i << LED_Decide, GPIO_PIN_RESET);
	LED_Decide = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
