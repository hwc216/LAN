#include "stm32g4xx.h" // Device header
#include "led.h"

void LED_Init(void) // LED初始化即灯全灭
{

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_Disp(uint8_t c) // 打开某一部分灯
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // 所有LED熄灭（l小写）
	HAL_GPIO_WritePin(GPIOC, c << 8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_ON(uint16_t LED_Decide) // 只打开某一个灯
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // 所有LED熄灭（l小写）
	LED_Decide += 7;
	HAL_GPIO_WritePin(GPIOC, 0x01 << LED_Decide, GPIO_PIN_RESET);
	LED_Decide = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_OFF(uint16_t LED_Decide) // 关闭某一个灯
{
	uint8_t i;
	i = 0x00;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET); // 所有LED熄灭（l小写）
	LED_Decide += 7;
	HAL_GPIO_WritePin(GPIOC, i << LED_Decide, GPIO_PIN_RESET);
	LED_Decide = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
