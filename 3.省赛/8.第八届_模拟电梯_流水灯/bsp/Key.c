#include "key.h"
#include "lcd.h"
uint8_t judge_state[4] = {0}, double_click_time[4] = {0}, key_state[4] = {0}, double_click_timerEN[4] = {0},
		single_key_flag[4] = {100}, double_key_flag[4] = {0}, long_key_flag[4] = {0};
uint16_t key_time[4] = {0};

uint8_t long_press_rate_limiter[4] = {0}; // 减速因子，用于控制长按时PA6_duty减少的速度

void HandleLongPress(int index, uint8_t state)
{
	// 如果当前按键是长按状态
	if (state == 0 && key_time[index] >= 70)
	{
		long_key_flag[index] = 1; // 设置长按标志位
	}
	// 如果当前按键从长按状态释放
	else if (state == 1 && long_key_flag[index] == 1)
	{
		long_key_flag[index] = 0; // 清除长按标志位
	}
}

// /*****按键中断使用的是定时器4*******/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 中断回调函数
{
	if (htim->Instance == TIM4) // 定时器4的中断事件
	{
		uint8_t read_keys[] = {
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0),
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1),
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2),
			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)};

		for (int i = 0; i < 4; i++)
		{
			key_state[i] = read_keys[i];

			if (key_state[i] == 0)
			{ // 按键按下
				if (judge_state[i] == 0)
				{
					// 刚检测到按键按下
					judge_state[i] = 1;
					key_time[i] = 0;
				}
				else if (judge_state[i] == 1)
				{
					// 确认按键确实被按下，消抖完成
					judge_state[i] = 2;
				}
			}
			else
			{ // 按键未按下或释放
				if (judge_state[i] == 2)
				{
					if (key_time[i] < 70)
					{
						// 按键被释放，且未达到长按时间
						double_click_timerEN[i] ^= 1;
						if (double_click_timerEN[i])
						{
							// 第一次检测到可能是双击
							double_click_time[i] = 0;
						}
						else
						{
							// 检测到第二次按下，确认为双击
							double_key_flag[i] = 1;
						}
					}
					else
					{
						// 按键被释放，且达到长按时间，确认为长按
						long_key_flag[i] = 1;
					}
					judge_state[i] = 0;
				}
				else if (double_click_timerEN[i] && ++double_click_time[i] >= 20)
				{
					// 确认为单击
					single_key_flag[i] = 1;
					double_click_timerEN[i] = 0;
				}
			}

			// 如果当前状态是按下状态，增加按键时间
			if (judge_state[i] == 2)
			{
				key_time[i]++;
			}
		}

		// 新增的长按处理逻辑
		for (int i = 0; i < 4; i++)
		{
			// 调用HandleLongPress函数处理每个按键的长按状态
			HandleLongPress(i, key_state[i]);
		}
	}
}

// /*****按键中断使用的是定时器4*******/

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 中断回调函数
// {
// 	if (htim->Instance == TIM4) // 定时器4的中断事件
// 	{
// 		key_state[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
// 		key_state[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
// 		key_state[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
// 		key_state[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
// 		for (int i = 0; i < 4; i++)
// 		{
// 			switch (judge_state[i])
// 			{
// 			case 0:
// 			{
// 				if (key_state[i] == 0) // 按键按下
// 				{
// 					judge_state[i] = 1;
// 					key_time[i] = 0;
// 				}
// 				break;
// 			}
// 			case 1: // 消抖过程
// 			{
// 				if (key_state[i] == 0)
// 				{
// 					judge_state[i] = 2;
// 				}
// 				else
// 					judge_state[i] = 0; // 未按下
// 				break;
// 			}
// 			case 2:
// 			{
// 				if ((key_state[i] == 1) && key_time[i] < 70) // 等待松开过程,且非长按键
// 				{
// 					if (double_click_timerEN[i] == 0) // 可能双击按键的第一次，进入计时
// 					{
// 						double_click_timerEN[i] = 1;
// 						double_click_time[i] = 0;
// 					}
// 					else // 在计时范围内又按了一次
// 					{
// 						double_key_flag[i] = 1; // 双击情况
// 						double_click_timerEN[i] = 0;
// 					}
// 					judge_state[i] = 0;
// 				}
// 				else if (key_state[i] == 1 && key_time[i] >= 70)
// 					judge_state[i] = 0; // 松开且是长按键
// 				else
// 				{
// 					if (key_time[i] >= 70)
// 						long_key_flag[i] = 1; // 长按键
// 					key_time[i]++;			  // 长按键计时 还没松开
// 				}
// 				break;
// 			}
// 			}
// 			if (double_click_timerEN[i] == 1) // 延时确认是否双击
// 			{
// 				double_click_time[i]++;
// 				if (double_click_time[i] >= 20)
// 				{
// 					single_key_flag[i] = 1; // 按键1单次按下
// 					double_click_timerEN[i] = 0;
// 				}
// 			}
// 		}
// 	}
// }
