#include "key.h"
#include "lcd.h"

#define LONG_PRESS_TIME 250		 // 定义长按键时间阈值
#define DOUBLE_PRESS_INTERVAL 20 // 定义双击间隔时间阈值

extern uint8_t ADC_Key; // ADC_KEY按键值

uint8_t judge_state[4] = {0}, double_click_time[4] = {0}, key_state[4] = {0}, double_click_timerEN[4] = {0},
		single_key_flag[4] = {100}, double_key_flag[4] = {0}, long_key_flag[4] = {0};
uint16_t key_time[4] = {0};

uint8_t judge_ADC_state[8] = {0}, double_ADC_click_time[8] = {0}, key_ADC_state[8] = {0}, double_ADC_click_timerEN[8] = {0},
		single_ADC_key_flag[8] = {100}, double_ADC_key_flag[8] = {0}, long_ADC_key_flag[8] = {0};
uint16_t key_ADC_time[8] = {0};

void setReadKeys(uint8_t read_keys[], int i) // ADC按键使用函数
{
	// 先将所有位设置为1
	for (int j = 0; j < 8; ++j)
	{
		read_keys[j] = 1;
	}

	// 然后根据i的值将对应的位设置为0
	// i的值从1到8，所以要减1来得到正确的索引
	if (i >= 1 && i <= 8)
	{
		read_keys[i - 1] = 0;
	}
}

// 按键中断使用的是定时器4
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
					if (key_time[i] < LONG_PRESS_TIME)
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
				else if (double_click_timerEN[i] && ++double_click_time[i] >= DOUBLE_PRESS_INTERVAL)
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
	}

	if (htim->Instance == TIM6) // 定时器6的中断事件
	{
		uint8_t read_keys[8];
		setReadKeys(read_keys, ADC_Key);

		for (int i = 0; i < 8; i++)
		{
			key_ADC_state[i] = read_keys[i];

			if (key_ADC_state[i] == 0)
			{ // 按键按下
				if (judge_ADC_state[i] == 0)
				{
					// 刚检测到按键按下
					judge_ADC_state[i] = 1;
					key_ADC_time[i] = 0;
				}
				else if (judge_ADC_state[i] == 1)
				{
					// 确认按键确实被按下，消抖完成
					judge_ADC_state[i] = 2;
				}
			}
			else
			{ // 按键未按下或释放
				if (judge_ADC_state[i] == 2)
				{
					if (key_ADC_time[i] < LONG_PRESS_TIME)
					{
						// 按键被释放，且未达到长按时间
						double_ADC_click_timerEN[i] ^= 1;
						if (double_ADC_click_timerEN[i])
						{
							// 第一次检测到可能是双击
							double_ADC_click_time[i] = 0;
						}
						else
						{
							// 检测到第二次按下，确认为双击
							double_ADC_key_flag[i] = 1;
						}
					}
					else
					{
						// 按键被释放，且达到长按时间，确认为长按
						long_ADC_key_flag[i] = 1;
					}
					judge_ADC_state[i] = 0;
				}
				else if (double_ADC_click_timerEN[i] && ++double_ADC_click_time[i] >= DOUBLE_PRESS_INTERVAL)
				{
					// 确认为单击
					single_ADC_key_flag[i] = 1;
					double_ADC_click_timerEN[i] = 0;
				}
			}

			// 如果当前状态是按下状态，增加按键时间
			if (judge_ADC_state[i] == 2)
			{
				key_ADC_time[i]++;
			}
		}
	}
}

// //按键长按数字递增写法
//   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 && key_time[3] > 40) // 按键4长按
//   {
// 	long_key_flag[3] = 0;
//   }

// ADC按键操作函数

// if (single_ADC_key_flag[0] == 1) // 切换主界面与参数设置界面
// {
// 	LCD_Disp_Change ^= 0x01;

// 	// 回到主界面时操作
// 	if (LCD_Disp_Change == 0)
// 	{
// 		LCD_Line_Change = 0;
// 	}

// 	LCD_Clear(Black);
// 	single_ADC_key_flag[0] = 0;
// }

// if (single_ADC_key_flag[1] == 1) // 按键2
// {
// 	if (LCD_Disp_Change == 1) // 在参数界面有效
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// 修改第一行数据  加0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// 修改第二行数据  加0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// 修改第三行数据  加0.01
// 		}
// 	}
// 	single_ADC_key_flag[1] = 0;
// }

// if (single_ADC_key_flag[2] == 1) // 按键3
// {
// 	if (LCD_Disp_Change == 1) // 在参数界面有效
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// 修改第一行数据  减0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// 修改第二行数据  减0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// 修改第三行数据  减0.01
// 		}
// 	}
// 	single_ADC_key_flag[2] = 0;
// }

// if (single_ADC_key_flag[3] == 1) // 按键4
// {

// 	if (LCD_Disp_Change)
// 	{
// 		LCD_Line_Change++; // 选择高亮显示行
// 		if (LCD_Line_Change == 4)
// 		{
// 			LCD_Line_Change = 0;
// 		}
// 	}
// 	single_ADC_key_flag[3] = 0;
// }

// if (single_ADC_key_flag[4] == 1) // 选择主界面显示商品1 //按键5
// {
// 	single_ADC_key_flag[4] = 0;
// }

// if (single_ADC_key_flag[5] == 1) // 选择主界面显示商品2  //按键6
// {
// 	single_ADC_key_flag[5] = 0;
// }

// if (single_ADC_key_flag[6] == 1) // 选择主界面显示商品3  //按键7
// {
// 	single_ADC_key_flag[6] = 0;
// }

// if (single_ADC_key_flag[7] == 1) // 按键8
// {

// 	single_ADC_key_flag[7] = 0;
// }

// if (ADC_Key == 2 && key_ADC_time[1] > 40) // ADC按键2长按递增
// {
// 	if (LCD_Disp_Change == 1) // 在参数界面有效
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// 修改第一行数据  加0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// 修改第二行数据  加0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// 修改第三行数据  加0.01
// 		}
// 	}
// 	long_ADC_key_flag[1] = 0;
// }

// if (ADC_Key == 3 && key_ADC_time[2] > 40) // ADC按键3长按递增
// {
// 	if (LCD_Disp_Change == 1) // 在参数界面有效
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// 修改第一行数据
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// 修改第二行数据
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// 修改第三行数据
// 		}
// 	}
// 	long_ADC_key_flag[2] = 0;
// }
