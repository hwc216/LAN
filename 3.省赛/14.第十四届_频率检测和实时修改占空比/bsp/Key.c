#include "key.h"
#include "lcd.h"

#define LONG_PRESS_TIME 100	 // ���峤����ʱ����ֵ
#define DOUBLE_PRESS_INTERVAL 20 // ����˫�����ʱ����ֵ

uint8_t judge_state[4] = {0}, double_click_time[4] = {0}, key_state[4] = {0}, double_click_timerEN[4] = {0},
		single_key_flag[4] = {100}, double_key_flag[4] = {0}, long_key_flag[4] = {0};
uint16_t key_time[4] = {0};

// �����ж�ʹ�õ��Ƕ�ʱ��4
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // �жϻص�����
{
	if (htim->Instance == TIM4) // ��ʱ��4���ж��¼�
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
			{ // ��������
				if (judge_state[i] == 0)
				{
					// �ռ�⵽��������
					judge_state[i] = 1;
					key_time[i] = 0;
				}
				else if (judge_state[i] == 1)
				{
					// ȷ�ϰ���ȷʵ�����£��������
					judge_state[i] = 2;
				}
			}
			else
			{ // ����δ���»��ͷ�
				if (judge_state[i] == 2)
				{
					if (key_time[i] < LONG_PRESS_TIME)
					{
						// �������ͷţ���δ�ﵽ����ʱ��
						double_click_timerEN[i] ^= 1;
						if (double_click_timerEN[i])
						{
							// ��һ�μ�⵽������˫��
							double_click_time[i] = 0;
						}
						else
						{
							// ��⵽�ڶ��ΰ��£�ȷ��Ϊ˫��
							double_key_flag[i] = 1;
						}
					}
					else
					{
						// �������ͷţ��Ҵﵽ����ʱ�䣬ȷ��Ϊ����
						long_key_flag[i] = 1;
					}
					judge_state[i] = 0;
				}
				else if (double_click_timerEN[i] && ++double_click_time[i] >= DOUBLE_PRESS_INTERVAL)
				{
					// ȷ��Ϊ����
					single_key_flag[i] = 1;
					double_click_timerEN[i] = 0;
				}
			}

			// �����ǰ״̬�ǰ���״̬�����Ӱ���ʱ��
			if (judge_state[i] == 2)
			{
				key_time[i]++;
			}
		}
	}
}

// /*****�����ж�ʹ�õ��Ƕ�ʱ��4*******/

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // �жϻص�����
// {
// 	if (htim->Instance == TIM4) // ��ʱ��4���ж��¼�
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
// 				if (key_state[i] == 0) // ��������
// 				{
// 					judge_state[i] = 1;
// 					key_time[i] = 0;
// 				}
// 				break;
// 			}
// 			case 1: // ��������
// 			{
// 				if (key_state[i] == 0)
// 				{
// 					judge_state[i] = 2;
// 				}
// 				else
// 					judge_state[i] = 0; // δ����
// 				break;
// 			}
// 			case 2:
// 			{
// 				if ((key_state[i] == 1) && key_time[i] < 70) // �ȴ��ɿ�����,�ҷǳ�����
// 				{
// 					if (double_click_timerEN[i] == 0) // ����˫�������ĵ�һ�Σ������ʱ
// 					{
// 						double_click_timerEN[i] = 1;
// 						double_click_time[i] = 0;
// 					}
// 					else // �ڼ�ʱ��Χ���ְ���һ��
// 					{
// 						double_key_flag[i] = 1; // ˫�����
// 						double_click_timerEN[i] = 0;
// 					}
// 					judge_state[i] = 0;
// 				}
// 				else if (key_state[i] == 1 && key_time[i] >= 70)
// 					judge_state[i] = 0; // �ɿ����ǳ�����
// 				else
// 				{
// 					if (key_time[i] >= 70)
// 						long_key_flag[i] = 1; // ������
// 					key_time[i]++;			  // ��������ʱ ��û�ɿ�
// 				}
// 				break;
// 			}
// 			}
// 			if (double_click_timerEN[i] == 1) // ��ʱȷ���Ƿ�˫��
// 			{
// 				double_click_time[i]++;
// 				if (double_click_time[i] >= 20)
// 				{
// 					single_key_flag[i] = 1; // ����1���ΰ���
// 					double_click_timerEN[i] = 0;
// 				}
// 			}
// 		}
// 	}
// }
