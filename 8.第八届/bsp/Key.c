#include "key.h"
#include "lcd.h"
uint8_t judge_state[4] = {0}, double_click_time[4] = {0}, key_state[4] = {0}, double_click_timerEN[4] = {0},
		single_key_flag[4] = {100}, double_key_flag[4] = {0}, long_key_flag[4] = {0};
uint16_t key_time[4] = {0};

uint8_t long_press_rate_limiter[4] = {0}; // �������ӣ����ڿ��Ƴ���ʱPA6_duty���ٵ��ٶ�

void HandleLongPress(int index, uint8_t state)
{
	// �����ǰ�����ǳ���״̬
	if (state == 0 && key_time[index] >= 70)
	{
		long_key_flag[index] = 1; // ���ó�����־λ
	}
	// �����ǰ�����ӳ���״̬�ͷ�
	else if (state == 1 && long_key_flag[index] == 1)
	{
		long_key_flag[index] = 0; // ���������־λ
	}
}

// /*****�����ж�ʹ�õ��Ƕ�ʱ��4*******/
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
					if (key_time[i] < 70)
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
				else if (double_click_timerEN[i] && ++double_click_time[i] >= 20)
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

		// �����ĳ��������߼�
		for (int i = 0; i < 4; i++)
		{
			// ����HandleLongPress��������ÿ�������ĳ���״̬
			HandleLongPress(i, key_state[i]);
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
