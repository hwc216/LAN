#include "key.h"
#include "lcd.h"

#define LONG_PRESS_TIME 250		 // ���峤����ʱ����ֵ
#define DOUBLE_PRESS_INTERVAL 20 // ����˫�����ʱ����ֵ

extern uint8_t ADC_Key; // ADC_KEY����ֵ

uint8_t judge_state[4] = {0}, double_click_time[4] = {0}, key_state[4] = {0}, double_click_timerEN[4] = {0},
		single_key_flag[4] = {100}, double_key_flag[4] = {0}, long_key_flag[4] = {0};
uint16_t key_time[4] = {0};

uint8_t judge_ADC_state[8] = {0}, double_ADC_click_time[8] = {0}, key_ADC_state[8] = {0}, double_ADC_click_timerEN[8] = {0},
		single_ADC_key_flag[8] = {100}, double_ADC_key_flag[8] = {0}, long_ADC_key_flag[8] = {0};
uint16_t key_ADC_time[8] = {0};

void setReadKeys(uint8_t read_keys[], int i) // ADC����ʹ�ú���
{
	// �Ƚ�����λ����Ϊ1
	for (int j = 0; j < 8; ++j)
	{
		read_keys[j] = 1;
	}

	// Ȼ�����i��ֵ����Ӧ��λ����Ϊ0
	// i��ֵ��1��8������Ҫ��1���õ���ȷ������
	if (i >= 1 && i <= 8)
	{
		read_keys[i - 1] = 0;
	}
}

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

	if (htim->Instance == TIM6) // ��ʱ��6���ж��¼�
	{
		uint8_t read_keys[8];
		setReadKeys(read_keys, ADC_Key);

		for (int i = 0; i < 8; i++)
		{
			key_ADC_state[i] = read_keys[i];

			if (key_ADC_state[i] == 0)
			{ // ��������
				if (judge_ADC_state[i] == 0)
				{
					// �ռ�⵽��������
					judge_ADC_state[i] = 1;
					key_ADC_time[i] = 0;
				}
				else if (judge_ADC_state[i] == 1)
				{
					// ȷ�ϰ���ȷʵ�����£��������
					judge_ADC_state[i] = 2;
				}
			}
			else
			{ // ����δ���»��ͷ�
				if (judge_ADC_state[i] == 2)
				{
					if (key_ADC_time[i] < LONG_PRESS_TIME)
					{
						// �������ͷţ���δ�ﵽ����ʱ��
						double_ADC_click_timerEN[i] ^= 1;
						if (double_ADC_click_timerEN[i])
						{
							// ��һ�μ�⵽������˫��
							double_ADC_click_time[i] = 0;
						}
						else
						{
							// ��⵽�ڶ��ΰ��£�ȷ��Ϊ˫��
							double_ADC_key_flag[i] = 1;
						}
					}
					else
					{
						// �������ͷţ��Ҵﵽ����ʱ�䣬ȷ��Ϊ����
						long_ADC_key_flag[i] = 1;
					}
					judge_ADC_state[i] = 0;
				}
				else if (double_ADC_click_timerEN[i] && ++double_ADC_click_time[i] >= DOUBLE_PRESS_INTERVAL)
				{
					// ȷ��Ϊ����
					single_ADC_key_flag[i] = 1;
					double_ADC_click_timerEN[i] = 0;
				}
			}

			// �����ǰ״̬�ǰ���״̬�����Ӱ���ʱ��
			if (judge_ADC_state[i] == 2)
			{
				key_ADC_time[i]++;
			}
		}
	}
}

// //�����������ֵ���д��
//   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 && key_time[3] > 40) // ����4����
//   {
// 	long_key_flag[3] = 0;
//   }

// ADC������������

// if (single_ADC_key_flag[0] == 1) // �л���������������ý���
// {
// 	LCD_Disp_Change ^= 0x01;

// 	// �ص�������ʱ����
// 	if (LCD_Disp_Change == 0)
// 	{
// 		LCD_Line_Change = 0;
// 	}

// 	LCD_Clear(Black);
// 	single_ADC_key_flag[0] = 0;
// }

// if (single_ADC_key_flag[1] == 1) // ����2
// {
// 	if (LCD_Disp_Change == 1) // �ڲ���������Ч
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// �޸ĵ�һ������  ��0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// �޸ĵڶ�������  ��0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// �޸ĵ���������  ��0.01
// 		}
// 	}
// 	single_ADC_key_flag[1] = 0;
// }

// if (single_ADC_key_flag[2] == 1) // ����3
// {
// 	if (LCD_Disp_Change == 1) // �ڲ���������Ч
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// �޸ĵ�һ������  ��0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// �޸ĵڶ�������  ��0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// �޸ĵ���������  ��0.01
// 		}
// 	}
// 	single_ADC_key_flag[2] = 0;
// }

// if (single_ADC_key_flag[3] == 1) // ����4
// {

// 	if (LCD_Disp_Change)
// 	{
// 		LCD_Line_Change++; // ѡ�������ʾ��
// 		if (LCD_Line_Change == 4)
// 		{
// 			LCD_Line_Change = 0;
// 		}
// 	}
// 	single_ADC_key_flag[3] = 0;
// }

// if (single_ADC_key_flag[4] == 1) // ѡ����������ʾ��Ʒ1 //����5
// {
// 	single_ADC_key_flag[4] = 0;
// }

// if (single_ADC_key_flag[5] == 1) // ѡ����������ʾ��Ʒ2  //����6
// {
// 	single_ADC_key_flag[5] = 0;
// }

// if (single_ADC_key_flag[6] == 1) // ѡ����������ʾ��Ʒ3  //����7
// {
// 	single_ADC_key_flag[6] = 0;
// }

// if (single_ADC_key_flag[7] == 1) // ����8
// {

// 	single_ADC_key_flag[7] = 0;
// }

// if (ADC_Key == 2 && key_ADC_time[1] > 40) // ADC����2��������
// {
// 	if (LCD_Disp_Change == 1) // �ڲ���������Ч
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// �޸ĵ�һ������  ��0.01
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// �޸ĵڶ�������  ��0.01
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// �޸ĵ���������  ��0.01
// 		}
// 	}
// 	long_ADC_key_flag[1] = 0;
// }

// if (ADC_Key == 3 && key_ADC_time[2] > 40) // ADC����3��������
// {
// 	if (LCD_Disp_Change == 1) // �ڲ���������Ч
// 	{
// 		if (LCD_Line_Change == 1)
// 		{
// 			// �޸ĵ�һ������
// 		}

// 		if (LCD_Line_Change == 2)
// 		{
// 			// �޸ĵڶ�������
// 		}

// 		if (LCD_Line_Change == 3)
// 		{
// 			// �޸ĵ���������
// 		}
// 	}
// 	long_ADC_key_flag[2] = 0;
// }
