#include "stm32g4xx.h" // Device header
#include "interrupt.h"
#include "usart.h"

// 定义全局变量用于存储捕获值和计算结果
float ccrl_val1a = 0, ccrl_val1b = 0;
float ccrl_val2a = 0, ccrl_val2b = 0;
uint16_t frq1 = 0, frq2 = 0; // 频率
float duty1 = 0, duty2 = 0;  // 占空比

// 重置定时器计数器并重新启动输入捕获通道
void ResetAndStartCapture(TIM_HandleTypeDef *htim, uint32_t Channel)
{
     __HAL_TIM_SetCounter(htim, 0);
     HAL_TIM_IC_Start(htim, Channel);
}

// 处理捕获事件的共用函数
void ProcessCapture(TIM_HandleTypeDef *htim, float *ccrl_vala, float *ccrl_valb, uint16_t *frq, float *duty)
{
     // 读取捕获值
     *ccrl_vala = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 直接
     *ccrl_valb = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // 间接

     // 重置计数器并启动捕获
     ResetAndStartCapture(htim, TIM_CHANNEL_1);
     ResetAndStartCapture(htim, TIM_CHANNEL_2);

     // 计算频率和占空比
     *frq = (uint16_t)((80000000 / 80) / *ccrl_vala);
     *duty = (*ccrl_valb / *ccrl_vala) * 100.0f;
}

// 输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
     // 根据定时器和通道处理捕获事件
     if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
     {
          ProcessCapture(htim, &ccrl_val1a, &ccrl_val1b, &frq1, &duty1);
     }
     else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
     {
          ProcessCapture(htim, &ccrl_val2a, &ccrl_val2b, &frq2, &duty2);
     }
}

// 串口接收中断函数
char rxdata[30];
uint8_t rxdat;
unsigned char rx_pointer;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hurat)
{
     rxdata[rx_pointer++] = rxdat;
     HAL_UART_Receive_IT(&huart1, &rxdat, 1);
}
