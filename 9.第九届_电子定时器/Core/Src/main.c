/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"
#include "Key.h"
#include "interrupt.h"
#include "badc.h"
#include "i2c - hal.h"
#include "PWM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*执行速度控制变量*/
__IO uint32_t uwTick_LED_Speed_Ctrl;
__IO uint32_t uwTick_KEY_Speed_Ctrl;
__IO uint32_t uwTick_LCD_Speed_Ctrl;
__IO uint32_t uwTick_RTC_Speed_Ctrl;
__IO uint32_t uwTick_Time_Count;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*全局变量区*/
uint8_t ucLED;

uint8_t LCD_String_Disp[21];  // 显示屏显示数据存储数组
uint8_t LCD_Disp_Changle = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Changle = 0; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[4] = {10, 10}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10}; // EEPROM数据显示数组

uint8_t Save_Num = 1;                                                                 // 时间存储区序号
uint8_t Run_Statue_flag = 1;                                                          // 运行状态标志位
uint8_t Time_Mermony_Space[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; // 时间存储缓冲数组
int8_t Time_Mermony_Space_Disp[3];                                                    // 时间显示数组
uint8_t index;                                                                        // 下标，用来取Time_Mermony_Space中的时间

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

/*拓展变量区*/
extern uint8_t single_key_flag[4]; // 按键单击
extern uint8_t double_key_flag[4]; // 按键双击
extern uint8_t long_key_flag[4];   // 按键长按
extern uint16_t key_time[4];
extern uint16_t frq1, frq2; // 输入捕获测量的频率
extern float duty1, duty2;  // 占空比

extern char rxdata[30]; // 串口接收数组
extern uint8_t rxdat;
extern unsigned char rx_pointer;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*子函数声明区*/

void LCD_Proc(void);
void KEY_Proc(void);
void LED_Proc(void);
void RTC_Proc(void);
void USART_Proc(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /*LED灯初始化*/
  LED_Init();

  /*LCD显示屏初始化*/
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);

  /*定时器4基本中断开启,按键中断定时器 时间到就进中断检测按键是否按下*/
  HAL_TIM_Base_Start_IT(&htim4);

  /*定时器16通道1 PWM生成,PA6 引脚PWM生成定时器*/
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  /*定时器17通道1 PWM生成,PA7 引脚PWM生成定时器*/
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, PA6_duty);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7_duty);

  /*定时器2通道1 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /*定时器2通道2 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /*定时器3通道1 输入捕获中断开启,PB4引脚*/
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /*定时器3通道2 输入捕获中断开启,PB4引脚*/
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1，PB12引脚
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2，PB15引脚

  /********************************串口接收中断开启***********************************************************/
  HAL_UART_Receive_IT(&huart1, &rxdat, 1);

  /********************************EEPROM存储读取***********************************************************/
  I2C_24c02_Read((uint8_t *)&EEPROM_Save[3], 0x37, 1); // 读取一次判断是否第一次上电

  if (EEPROM_Save[3] != 1) // 初次上电
  {
    EEPROM_Save[3] = 1; // 将该位的值改为1写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的时间 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[3], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)Time_Mermony_Space, 0x00, 15); // 将默认的时间写入EEPROM中
  }
  else // 再次上电读取eeprom中设置的时间 更新显示的时间
  {
    I2C_24c02_Read(Time_Mermony_Space, 0x00, 15); // 读取EEPROM中的值
    HAL_Delay(10);
  }

  for (int i = 0; i < 3; i++) // 上电初始化时间显示设置
  {
    Time_Mermony_Space_Disp[i] = Time_Mermony_Space[i];
  }

  /********************************EEPROM存储读取***********************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*代码写在该行下面 否则会被初始化*/

    LCD_Proc();
    KEY_Proc();
    LED_Proc();
    RTC_Proc();

    if (rx_pointer != 0)
    {
      int temp = rx_pointer;
      HAL_Delay(1);
      if (temp == rx_pointer)
        USART_Proc(); // 完成接收
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*****************************************LED灯控制函数***************************************************/
void LED_Proc(void)
{
  if ((uwTick - uwTick_LED_Speed_Ctrl) < 500)
    return;
  uwTick_LED_Speed_Ctrl = uwTick; // 500ms执行一次

  if (Run_Statue_flag == 3)
  {
    ucLED ^= 0x01;
    PWM_Set_Duty(&htim16, TIM_CHANNEL_1, 80);
    PWM_Set_Frequency(&htim16, TIM_CHANNEL_1, 1000);
  }
  else
  {
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    ucLED &= 0x00;
  }
  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 300)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
  sprintf((char *)LCD_String_Disp, "No %d", Save_Num);
  LCD_DisplayStringAtLineColumn(Line2, Col3, LCD_String_Disp);

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp)); // 时
  sprintf((char *)LCD_String_Disp, "%02d", Time_Mermony_Space_Disp[0]);
  if (LCD_Line_Changle == 1)
    LCD_SetTextColor(Green);
  else
    LCD_SetTextColor(White);

  LCD_DisplayStringAtLineColumn(Line5, Col7, LCD_String_Disp);

  LCD_SetTextColor(White);
  LCD_DisplayChar(Line5, Col9, ':');

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp)); // 分
  sprintf((char *)LCD_String_Disp, " %02d", Time_Mermony_Space_Disp[1]);
  if (LCD_Line_Changle == 2)
    LCD_SetTextColor(Green);
  else
    LCD_SetTextColor(White);
  LCD_DisplayStringAtLineColumn(Line5, Col10, LCD_String_Disp);

  LCD_SetTextColor(White);
  LCD_DisplayChar(Line5, Col13, ':');

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp)); // 秒
  sprintf((char *)LCD_String_Disp, " %02d", Time_Mermony_Space_Disp[2]);
  if (LCD_Line_Changle == 3)
    LCD_SetTextColor(Green);
  else
    LCD_SetTextColor(White);
  LCD_DisplayStringAtLineColumn(Line5, Col14, LCD_String_Disp);

  LCD_SetTextColor(White);

  switch (Run_Statue_flag)
  {
  case 1:
    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Standby ");
    LCD_DisplayStringAtLineColumn(Line8, Col8, LCD_String_Disp);
    break;

  case 2:
    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Setting ");
    LCD_DisplayStringAtLineColumn(Line8, Col8, LCD_String_Disp);
    break;

  case 3:
    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Running ");
    LCD_DisplayStringAtLineColumn(Line8, Col8, LCD_String_Disp);
    break;

  case 4:
    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Pause   ");
    LCD_DisplayStringAtLineColumn(Line8, Col8, LCD_String_Disp);
    break;
  }

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
  sprintf((char *)LCD_String_Disp, "%dHz  %.2f%%     ", frq1, duty1);
  LCD_DisplayStringAtLineColumn(Line9, Col3, LCD_String_Disp);
}
/*****************************************按键控制函数***************************************************/
void KEY_Proc(void)
{
  if ((uwTick - uwTick_KEY_Speed_Ctrl) < 50)
    return;
  uwTick_KEY_Speed_Ctrl = uwTick; // 50ms执行一次

  if (single_key_flag[0] == 1) // 按键1单击切换存储区域
  {
    Save_Num++;
    LCD_Line_Changle = 0;
    if (Save_Num == 6)
    {
      Save_Num = 1;
    }

    index = (Save_Num - 1) * 3; // 计算Time_Memory_Space的起始索引
    for (int i = 0; i < 3; i++)
    {
      Time_Mermony_Space_Disp[i] = Time_Mermony_Space[index + i];
    }

    LCD_Clear(Black);
    single_key_flag[0] = 0;
  }

  if (single_key_flag[1] == 1) // 按键2单击
  {

    LCD_Line_Changle++;  // 选择高亮显示行
    Run_Statue_flag = 2; // Setting状态
    if (LCD_Line_Changle == 4)
    {
      LCD_Line_Changle = 1;
    }
    single_key_flag[1] = 0;
  }

  if (single_key_flag[2] == 1) // 按键3单击
  {

    if (LCD_Line_Changle == 1) // 设定第一行
    {
      Time_Mermony_Space[index] += 1;
      if (Time_Mermony_Space[index] > 23)
        Time_Mermony_Space[index] = 0;
    }
    else if (LCD_Line_Changle == 2) // 设定第二行
    {
      Time_Mermony_Space[index + 1] += 1;
      if (Time_Mermony_Space[index + 1] > 59)
        Time_Mermony_Space[index + 1] = 0;
    }
    else if (LCD_Line_Changle == 3) // 设定第三行
    {
      Time_Mermony_Space[index + 2] += 1;
      if (Time_Mermony_Space[index + 2] > 59)
        Time_Mermony_Space[index + 2] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
      Time_Mermony_Space_Disp[i] = Time_Mermony_Space[index + i];
    }

    single_key_flag[2] = 0;
  }

  if (single_key_flag[3] == 1) // 按键4单击
  {
    LCD_Line_Changle = 0;
    if (Run_Statue_flag < 3)
      Run_Statue_flag = 3; // Running状态
    else if (Run_Statue_flag == 3)
    {
      Run_Statue_flag = 4; // Pause状态
    }
    else if (Run_Statue_flag == 4)
    {
      Run_Statue_flag = 3; // Running状态
    }
    single_key_flag[3] = 0;
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0 && key_time[1] > 40) // 按键2长按
  {
    Run_Statue_flag = 1; // Standy状态
    LCD_Line_Changle = 0;
    long_key_flag[1] = 0;

    I2C_24c02_Write((uint8_t *)Time_Mermony_Space, 0x00, 15); // 将设定好的时间写入EEPROM中
    HAL_Delay(10);
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0 && key_time[2] > 40) // 按键3长按
  {

    if (LCD_Line_Changle == 1) // 设定第一行
    {
      Time_Mermony_Space[index] += 1;
      if (Time_Mermony_Space[index] > 23)
        Time_Mermony_Space[index] = 0;
    }
    else if (LCD_Line_Changle == 2) // 设定第二行
    {
      Time_Mermony_Space[index + 1] += 1;
      if (Time_Mermony_Space[index + 1] > 59)
        Time_Mermony_Space[index + 1] = 0;
    }
    else if (LCD_Line_Changle == 3) // 设定第三行
    {
      Time_Mermony_Space[index + 2] += 1;
      if (Time_Mermony_Space[index + 2] > 59)
        Time_Mermony_Space[index + 2] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
      Time_Mermony_Space_Disp[i] = Time_Mermony_Space[index + i];
    }

    HAL_Delay(300);
    long_key_flag[2] = 0;
  }

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 && key_time[3] > 40) // 按键4长按
  {
    if (Run_Statue_flag > 2)
      Run_Statue_flag = 1; // Standy状态
    long_key_flag[2] = 0;

    for (int i = 0; i < 3; i++) // 按键4长按,时间恢复
    {
      Time_Mermony_Space_Disp[i] = Time_Mermony_Space[index + i];
    }
  }
}

/*****************************************串口接收函数***************************************************/
void USART_Proc(void)
{

  if (rx_pointer > 0)
  {
    if (rx_pointer == 1)
    {
      if (rxdata[0] == 'C')
      {
        printf("F:%.2f\r\n", duty1);
      }

      else if (rxdata[0] == 'S')
      {
        printf("ADC1:%.2f\r\n", Get_ADC(&hadc1));
      }
      else
      {
        printf("Error\r\n");
      }
    }

    rx_pointer = 0;
    memset(rxdata, 0, 30);
  }
}

/*RTC时钟配置*/
void RTC_Proc(void)
{
  if ((uwTick - uwTick_RTC_Speed_Ctrl) < 1000)
    return;
  uwTick_RTC_Speed_Ctrl = uwTick; // 1s执行一次
  if (Run_Statue_flag == 3)
  {
    Time_Mermony_Space_Disp[2]--;
    if (Time_Mermony_Space_Disp[2] < 0)
    {
      Time_Mermony_Space_Disp[2] = 59;
      Time_Mermony_Space_Disp[1]--;
      if (Time_Mermony_Space_Disp[1] < 0)
      {
        Time_Mermony_Space_Disp[1] = 59;
        Time_Mermony_Space_Disp[0]--;
        if (Time_Mermony_Space_Disp[0] == -1)
        {
          for (int i = 0; i < 3; i++) // 计时结束，时间恢复
          {
            Time_Mermony_Space_Disp[i] = Time_Mermony_Space[index + i];
          }
          Run_Statue_flag = 1;
        }
      }
    }
  }

  // 获取当前时间和日期
  HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);

  // printf("Date: %02d-%02d-%02d Time: %02d:%02d:%02d\r\n", RTC_Date.Year, RTC_Date.Month, RTC_Date.Date,
  //        RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
}

/*串口重定向*/
int fputc(int ch, FILE *f)
{
  char c = ch;
  HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1, 50);
  return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
