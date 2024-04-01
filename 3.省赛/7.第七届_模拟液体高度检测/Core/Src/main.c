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
__IO uint32_t uwTick_ADC_Speed_Ctrl;
__IO uint32_t uwTick_Time_Count;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*全局变量区*/
uint8_t ucLED;
uint8_t LED_1_count = 0, LED_2_count = 0, LED_3_count = 0;
uint8_t LED_2_flag, LED_3_flag;

uint8_t LCD_String_Disp[21];  // 显示屏显示数据存储数组
uint8_t LCD_Disp_Changle = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Changle = 0; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[5] = {30, 50, 70}; // EEPROM芯片中存取的数据

float K = 30.3;
uint8_t Height;
uint8_t Level = 0, Level_old = 0, Level_first = 0;
uint8_t Threshold[3] = {30, 50, 70};
float Get_Adc;

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

/*拓展变量区*/
extern uint8_t single_key_flag[4]; // 按键单击
extern uint8_t double_key_flag[4]; // 按键双击
extern uint8_t long_key_flag[4];   // 按键长按
extern uint16_t frq1, frq2;        // 输入捕获测量的频率
extern float duty1, duty2;         // 占空比

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
void ADC_Proc(void);
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

  if (EEPROM_Save[3] != 6) // 初次上电
  {
    EEPROM_Save[3] = 6; // 将该位的值改为9写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的阈值 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[3], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 3); // 将阈值写入EEPROM中
  }
  else // 再次上电读取eeprom中设置的阈值 更新显示的阈
  {
    I2C_24c02_Read(EEPROM_Save, 0x00, 3); // 读取EEPROM中的值
    HAL_Delay(10);
    Threshold[0] = EEPROM_Save[0];
    Threshold[1] = EEPROM_Save[1];
    Threshold[2] = EEPROM_Save[2];
  }
  /********************************EEPROM存储读取***********************************************************/

  Get_Adc = Get_ADC_Average(&hadc2, 10); // 开机先读取一次ADC

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
    ADC_Proc();

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
  if ((uwTick - uwTick_LED_Speed_Ctrl) < 200)
    return;
  uwTick_LED_Speed_Ctrl = uwTick; // 200ms执行一次

  ++LED_1_count;
  if (LED_1_count == 5)
  {
    ucLED ^= 0x01;
    LED_1_count = 0;
  }

  if (LED_2_flag == 1)
  {
    ++LED_2_count;
    if (LED_2_count <= 10)
      ucLED ^= 0x02;
    else
    {
      LED_2_flag = 0;
      LED_2_count = 0;
    }
  }
  else
    ucLED &= 0x05;

  if (LED_3_flag == 1)
  {
    ++LED_3_count;
    if (LED_3_count <= 10)
      ucLED ^= 0x04;
    else
    {
      LED_3_flag = 0;
      LED_3_count = 0;
    }
  }
  else
    ucLED &= 0x03;

  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 300)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  Height = K * Get_Adc; // 计算当前液体高度
  if (Get_Adc > 3.29f)
  {
    Height = 100;
  }

  if (LCD_Disp_Changle == 0)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Liquid Level");
    LCD_DisplayStringAtLineColumn(Line2, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Height:  %dcm     ", Height);
    LCD_DisplayStringAtLineColumn(Line3, Col5, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "ADC:     %.2fV   ", Get_Adc);
    LCD_DisplayStringAtLineColumn(Line4, Col5, LCD_String_Disp);
    if (Level_first == 1)
    {
      if (Height <= Threshold[0])
        Level = 0;
      else if (Height <= Threshold[1])
        Level = 1;
      else if (Height <= Threshold[2])
        Level = 2;
      else if (Height > Threshold[2])
        Level = 3;
    }
    else
    {
      if (Height <= Threshold[0])
        Level = 0;
      else if (Height <= Threshold[1])
        Level = 1;
      else if (Height <= Threshold[2])
        Level = 2;
      else if (Height > Threshold[2])
        Level = 3;
      Level_old = Level;
      Level_first = 1;
    }

    if (Level_old != Level && Level_first == 1)
    {
      LED_2_flag = 1;
      LED_2_count = 0;
    }

    if (Level_old > Level)
    {
      printf("A:H%d+L%d+D\r\n", Height, Level);
    }
    else if (Level_old < Level)
    {
      printf("A:H%d+L%d+U\r\n", Height, Level);
    }

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Level:   %d   ", Level);
    LCD_DisplayStringAtLineColumn(Line5, Col5, LCD_String_Disp);
    Level_old = Level;
  }
  else
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Parameter Setup");
    LCD_DisplayStringAtLineColumn(Line2, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Threshold1: %dcm   ", Threshold[0]);
    if (LCD_Line_Changle == 1)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line4, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Threshold2: %dcm   ", Threshold[1]);
    if (LCD_Line_Changle == 2)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line5, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Threshold3: %dcm   ", Threshold[2]);
    if (LCD_Line_Changle == 3)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line6, Col3, LCD_String_Disp);
  }
}
/*****************************************按键控制函数***************************************************/
void KEY_Proc(void)
{
  if ((uwTick - uwTick_KEY_Speed_Ctrl) < 50)
    return;
  uwTick_KEY_Speed_Ctrl = uwTick; // 50ms执行一次

  if (single_key_flag[0] == 1) // 按键1单击刷新显示屏
  {
    LCD_Disp_Changle ^= 1;     // 0-主界面 1-次界面
    if (LCD_Disp_Changle == 0) // 返回主界面时保存设置的值
    {
      EEPROM_Save[0] = Threshold[0];
      EEPROM_Save[1] = Threshold[1];
      EEPROM_Save[2] = Threshold[2];
      I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 3); // 将阈值写入EEPROM中
      HAL_Delay(10);
      LCD_Line_Changle = 0;
    }
    LCD_Clear(Black);
    single_key_flag[0] = 0;
  }

  if (single_key_flag[1] == 1) // 按键2单击
  {
    if (LCD_Disp_Changle)
    {
      LCD_Line_Changle++; // 选择高亮显示行
      if (LCD_Line_Changle == 4)
      {
        LCD_Line_Changle = 0;
      }
    }
    single_key_flag[1] = 0;
  }

  if (single_key_flag[2] == 1) // 按键3单击
  {
    if (LCD_Disp_Changle)
    {
      if (LCD_Line_Changle == 1) // 设定第一行
      {
        Threshold[0] += 5;
        if (Threshold[0] > 95)
          Threshold[0] = 95;
        if (Threshold[0] == Threshold[1])
          Threshold[0] -= 5;
      }
      else if (LCD_Line_Changle == 2) // 设定第二行
      {
        Threshold[1] += 5;
        if (Threshold[1] > 95)
          Threshold[1] = 95;
        if (Threshold[1] == Threshold[2])
          Threshold[1] -= 5;
      }
      else if (LCD_Line_Changle == 3) // 设定第三行
      {
        Threshold[2] += 5;
        if (Threshold[2] > 95)
          Threshold[2] = 95;
      }
    }
    single_key_flag[2] = 0;
  }

  if (single_key_flag[3] == 1) // 按键4单击
  {
    if (LCD_Line_Changle == 1) // 设定第一行
    {
      Threshold[0] -= 5;
      if (Threshold[0] == 0)
        Threshold[0] = 5;
    }
    else if (LCD_Line_Changle == 2) // 设定第二行
    {
      Threshold[1] -= 5;
      if (Threshold[1] == 0)
        Threshold[1] = 5;
      if (Threshold[1] == Threshold[0])
        Threshold[1] += 5;
    }
    else if (LCD_Line_Changle == 3) // 设定第三行
    {
      Threshold[2] -= 5;
      if (Threshold[2] == 0)
        Threshold[2] = 5;
      if (Threshold[2] == Threshold[1])
        Threshold[2] += 5;
    }
    single_key_flag[3] = 0;
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
        LED_3_flag = 1;
        printf("C:H%d+L%d\r\n", Height, Level);
      }

      else if (rxdata[0] == 'S')
      {
        LED_3_flag = 1;
        printf("S:TL%d+TM%d+TH%d\r\n", Threshold[0], Threshold[1], Threshold[2]);
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

void ADC_Proc(void)
{
  if ((uwTick - uwTick_ADC_Speed_Ctrl) < 1000)
    return;
  uwTick_ADC_Speed_Ctrl = uwTick; // 1s执行一次
  Get_Adc = Get_ADC_Average(&hadc2, 10);
}

/*RTC时钟配置*/
void RTC_Proc(void)
{
  if ((uwTick - uwTick_RTC_Speed_Ctrl) < 1000)
    return;
  uwTick_RTC_Speed_Ctrl = uwTick; // 1s执行一次

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
