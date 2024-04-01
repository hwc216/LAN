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
#include "seg_LED.h"
#include "adc_key.h"
#include "ds18b20.h"
#include "dht11.h"
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

uint8_t LCD_String_Disp[21]; // 显示屏显示数据存储数组
uint8_t LCD_Disp_Change = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Change = 0; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t EEPROM_Save[4] = {1, 1}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {1, 1}; // EEPROM数据显示数组

ADC2_ChannelValues adcValues; // ADC多通道扫描数值结构体

float PA4_AD_Value, PA5_AD_Value;

uint8_t X_value = 1, Y_value = 1;

uint8_t Mode_flag = 0; // 0-倍频模式  1-分频模式

uint8_t Record_Channel_flag = 0; // 0-PA4 1-PA5

uint8_t LCD_Disp_Mode;

typedef struct
{
  uint8_t Record_num; // 记录次数
  float AD_value_max; // 电压最大值
  float AD_value_min; // 电压最小值
  float AD_value_avg; // 电压平均值
  float AD_value_sum; // 记录电压值总和
} Record_Value;

uint8_t first_flag = 0;

Record_Value PA4, PA5;

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

/*拓展变量区*/
extern uint8_t single_key_flag[4]; // 按键单击
extern uint8_t double_key_flag[4]; // 按键双击
extern uint8_t long_key_flag[4];   // 按键长按
// extern uint16_t frq1, frq2;        // 输入捕获测量的频率
// extern float duty1, duty2;         // 占空比
extern uint32_t RP[4];

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
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM15_Init();
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

  // /*定时器16通道1 PWM生成,PA6 引脚PWM生成定时器*/
  // HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  /*定时器17通道1 PWM生成,PA7 引脚PWM生成定时器*/
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  /*定时器16通道1 输入捕获中断开启,PA6引脚*/
  // HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);

  /*定时器17通道1 输入捕获中断开启,PA7引脚*/
  // HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

  /*定时器15通道1 输入捕获中断开启,PA2引脚*/
  // HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);

  // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, PA6_duty);
  // __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7_duty);

  /*定时器2通道1 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /*定时器2通道2 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /*定时器3通道1 输入捕获中断开启,PB4引脚*/
  // HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /*定时器3通道2 输入捕获中断开启,PB4引脚*/
  // HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1，PB12引脚
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2，PB15引脚

  /********************************串口接收中断开启***********************************************************/
  HAL_UART_Receive_IT(&huart1, &rxdat, 1);

  /********************************EEPROM存储读取***********************************************************/
  I2C_24c02_Read((uint8_t *)&EEPROM_Save[3], 0x37, 1); // 读取一次判断是否第一次上电

  if (EEPROM_Save[3] != 2) // 初次上电
  {
    EEPROM_Save[3] = 2; // 将该位的值改为2写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的阈值 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[3], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 2); // 将阈值写入EEPROM中
  }
  else // 再次上电读取eeprom中设置的阈值 更新显示的阈
  {
    I2C_24c02_Read(EEPROM_Save, 0x00, 2); // 读取EEPROM中的值
    HAL_Delay(10);
    Y_value = EEPROM_Save[0]; // 电压参数
    X_value = EEPROM_Save[1]; // 频率参数
  }
  /********************************EEPROM存储读取***********************************************************/

  PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, RP[2] * X_value);

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
  if ((uwTick - uwTick_LED_Speed_Ctrl) < 100)
    return;
  uwTick_LED_Speed_Ctrl = uwTick; // 200ms执行一次

  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  if (!Mode_flag)
    ucLED |= 0x01;
  else
    ucLED &= 0x0E;

  if (Mode_flag)
    ucLED |= 0x02;
  else
    ucLED &= 0x0D;

  if (PA4_AD_Value > PA5_AD_Value * Y_value)
    ucLED ^= 0x04;
  else
    ucLED &= 0x0B;

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 100)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  if (Mode_flag == 0) // 倍频
  {
    PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, RP[2] * X_value);
    printf("%d\r\n", RP[2]);
  }
  else
  {
    PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, RP[2] / X_value);
  }

  if (LCD_Disp_Change == 0) // 数据界面
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "DATA      ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PA4=%.2f  ", PA4_AD_Value);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PA5=%.2f  ", PA5_AD_Value);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PA1=%d    ", RP[2]);
    LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);
  }
  else if (LCD_Disp_Change == 1) // 参数界面
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PARA      ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "X=%d  ", X_value);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Y=%d  ", Y_value);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);
  }
  else // 记录界面
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "REC-PA%c  ", Record_Channel_flag ? '5' : '4');
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    if (Record_Channel_flag == 0) // PA4 记录
    {
      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "N=%d    ", PA4.Record_num);
      LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "A=%.2f  ", PA4.AD_value_max);
      LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "T=%.2f  ", PA4.AD_value_min);
      LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "H=%.2f  ", PA4.AD_value_avg);
      LCD_DisplayStringAtLineColumn(Line7, Col6, LCD_String_Disp);
    }
    else
    {
      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "N=%d    ", PA5.Record_num);
      LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "A=%.2f  ", PA5.AD_value_max);
      LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "T=%.2f  ", PA5.AD_value_min);
      LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "H=%.2f  ", PA5.AD_value_avg);
      LCD_DisplayStringAtLineColumn(Line7, Col6, LCD_String_Disp);
    }
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
    LCD_Disp_Change += 1; // 0-数据界面 1-参数界面 2-记录界面
    if (LCD_Disp_Change > 2)
    {
      LCD_Disp_Change = 0;
    }

    if (LCD_Disp_Change == 0) // 返回主界面时保存设置的值
    {
    }
    LCD_Clear(Black);
    single_key_flag[0] = 0;
  }

  if (single_key_flag[1] == 1) // 按键2单击
  {
    if (LCD_Disp_Change == 1) // 参数界面
    {
      // 调整X的值 1-4循环 数值存入EEPROM位置1
      X_value++;
      if (X_value > 4)
        X_value = 1;
      EEPROM_Save[1] = X_value;
      I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 2); // 将频率参数写入EEPROM中
      HAL_Delay(10);
    }
    single_key_flag[1] = 0;
  }

  if (single_key_flag[2] == 1) // 按键3单击
  {
    if (LCD_Disp_Change == 1) // 参数界面
    {
      // 调整Y的值 1-4循环 存入EEPROM位置0
      Y_value++;
      if (Y_value > 4)
        Y_value = 1;
      EEPROM_Save[0] = Y_value;
      I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 2); // 将电压参数写入EEPROM中
      HAL_Delay(10);
    }
    single_key_flag[2] = 0;
  }

  if (single_key_flag[3] == 1) // 按键4单击
  {
    if (LCD_Disp_Change == 0) // 在数据界面
    {
      // 启动一次电压测量功能，LCD 屏幕显示的电压数据更新一次。
      PA4_AD_Value = Return_3_3(adcValues.AdcChannel17Value);
      PA5_AD_Value = Return_3_3(adcValues.AdcChannel13Value);

      PA4.Record_num++; // 记录次数
      PA5.Record_num++;
      if (first_flag == 0)
      {
        first_flag = 1;
        PA4.AD_value_min = 999;
        PA5.AD_value_min = 999;
      }

      if (PA4_AD_Value > PA4.AD_value_max) // PA4采集最大值
        PA4.AD_value_max = PA4_AD_Value;

      if (PA5_AD_Value > PA5.AD_value_max) // PA5采集最大值
        PA5.AD_value_max = PA5_AD_Value;

      if (PA4_AD_Value < PA4.AD_value_min) // PA4采集最小值
        PA4.AD_value_min = PA4_AD_Value;

      if (PA5_AD_Value < PA5.AD_value_min) // PA5采集最小值
        PA5.AD_value_min = PA5_AD_Value;

      PA4.AD_value_sum += PA4_AD_Value; // PA4采集总和
      PA5.AD_value_sum += PA5_AD_Value; // PA5采集总和

      PA4.AD_value_avg = PA4.AD_value_sum / (float)PA4.Record_num; // PA4采集平均值
      PA5.AD_value_avg = PA5.AD_value_sum / (float)PA5.Record_num; // PA5采集平均值
    }

    if (LCD_Disp_Change == 1) // 参数界面
    {
      // 切换脉冲输出模式为倍频模式或分频模式
      Mode_flag ^= 0x01; // 0-倍频  1-分频
    }

    if (LCD_Disp_Change == 2) // 记录界面
    {
      // 切换不同测量通道的记录结果
      Record_Channel_flag ^= 0x01; // 0-PA4 1-PA5
    }
    single_key_flag[3] = 0;
  }

  if (long_key_flag[3] == 1) // 按键4长按
  {
    if (LCD_Disp_Change == 2) // 记录界面
    {
      // 长按 B4 按键超过 1 秒后松开，可以清零当前通道的全部数据记录结果，包括记录次数、最大值、最小值和平均值。

      if (Record_Channel_flag == 0) // PA4 记录数据清零
      {
        PA4.Record_num = 0;
        PA4.AD_value_max = 0;
        PA4.AD_value_min = 0;
        PA4.AD_value_avg = 0;
        PA4.AD_value_sum = 0;
      }
      else // PA5 记录数据 清零
      {
        PA5.Record_num = 0;
        PA5.AD_value_max = 0;
        PA5.AD_value_min = 0;
        PA5.AD_value_avg = 0;
        PA5.AD_value_sum = 0;
      }
    }
    long_key_flag[3] = 0;
  }
}

/*****************************************ADC处理函数***************************************************/

void ADC_Proc(void)
{

  adcValues = Read_ADC2_Channels();

  // ADC_Key = Scan_Btn(adcValues.AdcChannel13Value); // 中位值滤波获取ADC_Key的键值
}

/*****************************************串口接收函数***************************************************/

void USART_Proc(void)
{

  if (rx_pointer > 0)
  {
    if (rx_pointer == 1)
    {
      if (rxdata[0] == 'X')
      {
        printf("X:%d\r\n", X_value);
      }
      else if (rxdata[0] == 'Y')
      {
        printf("Y:%d\r\n", Y_value);
      }
      else if (rxdata[0] == '#')
      {
        LCD_Disp_Mode ^= 0x01;
        setDisplayMod(LCD_Disp_Mode);
        LCD_Clear(Black);
      }
    }

    if (rx_pointer == 3)
    {
      if (rxdata[0] == 'P' && rxdata[1] == 'A' && rxdata[2] == '1')
      {
        printf("PA1:%d\r\n", RP[2]);
      }
      if (rxdata[0] == 'P' && rxdata[1] == 'A' && rxdata[2] == '4')
      {
        printf("PA4:%.2f\r\n", Return_3_3(adcValues.AdcChannel17Value));
      }
      if (rxdata[0] == 'P' && rxdata[1] == 'A' && rxdata[2] == '5')
      {
        printf("PA5:%.2f\r\n", Return_3_3(adcValues.AdcChannel13Value));
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
