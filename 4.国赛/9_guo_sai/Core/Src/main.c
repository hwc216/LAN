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

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[10] = {0, 2, 0, 0, 2, 0, 0, 2, 0}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10};                     // EEPROM数据显示数组

uint8_t segBuff[3]; // Seg数码管显示数组

ADC2_ChannelValues adcValues; // ADC多通道扫描数值结构体
uint8_t ADC_Key;              // ADC_KEY按键值

float ADC_RP5_Value; // RP5 ADC采集值

uint8_t Goods_Choose = 1;

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

typedef struct
{
  float Price;     // 单价
  float Price_Old; // 上次单价
  int Price_Save;
  float All_Price; // 总价
} Goods;

Goods Good_1, Good_2, Good_3;

float Goods_Height; // 商品重量

uint16_t Change_Num;

/*拓展变量区*/
extern uint8_t single_key_flag[4];     // 按键单击
extern uint8_t double_key_flag[4];     // 按键双击
extern uint8_t long_key_flag[4];       // 按键长按
extern uint8_t single_ADC_key_flag[8]; // ADC按键单击
extern uint8_t double_ADC_key_flag[8]; // ADC按键双击
extern uint8_t long_ADC_key_flag[8];   // ADC按键长按
extern uint16_t key_ADC_time[];

// extern uint16_t frq1, frq2;        // 输入捕获测量的频率
// extern float duty1, duty2;         // 占空比

extern uint32_t RP[4]; // 轮询ADC采集值或者占空比值

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
  MX_TIM6_Init();
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

  /*定时器6基本中断开启,按键中断定时器 时间到就进中断检测按键是否按下*/
  HAL_TIM_Base_Start_IT(&htim6);

  // /*定时器16通道1 PWM生成,PA6 引脚PWM生成定时器*/
  // HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  // /*定时器17通道1 PWM生成,PA7 引脚PWM生成定时器*/
  // HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  /*定时器16通道1 输入捕获中断开启,PA6引脚*/
  HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);

  /*定时器17通道1 输入捕获中断开启,PA7引脚*/
  HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

  /*定时器15通道1 输入捕获中断开启,PA2引脚*/
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);

  // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, PA6_duty);
  // __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7_duty);

  /*定时器2通道1 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /*定时器2通道2 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /*定时器3通道1 输入捕获中断开启,PB4引脚*/
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /*定时器3通道2 输入捕获中断开启,PB4引脚*/
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1，PB12引脚
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2，PB15引脚

  /********************************串口接收中断开启***********************************************************/
  HAL_UART_Receive_IT(&huart1, &rxdat, 1);

  /********************************EEPROM存储读取***********************************************************/
  I2C_24c02_Read((uint8_t *)&EEPROM_Save[9], 0x37, 1); // 读取一次判断是否第一次上电

  if (EEPROM_Save[9] != 5) // 初次上电
  {
    EEPROM_Save[9] = 5; // 将该位的值改为9写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的阈值 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[9], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 9); // 将单价写入EEPROM中
  }
  else // 再次上电读取eeprom中设置的阈值 更新显示的阈
  {
    I2C_24c02_Read(EEPROM_Save, 0x00, 9); // 读取EEPROM中的值
    HAL_Delay(10);
    Good_1.Price = (EEPROM_Save[0] + EEPROM_Save[1] * 10 + EEPROM_Save[2] * 100) / 100.0f;
    Good_2.Price = (EEPROM_Save[3] + EEPROM_Save[4] * 10 + EEPROM_Save[5] * 100) / 100.0f;
    Good_3.Price = (EEPROM_Save[6] + EEPROM_Save[7] * 10 + EEPROM_Save[8] * 100) / 100.0f;

    Good_1.Price_Old = Good_1.Price; // 更新旧值
    Good_2.Price_Old = Good_2.Price;
    Good_3.Price_Old = Good_3.Price;
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
  if ((uwTick - uwTick_LED_Speed_Ctrl) < 400)
    return;
  uwTick_LED_Speed_Ctrl = uwTick; // 400ms执行一次

  static uint8_t Time_delay;

  if (LCD_Disp_Change == 0)
  {
    Time_delay++;
    if (Time_delay > 2)
    {
      ucLED ^= 0x01;
      Time_delay = 0;
    }
    else
      ucLED &= 0x00;
  }
  else
  {
    ucLED ^= 0x01;
  }

  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 100)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  ADC_RP5_Value = Return_3_3(adcValues.AdcChannel17Value);
  Goods_Height = ADC_RP5_Value * 3.03f;

  if (LCD_Disp_Change == 0)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    if (Goods_Choose == 1)
    {
      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Tips  ");
      LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ID:%d  ", 1);
      LCD_DisplayStringAtLineColumn(Line4, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_DJ:%.2f$/kg  ", Good_1.Price);
      LCD_DisplayStringAtLineColumn(Line5, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZL:%.2fkg  ", Goods_Height);
      LCD_DisplayStringAtLineColumn(Line6, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZJ:%.2f$  ", Good_1.All_Price);
      LCD_DisplayStringAtLineColumn(Line7, Col2, LCD_String_Disp);
    }
    else if (Goods_Choose == 2)
    {
      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Tips  ");
      LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ID:%d  ", 2);
      LCD_DisplayStringAtLineColumn(Line4, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_DJ:%.2f$/kg  ", Good_2.Price);
      LCD_DisplayStringAtLineColumn(Line5, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZL:%.2fkg  ", Goods_Height);
      LCD_DisplayStringAtLineColumn(Line6, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZJ:%.2f$  ", Good_2.All_Price);
      LCD_DisplayStringAtLineColumn(Line7, Col2, LCD_String_Disp);
    }
    else if (Goods_Choose == 3)
    {
      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Tips  ");
      LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ID:%d  ", 3);
      LCD_DisplayStringAtLineColumn(Line4, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_DJ:%.2f$/kg  ", Good_3.Price);
      LCD_DisplayStringAtLineColumn(Line5, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZL:%.2fkg  ", Goods_Height);
      LCD_DisplayStringAtLineColumn(Line6, Col2, LCD_String_Disp);

      memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
      sprintf((char *)LCD_String_Disp, "Goods_ZJ:%.2f$  ", Good_3.All_Price);
      LCD_DisplayStringAtLineColumn(Line7, Col2, LCD_String_Disp);
    }
  }
  else
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Para  ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Goods1:%.2f$/kg  ", Good_1.Price);
    if (LCD_Line_Change == 1)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line4, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Goods2:%.2f$/kg  ", Good_2.Price);
    if (LCD_Line_Change == 2)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line5, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Goods3:%.2f$/kg  ", Good_3.Price);
    if (LCD_Line_Change == 3)
    {
      LCD_SetTextColor(Green);
    }
    else
    {
      LCD_SetTextColor(White);
    }
    LCD_DisplayStringAtLineColumn(Line6, Col4, LCD_String_Disp);

    LCD_SetTextColor(White);
    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Change_Num:%d  ", Change_Num);
    LCD_DisplayStringAtLineColumn(Line9, Col8, LCD_String_Disp);
  }
}

/*****************************************按键控制函数***************************************************/

void KEY_Proc(void)
{
  if ((uwTick - uwTick_KEY_Speed_Ctrl) < 50)
    return;
  uwTick_KEY_Speed_Ctrl = uwTick; // 500ms执行一次

  if (single_ADC_key_flag[0] == 1) // 切换主界面与参数设置界面
  {
    LCD_Disp_Change ^= 0x01;

    // 回到主界面时通过EEPROM保存调整结果  上电读取
    if (LCD_Disp_Change == 0)
    {

      if (Good_1.Price_Old != Good_1.Price || Good_2.Price_Old != Good_2.Price || Good_3.Price_Old != Good_3.Price)
        Change_Num++;

      Good_1.Price_Save = (int)(Good_1.Price * 100);
      Good_2.Price_Save = (int)(Good_2.Price * 100);
      Good_3.Price_Save = (int)(Good_3.Price * 100);

      EEPROM_Save[2] = Good_1.Price_Save / 100;
      EEPROM_Save[1] = Good_1.Price_Save / 10 % 10;
      EEPROM_Save[0] = Good_1.Price_Save % 10;

      EEPROM_Save[5] = Good_2.Price_Save / 100;
      EEPROM_Save[4] = Good_2.Price_Save / 10 % 10;
      EEPROM_Save[3] = Good_2.Price_Save % 10;

      EEPROM_Save[8] = Good_3.Price_Save / 100;
      EEPROM_Save[7] = Good_3.Price_Save / 10 % 10;
      EEPROM_Save[6] = Good_3.Price_Save % 10;

      I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 9); // 将单价写入EEPROM中

      Good_1.Price_Old = Good_1.Price; // 更新旧值
      Good_2.Price_Old = Good_2.Price;
      Good_3.Price_Old = Good_3.Price;

      printf("U.W.1:%.2f\r\n", Good_1.Price);
      printf("U.W.2:%.2f\r\n", Good_2.Price);
      printf("U.W.3:%.2f\r\n", Good_3.Price);

      LCD_Line_Change = 0;
    }

    LCD_Clear(Black);
    single_ADC_key_flag[0] = 0;
  }

  if (single_ADC_key_flag[1] == 1) // 按键2
  {
    if (LCD_Disp_Change == 1) // 在参数界面有效
    {
      if (LCD_Line_Change == 1)
      {
        // 修改第一行数据  加0.01
        Good_1.Price += 0.01f;
        if (Good_1.Price > 10.0f)
          Good_1.Price = 10;
      }

      if (LCD_Line_Change == 2)
      {
        // 修改第二行数据  加0.01
        Good_2.Price += 0.01f;
        if (Good_2.Price > 10.0f)
          Good_2.Price = 10;
      }

      if (LCD_Line_Change == 3)
      {
        // 修改第三行数据  加0.01
        Good_3.Price += 0.01f;
        if (Good_3.Price > 10.0f)
          Good_3.Price = 10;
      }
    }
    single_ADC_key_flag[1] = 0;
  }

  if (single_ADC_key_flag[2] == 1) // 按键3
  {
    if (LCD_Disp_Change == 1) // 在参数界面有效
    {
      if (LCD_Line_Change == 1)
      {
        // 修改第一行数据  减0.01
        Good_1.Price -= 0.01f;
        if (Good_1.Price <= 0.0f)
          Good_1.Price = 0.0f;
      }

      if (LCD_Line_Change == 2)
      {
        // 修改第二行数据  减0.01
        Good_2.Price -= 0.01f;
        if (Good_2.Price <= 0.0f)
          Good_2.Price = 0.0f;
      }

      if (LCD_Line_Change == 3)
      {
        // 修改第三行数据  减0.01
        Good_3.Price -= 0.01f;
        if (Good_3.Price <= 0.0f)
          Good_3.Price = 0.0f;
      }
    }
    single_ADC_key_flag[2] = 0;
  }

  if (single_ADC_key_flag[3] == 1) // 按键4
  {

    if (LCD_Disp_Change)
    {
      LCD_Line_Change++; // 选择高亮显示行
      if (LCD_Line_Change == 4)
      {
        LCD_Line_Change = 0;
      }
    }
    single_ADC_key_flag[3] = 0;
  }

  if (single_ADC_key_flag[4] == 1) // 选择主界面显示商品1 //按键5
  {
    Goods_Choose = 1;
    single_ADC_key_flag[4] = 0;
  }

  if (single_ADC_key_flag[5] == 1) // 选择主界面显示商品2  //按键6
  {
    Goods_Choose = 2;
    single_ADC_key_flag[5] = 0;
  }

  if (single_ADC_key_flag[6] == 1) // 选择主界面显示商品3  //按键7
  {
    Goods_Choose = 3;
    single_ADC_key_flag[6] = 0;
  }

  if (single_ADC_key_flag[7] == 1) // 按键8
  {
    // 更新当前货物总价  通过串口发送信息
    if (Goods_Choose == 1)
    {
      Good_1.All_Price = Good_1.Price * Goods_Height;
      printf("U.W.1:%.2f\r\n", Good_1.Price);
      printf("G.W:%.2f\r\n", Goods_Height);
      printf("Total:%.2f\r\n", Good_1.All_Price);
    }

    if (Goods_Choose == 2)
    {
      Good_2.All_Price = Good_2.Price * Goods_Height;
      printf("U.W.2:%.2f\r\n", Good_2.Price);
      printf("G.W:%.2f\r\n", Goods_Height);
      printf("Total:%.2f\r\n", Good_2.All_Price);
    }

    if (Goods_Choose == 3)
    {
      Good_3.All_Price = Good_3.Price * Goods_Height;
      printf("U.W.3:%.2f\r\n", Good_3.Price);
      printf("G.W:%.2f\r\n", Goods_Height);
      printf("Total:%.2f\r\n", Good_3.All_Price);
    }

    single_ADC_key_flag[7] = 0;
  }

  if (ADC_Key == 2 && key_ADC_time[1] > 40) // ADC按键2长按递增
  {
    if (LCD_Disp_Change == 1) // 在参数界面有效
    {
      if (LCD_Line_Change == 1)
      {
        // 修改第一行数据  加0.01
        Good_1.Price += 0.01f;
        if (Good_1.Price > 10.0f)
          Good_1.Price = 10;
      }

      if (LCD_Line_Change == 2)
      {
        // 修改第二行数据  加0.01
        Good_2.Price += 0.01f;
        if (Good_2.Price > 10.0f)
          Good_2.Price = 10;
      }

      if (LCD_Line_Change == 3)
      {
        // 修改第三行数据  加0.01
        Good_3.Price += 0.01f;
        if (Good_3.Price > 10.0f)
          Good_3.Price = 10;
      }
    }
    long_ADC_key_flag[1] = 0;
  }

  if (ADC_Key == 3 && key_ADC_time[2] > 40) // ADC按键3长按递增
  {
    if (LCD_Disp_Change == 1) // 在参数界面有效
    {
      if (LCD_Line_Change == 1)
      {
        // 修改第一行数据  减0.01
        Good_1.Price -= 0.01f;
        if (Good_1.Price <= 0.0f)
          Good_1.Price = 0.0f;
      }

      if (LCD_Line_Change == 2)
      {
        // 修改第二行数据  减0.01
        Good_2.Price -= 0.01f;
        if (Good_2.Price <= 0.0f)
          Good_2.Price = 0.0f;
      }

      if (LCD_Line_Change == 3)
      {
        // 修改第三行数据  减0.01
        Good_3.Price -= 0.01f;
        if (Good_3.Price <= 0.0f)
          Good_3.Price = 0.0f;
      }
    }
    long_ADC_key_flag[2] = 0;
  }
}

/*****************************************ADC处理函数***************************************************/

void ADC_Proc(void)
{

  adcValues = Read_ADC2_Channels();
  ADC_Key = Scan_Btn(adcValues.AdcChannel13Value); // 中位值滤波获取ADC_Key的键值
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
        // printf("F:%.2f\r\n", duty1);
      }

      else if (rxdata[0] == 'S')
      {
        printf("ADC1:%d\r\n", 1);
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
