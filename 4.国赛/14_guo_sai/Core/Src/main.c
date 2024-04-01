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
__IO uint32_t uwTick_Record_Speed_Ctrl;
__IO uint32_t uwTick_IM_MC_Speed_Ctrl;
__IO uint32_t uwTick_IM_DY_Speed_Ctrl;
__IO uint32_t uwTick_Time_Count;
__IO uint32_t uwTick_Time_Count_IM_MC;
__IO uint32_t uwTick_Time_Count_IM_DY;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*全局变量区*/
uint8_t ucLED;

uint8_t LCD_String_Disp[21]; // 显示屏显示数据存储数组
uint8_t LCD_Disp_Change = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Change = 1; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[4] = {10, 10}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10}; // EEPROM数据显示数组

uint8_t segBuff[3]; // Seg数码管显示数组

ADC2_ChannelValues adcValues; // ADC多通道扫描数值结构体
uint8_t ADC_Key;              // ADC_KEY按键值

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

float ADC_Value;
float DS_temp;
uint16_t Frq_value;
float Duty_value;

uint8_t ADC_Over_flag, DS_Over_flag, Frq_Over_flag;

uint8_t Key_Lock;       // 按键上锁标志位
uint8_t Record_Ok_flag; // 记录完成标志位

#define ARRAY_SIZE 150

float Record_ADC_R37[ARRAY_SIZE], Record_Duty_PA1[ARRAY_SIZE];
uint16_t Record_Frq_PA1[ARRAY_SIZE];

uint8_t Record_ADC_index, Record_Frq_index, Record_Duty_index;
uint8_t index_old;

uint8_t maichong_flag;
uint8_t dianya_flag;

float K, B;

uint16_t Duty_set;

typedef struct
{
  uint16_t FH;
  float AH;
  int16_t TH;

  uint16_t FH_buff;
  float AH_buff;
  int16_t TH_buff;
} PARA;

typedef struct
{
  uint16_t FN;
  uint16_t AN;
  uint16_t TN;

} RECD;

typedef struct
{
  uint16_t FP;
  float VP;
  uint16_t TT;

  uint16_t FP_buff;
  float VP_buff;
  uint16_t TT_buff;
} FSET;

PARA PARA_T;
RECD RECD_T;
FSET FSET_T;

/*拓展变量区*/
extern uint8_t single_key_flag[4]; // 按键单击
extern uint8_t double_key_flag[4]; // 按键双击
extern uint8_t long_key_flag[4];   // 按键长按
extern uint16_t key_time[4];
extern uint8_t single_ADC_key_flag[8]; // ADC按键单击
extern uint8_t double_ADC_key_flag[8]; // ADC按键双击
extern uint8_t long_ADC_key_flag[8];   // ADC按键长按

extern uint16_t frq1, frq2; // 输入捕获测量的频率
extern float duty1, duty2;  // 占空比

// extern uint32_t RP[4]; // 轮询ADC采集值或者占空比值

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
void Record_Proc_1(void);
void Record_Proc_2(void);
void imfor_recorve_MC_1(void);
void imfor_recorve_MC_2(void);
void imfor_recorve_DY_1(void);
void imfor_recorve_DY_2(void);
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

  /*定时器6基本中断开启,ADC按键中断定时器 时间到就进中断检测按键是否按下*/
  HAL_TIM_Base_Start_IT(&htim6);

  /*定时器17通道1 PWM生成,PA7 引脚PWM生成定时器*/
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  /*定时器2通道1 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /*定时器2通道2 输入捕获中断开启,PA15引脚*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2，PB15引脚

  /********************************串口接收中断开启***********************************************************/
  HAL_UART_Receive_IT(&huart1, &rxdat, 1);

  /********************************EEPROM存储读取***********************************************************/
  I2C_24c02_Read((uint8_t *)&EEPROM_Save[3], 0x37, 1); // 读取一次判断是否第一次上电

  if (EEPROM_Save[3] != 9) // 初次上电 占空比为10，10
  {
    EEPROM_Save[3] = 9; // 将该位的值改为9写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的阈值 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[3], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 2); // 将阈值写入EEPROM中
  }
  else // 再次上电读取eeprom中设置的阈值 更新显示的阈
  {
    I2C_24c02_Read(EEPROM_Save, 0x00, 2); // 读取EEPROM中的值
    HAL_Delay(10);
    EEPROM_Disp[0] = EEPROM_Save[0];
    EEPROM_Disp[1] = EEPROM_Save[1];
  }
  /********************************EEPROM存储读取***********************************************************/

  PARA_T.FH = PARA_T.FH_buff = 2000;
  PARA_T.AH = PARA_T.AH_buff = 3.0f;
  PARA_T.TH = PARA_T.TH_buff = 30;

  RECD_T.AN = 0;
  RECD_T.FN = 0;
  RECD_T.TN = 0;

  FSET_T.FP = FSET_T.FP_buff = 1;
  FSET_T.TT = FSET_T.TT_buff = 6;
  FSET_T.VP = FSET_T.VP_buff = 0.9f;

  Frq_value = frq1;
  Duty_value = duty1;

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
    Record_Proc_1();
    imfor_recorve_MC_1();
    imfor_recorve_DY_1();

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

  if (Key_Lock == 1)
    ucLED ^= 0x01;
  else
    ucLED &= 0xFE;

  if (maichong_flag)
    ucLED ^= 0x02;
  else
    ucLED &= 0xFD;

  if (dianya_flag)
    ucLED ^= 0x04;
  else
    ucLED &= 0xFB;

  if (Frq_Over_flag)
    ucLED |= 0x08;
  else
    ucLED &= 0xF7;

  if (ADC_Over_flag)
    ucLED |= 0x10;
  else
    ucLED &= 0xEF;

  if (DS_Over_flag)
    ucLED |= 0x20;
  else
    ucLED &= 0xDF;

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 100)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  ADC_Value = Get_ADC(&hadc2);
  DS_temp = ds18b20Read();

  if (Frq_value > PARA_T.FH && Frq_Over_flag == 0)
  {
    RECD_T.FN++;
    Frq_Over_flag = 1;
  }
  else if (Frq_value < PARA_T.FH)
  {
    Frq_Over_flag = 0;
  }

  if (ADC_Value > PARA_T.AH && ADC_Over_flag == 0)
  {
    RECD_T.AN++;
    ADC_Over_flag = 1;
  }
  else if (ADC_Value < PARA_T.AH)
  {
    ADC_Over_flag = 0;
  }

  if (DS_temp > PARA_T.TH && DS_Over_flag == 0)
  {
    RECD_T.TN++;
    DS_Over_flag = 1;
  }
  else if (DS_temp < PARA_T.TH)
  {
    DS_Over_flag = 0;
  }

  if (LCD_Disp_Change == 0)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "DATA  ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "F=%d    ", Frq_value);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "D=%d%%    ", (int)Duty_value);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "A=%.1f    ", ADC_Value);
    LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "T=%.1f    ", DS_temp);
    LCD_DisplayStringAtLineColumn(Line7, Col6, LCD_String_Disp);
  }
  else if (LCD_Disp_Change == 1)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PARA    ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "FH=%d    ", PARA_T.FH_buff);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "AH=%.1f    ", PARA_T.AH_buff);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "TH=%d   ", PARA_T.TH_buff);
    LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);
  }
  else if (LCD_Disp_Change == 2)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "RECD  ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "FN=%d    ", RECD_T.FN);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "AN=%d    ", RECD_T.AN);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "TN=%d    ", RECD_T.TN);
    LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);
  }
  else
  {

    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "FSET  ");
    LCD_DisplayStringAtLineColumn(Line2, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "FP=%d    ", FSET_T.FP_buff);
    LCD_DisplayStringAtLineColumn(Line4, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "VP=%.1f    ", FSET_T.VP_buff);
    LCD_DisplayStringAtLineColumn(Line5, Col6, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "TT=%d    ", FSET_T.TT_buff);
    LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);
  }
}

/*****************************************按键控制函数***************************************************/

void KEY_Proc(void)
{
  if ((uwTick - uwTick_KEY_Speed_Ctrl) < 50)
    return;
  uwTick_KEY_Speed_Ctrl = uwTick; // 50ms执行一次

  if (Key_Lock == 0)
  {
    if (single_key_flag[0] == 1) // 按键1单击刷新显示屏
    {
      LCD_Disp_Change += 1; // 0-实时数据界面 1-报警参数界面  2-报警统计界面 3-回放设置界面
      if (LCD_Disp_Change > 3)
        LCD_Disp_Change = 0;

      LCD_Line_Change = 1; // 切换界面默认选择行为第一行

      PARA_T.AH = PARA_T.AH_buff; // 按下按键1切换界面时，设置的值生效
      PARA_T.FH = PARA_T.FH_buff;
      PARA_T.TH = PARA_T.TH_buff;

      FSET_T.FP = FSET_T.FP_buff;
      FSET_T.TT = FSET_T.TT_buff;
      FSET_T.VP = FSET_T.VP_buff;

      LCD_Clear(Black);
      single_key_flag[0] = 0;
    }

    if (single_key_flag[1] == 1) // 按键2单击
    {
      if (LCD_Disp_Change == 0) // 在实时数据界面  “记录”按键
      {
        // 按下按键后，系统开始记录R37输出电压和输入到PA1引脚的脉冲信号频率，占空比
        // 记录完成前，所有按键失效，处于“锁定”状态，记录完成后恢复，只保留一组记录数据
        Key_Lock = 1;
      }

      if (LCD_Disp_Change == 1) // 在报警参数界面
      {
        // 切换选择(FH)(AH)(TH)
        LCD_Line_Change += 1;
        if (LCD_Line_Change > 3)
          LCD_Line_Change = 1;
        // 退出报警参数界面，设置生效
      }

      if (LCD_Disp_Change == 2) // 在报警统计界面
      {
        // 清零所有统计次数
        RECD_T.AN = 0;
        RECD_T.FN = 0;
        RECD_T.TN = 0;
      }

      if (LCD_Disp_Change == 3) // 在回放设置界面
      {
        // 切换选择(FP)(VP)(TT)
        LCD_Line_Change += 1;
        if (LCD_Line_Change > 3)
          LCD_Line_Change = 1;
        // 退出回放设置界面，设置生效
      }

      single_key_flag[1] = 0;
    }

    if (single_key_flag[2] == 1) // 按键3单击
    {
      if (LCD_Disp_Change == 0) // 实时数据界面
      {
        // 若设备完成了数据记录 则通过PA7引脚回放“电压信号”
        if (Record_Ok_flag && maichong_flag == 0)
        {
          dianya_flag = 1;
        }
      }

      if (LCD_Disp_Change == 1) // 报警参数界面
      {
        // 根据选择行  对应行++
        // FH += 1000  AH += 0.3 TH += 1
        if (LCD_Line_Change == 1)
        {
          PARA_T.FH_buff += 1000;
          if (PARA_T.FH_buff > 10000)
            PARA_T.FH_buff = 10000;
        }

        if (LCD_Line_Change == 2)
        {
          PARA_T.AH_buff += 0.3f;
          if (PARA_T.AH_buff > 3.3f)
            PARA_T.AH_buff = 3.3f;
        }

        if (LCD_Line_Change == 3)
        {
          PARA_T.TH_buff += 1;
          if (PARA_T.TH_buff > 80)
            PARA_T.TH_buff = 80;
        }
      }

      if (LCD_Disp_Change == 3) // 回放设置界面
      {
        // 根据选择行  对应行++
        // FP += 1  VP += 0.3 TT += 2
        if (LCD_Line_Change == 1)
        {
          FSET_T.FP_buff += 1;
          if (FSET_T.FP_buff > 10)
            FSET_T.FP_buff = 10;
        }

        if (LCD_Line_Change == 2)
        {
          FSET_T.VP_buff += 0.3f;
          if (FSET_T.VP_buff > 3.3f)
            FSET_T.VP_buff = 3.3f;
        }

        if (LCD_Line_Change == 3)
        {
          FSET_T.TT_buff += 2;
          if (FSET_T.TT_buff > 10)
            FSET_T.TT_buff = 10;
        }
      }
      single_key_flag[2] = 0;
    }

    if (single_key_flag[3] == 1) // 按键4单击
    {
      if (LCD_Disp_Change == 0) // 实时数据界面
      {
        // 若设备完成了数据记录 则通过PA7引脚回放“脉冲信号”
        if (Record_Ok_flag && dianya_flag == 0)
        {
          maichong_flag = 1;
        }
      }

      if (LCD_Disp_Change == 1) // 报警参数界面
      {
        // 根据选择行  对应行--
        // FH -= 1000  AH -= 0.3 TH -= 1
        if (LCD_Line_Change == 1)
        {
          PARA_T.FH_buff -= 1000;
          if (PARA_T.FH_buff < 1000)
            PARA_T.FH_buff = 1000;
        }

        if (LCD_Line_Change == 2)
        {
          PARA_T.AH_buff -= 0.3f;
          if (PARA_T.AH_buff < 0.0f)
            PARA_T.AH_buff = 0.0f;
        }

        if (LCD_Line_Change == 3)
        {
          PARA_T.TH_buff -= 1;
          if (PARA_T.TH_buff < 0)
            PARA_T.TH_buff = 0;
        }
      }

      if (LCD_Disp_Change == 3) // 回放设置界面
      {
        // 根据选择行  对应行--
        // FP -= 1  VP -= 0.3 TT -= 2
        if (LCD_Line_Change == 1)
        {
          FSET_T.FP_buff -= 1;
          if (FSET_T.FP_buff < 1)
            FSET_T.FP_buff = 1;
        }

        if (LCD_Line_Change == 2)
        {
          FSET_T.VP_buff -= 0.3f;
          if (FSET_T.VP_buff < 0.0f)
            FSET_T.VP_buff = 0.0f;
        }

        if (LCD_Line_Change == 3)
        {
          FSET_T.TT_buff -= 2;
          if (FSET_T.TT_buff < 2)
            FSET_T.TT_buff = 2;
        }
      }
      single_key_flag[3] = 0;
    }

    //
    //
    // 在任何界面下 所检测到B3 B4均处于按下状态且时间超过2s 回到初始状态
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 &&
        key_time[2] > 200 && key_time[3] > 200)
    {

      PARA_T.FH = PARA_T.FH_buff = 2000;
      PARA_T.AH = PARA_T.AH_buff = 3.0f;
      PARA_T.TH = PARA_T.TH_buff = 30;

      RECD_T.AN = 0;
      RECD_T.FN = 0;
      RECD_T.TN = 0;

      FSET_T.FP = FSET_T.FP_buff = 1;
      FSET_T.TT = FSET_T.TT_buff = 6;
      FSET_T.VP = FSET_T.VP_buff = 0.9f;

      long_key_flag[2] = 0;
      long_key_flag[3] = 0;

      LCD_Disp_Change = 0; // 回到初始界面
    }
  }

  if (Key_Lock)
  {
    single_key_flag[0] = 0;
    single_key_flag[1] = 0;
    single_key_flag[2] = 0;
    single_key_flag[3] = 0;
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

void Record_Proc_1(void) // 记录处理函数
{

  if (Key_Lock == 1 && (uwTick - uwTick_Time_Count) < FSET_T.TT * 1000)
  {
    Record_Proc_2();
  }
  else if (Key_Lock == 1 && (uwTick - uwTick_Time_Count) > FSET_T.TT * 1000)
  {
    Key_Lock = 0;
    Record_Ok_flag = 1;
    index_old = Record_ADC_index;

    Record_ADC_index = 0;
    Record_Frq_index = 0;
    Record_Duty_index = 0;

    printf("ok\r\n");
  }
  else if (Key_Lock == 0)
  {
    uwTick_Time_Count = uwTick;
  }
}

void Record_Proc_2(void) // 记录处理函数
{
  if ((uwTick - uwTick_Record_Speed_Ctrl) < 50)
    return;
  uwTick_Record_Speed_Ctrl = uwTick; // 50ms执行一次

  Record_ADC_R37[Record_ADC_index++] = ADC_Value;
  Record_Frq_PA1[Record_Frq_index++] = Frq_value;
  Record_Duty_PA1[Record_Duty_index++] = Duty_value;
}

void imfor_recorve_MC_1(void)
{

  if (maichong_flag == 1 && (uwTick - uwTick_Time_Count_IM_MC) < FSET_T.TT * 1000)
  {
    imfor_recorve_MC_2();
  }
  else if (maichong_flag == 1 && (uwTick - uwTick_Time_Count_IM_MC) > FSET_T.TT * 1000)
  {
    maichong_flag = 0;
    Record_Duty_index = 0;
    Record_ADC_index = 0;
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 0);
    printf("ok\r\n");
  }
  else if (maichong_flag == 0)
  {
    uwTick_Time_Count_IM_MC = uwTick;
  }
}

void imfor_recorve_MC_2(void)
{
  if ((uwTick - uwTick_IM_MC_Speed_Ctrl) < 50)
    return;
  uwTick_IM_MC_Speed_Ctrl = uwTick; // 50ms执行一次

  PWM_Set_Duty(&htim17, TIM_CHANNEL_1, Record_Duty_PA1[Record_Duty_index++]);
  PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, Record_Frq_PA1[Record_ADC_index++] / FSET_T.FP);
}

void imfor_recorve_DY_1(void)
{

  if (dianya_flag == 1 && (uwTick - uwTick_Time_Count_IM_DY) < FSET_T.TT * 1000)
  {
    imfor_recorve_DY_2();
  }
  else if (dianya_flag == 1 && (uwTick - uwTick_Time_Count_IM_DY) > FSET_T.TT * 1000)
  {
    dianya_flag = 0;
    Record_ADC_index = 0;
    printf("ok\r\n");
  }
  else if (dianya_flag == 0)
  {
    uwTick_Time_Count_IM_DY = uwTick;
  }
}

void imfor_recorve_DY_2(void)
{
  if ((uwTick - uwTick_IM_DY_Speed_Ctrl) < 50)
    return;
  uwTick_IM_DY_Speed_Ctrl = uwTick; // 50ms执行一次

  if (Record_ADC_R37[Record_ADC_index++] > FSET_T.VP)
  {
    K = 90.0f / (3.3f - FSET_T.VP);
    B = 100.0f - (297.0f / (3.3f - FSET_T.VP));
    Duty_set = (uint16_t)(K * Record_ADC_R37[Record_ADC_index - 1] + B);
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, Duty_set);
  }
  else if (Record_ADC_R37[Record_ADC_index++] < FSET_T.VP)
  {
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 10);
  }
  else if (Record_ADC_R37[Record_ADC_index++] > 3.3f)
  {
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 100);
  }

  PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, 1000);
}

/*RTC时钟配置*/

void RTC_Proc(void)
{
  if ((uwTick - uwTick_RTC_Speed_Ctrl) < 100)
    return;
  uwTick_RTC_Speed_Ctrl = uwTick; // 100ms执行一次

  Frq_value = frq1;
  Duty_value = duty1;

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
