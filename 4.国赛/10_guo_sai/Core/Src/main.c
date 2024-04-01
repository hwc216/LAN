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

uint8_t EEPROM_Save[6] = {5, 3, 5, 5, 6}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10};        // EEPROM数据显示数组

uint8_t segBuff[3];    // Seg数码管显示数组
uint8_t seg_disp_flag; // 2s切换显示标志位

ADC2_ChannelValues adcValues; // ADC多通道扫描数值结构体

int Ds_Temp_for_seg; // 数码管显示ds18b20温度值  整形
float DS_Temp;       // ds18b20温度值
uint8_t T_Value = 30;
uint8_t T_Value_old = 30;

uint8_t Channel_Num = 1;
uint8_t Channel_Num_old = 1;

uint16_t Record_Num; // 记录次数

float AO1_Value, AO2_Value; // RP5 ADC值   RP6 ADC值

uint8_t PWM2_duty_Value; // 占空比

uint8_t Auto_upload_flag; // 自动上报标志位

uint8_t LED8_flag;

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

/*拓展变量区*/
extern uint8_t single_key_flag[4]; // 按键单击
extern uint8_t double_key_flag[4]; // 按键双击
extern uint8_t long_key_flag[4];   // 按键长按
extern uint16_t key_time[4];
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
void USART_TR_Proc(void);

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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
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

  /*定时器3通道2 输入捕获中断开启,PA7引脚*/
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1，PB12引脚
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2

  /********************************串口接收中断开启***********************************************************/
  HAL_UART_Receive_IT(&huart1, &rxdat, 1);

  /********************************EEPROM存储读取***********************************************************/
  I2C_24c02_Read((uint8_t *)&EEPROM_Save[5], 0x37, 1); // 读取一次判断是否第一次上电

  if (EEPROM_Save[5] != 4) // 初次上电 占空比为10，10
  {
    EEPROM_Save[5] = 4; // 将该位的值改为9写入eeprom中 用于判断非第一次上电 确保再次上电读取eeprom中设置的阈值 以免第一次直接赋值0
    I2C_24c02_Write((uint8_t *)&EEPROM_Save[5], 0x37, 1);
    HAL_Delay(10);
    I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 5); // 将阈值写入EEPROM中
    HAL_Delay(10);
  }
  else // 再次上电读取eeprom中设置的阈值 更新显示的阈
  {
    I2C_24c02_Read((uint8_t *)EEPROM_Save, 0x00, 5); // 读取EEPROM中的值
    HAL_Delay(10);
    Record_Num = EEPROM_Save[0] + EEPROM_Save[1] * 10 + EEPROM_Save[2] * 100 + EEPROM_Save[3] * 1000 + EEPROM_Save[4] * 10000;
  }
  /********************************EEPROM存储读取***********************************************************/
  ds18b20Read();
  DS_Temp = ds18b20Read();

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
    USART_TR_Proc();

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

  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  if (Auto_upload_flag)
    ucLED |= 0x01;
  else
    ucLED &= 0x80;

  if (LED8_flag == 1)
    ucLED |= 0x80;
  else
    ucLED &= 0x01;

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 100)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  AO1_Value = Return_3_3(adcValues.AdcChannel17Value);
  AO2_Value = Return_3_3(adcValues.AdcChannel13Value);
  PWM2_duty_Value = RP[1];

  if (seg_disp_flag)
  {
    segBuff[1] = T_Value / 10 % 10;
    segBuff[2] = T_Value % 10;
    SEG_Display(12, segBuff[1], segBuff[2], 0, 0, 0);
  }
  else
  {
    SEG_Display(10, 0, Channel_Num, 0, 0, 0);
  }

  if (Channel_Num == 1)
  {
    if (AO1_Value > (PWM2_duty_Value * 3.3f) / 100.0f)
    {
      Auto_upload_flag = 1;
    }
    else
      Auto_upload_flag = 0;
  }
  else if (Channel_Num == 2)
  {
    if (AO2_Value > PWM2_duty_Value * 3.3f / 100.0f)
    {
      Auto_upload_flag = 1;
    }
    else
      Auto_upload_flag = 0;
  }

  if (DS_Temp > T_Value)
    LED8_flag = 1;
  else
    LED8_flag = 0;

  if (LCD_Disp_Change == 0)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Main  ");
    LCD_DisplayStringAtLineColumn(Line1, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "AO1:%.2fV  ", AO1_Value);
    LCD_DisplayStringAtLineColumn(Line3, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "AO2:%.2fV  ", AO2_Value);
    LCD_DisplayStringAtLineColumn(Line4, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "PWM2:%d%%  ", PWM2_duty_Value);
    LCD_DisplayStringAtLineColumn(Line5, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Temp:%.2fC  ", DS_Temp);
    LCD_DisplayStringAtLineColumn(Line6, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "N:%d  ", Record_Num);
    LCD_DisplayStringAtLineColumn(Line7, Col3, LCD_String_Disp);
  }
  else
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Para  ");
    LCD_DisplayStringAtLineColumn(Line1, Col9, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "T: %d  ", T_Value);
    if (LCD_Line_Change == 1)
      LCD_SetTextColor(Green);
    else
      LCD_SetTextColor(White);
    LCD_DisplayStringAtLineColumn(Line3, Col3, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "X: A0%d  ", Channel_Num);
    if (LCD_Line_Change == 2)
      LCD_SetTextColor(Green);
    else
      LCD_SetTextColor(White);
    LCD_DisplayStringAtLineColumn(Line4, Col3, LCD_String_Disp);
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
    LCD_Disp_Change ^= 1; // 0-数据界面 1-参数界面
    // 检测参数是否发生变化，存入EEPROM
    if (LCD_Disp_Change == 0) // 返回主界面时保存设置的值
    {
      if (T_Value_old != T_Value || Channel_Num_old != Channel_Num)
      {
        Record_Num++;
        T_Value_old = T_Value;
        Channel_Num_old = Channel_Num;
        EEPROM_Save[4] = Record_Num / 10000;
        EEPROM_Save[3] = Record_Num / 1000 % 10;
        EEPROM_Save[2] = Record_Num / 100 % 10;
        EEPROM_Save[1] = Record_Num / 10 % 10;
        EEPROM_Save[0] = Record_Num % 10;
        I2C_24c02_Write((uint8_t *)EEPROM_Save, 0x00, 5); // 将变化次数写入EEPROM中
      }
      LCD_Line_Change = 0;
    }
    LCD_Clear(Black);
    single_key_flag[0] = 0;
  }

  if (single_key_flag[1] == 1) // 按键2单击
  {
    if (LCD_Disp_Change) // 参数界面 选择高亮
    {
      LCD_Line_Change++; // 选择高亮显示行
      if (LCD_Line_Change == 3)
      {
        LCD_Line_Change = 1;
      }
    }
    single_key_flag[1] = 0;
  }

  if (single_key_flag[2] == 1) // 按键3单击
  {
    if (LCD_Disp_Change) // 参数界面
    {
      if (LCD_Line_Change == 1) // 设定第一行
      {
        // 温度参数加1 范围20-40
        T_Value++;
        if (T_Value > 40)
          T_Value = 20;
      }
      else if (LCD_Line_Change == 2) // 设定第二行
      {
        // 切换通道 AO1 或者 AO2
        Channel_Num += 1;
        if (Channel_Num > 2)
          Channel_Num = 1;
      }
    }
    single_key_flag[2] = 0;
  }

  if (single_key_flag[3] == 1) // 按键4单击
  {
    if (LCD_Disp_Change) // 参数界面
    {
      if (LCD_Line_Change == 1) // 设定第一行
      {
        // 温度参数减1  范围20-40
        T_Value--;
        if (T_Value < 20)
          T_Value = 40;
      }
      else if (LCD_Line_Change == 2) // 设定第二行
      {
        // 切换通道 AO1 或者 AO2
        Channel_Num += 1;
        if (Channel_Num > 2)
          Channel_Num = 1;
      }
    }
    single_key_flag[3] = 0;
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0 && key_time[2] > 50) // 按键3长按
  {
    if (LCD_Disp_Change) // 参数界面
    {
      // 温度参数快速递增  范围20-40
      T_Value++;
      if (T_Value > 40)
        T_Value = 20;
    }
    long_key_flag[2] = 0;
  }

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 && key_time[3] > 50) // 按键4长按
  {
    if (LCD_Disp_Change) // 参数界面
    {
      // 温度参数快速递减  范围20-40
      T_Value--;
      if (T_Value < 20)
        T_Value = 40;
    }
    long_key_flag[3] = 0;
  }
}

/*****************************************ADC处理函数***************************************************/

void ADC_Proc(void)
{
  adcValues = Read_ADC2_Channels();
}

/*****************************************串口接收函数***************************************************/

void USART_Proc(void)
{

  if (rx_pointer > 0)
  {
    if (rx_pointer == 4)
    {
      if (rxdata[0] == 'S' && rxdata[1] == 'T' && rxdata[2] == '\r' && rxdata[3] == '\n')
      {
        printf("$%.2f\r\n", DS_Temp);
      }
    }

    if (rx_pointer == 6)
    {
      if (rxdata[0] == 'P' && rxdata[1] == 'A' && rxdata[2] == 'R' && rxdata[3] == 'A' && rxdata[4] == '\r' && rxdata[5] == '\n')
      {
        printf("#%d,A0%d\r\n", T_Value, Channel_Num);
      }
    }

    rx_pointer = 0;
    memset(rxdata, 0, 30);
  }
}

void USART_TR_Proc(void)
{

  if ((uwTick - uwTick_Time_Count) < 1000)
    return;
  uwTick_Time_Count = uwTick; // 1000ms执行一次

  DS_Temp = ds18b20Read();

  if (Channel_Num == 1)
  {
    if (Auto_upload_flag)
    {
      printf("$%.2f\r\n", DS_Temp);
    }
  }
  else if (Channel_Num == 2)
  {
    if (Auto_upload_flag)
    {
      printf("$%.2f\r\n", DS_Temp);
    }
  }
}

/*RTC时钟配置*/

void RTC_Proc(void)
{
  if ((uwTick - uwTick_RTC_Speed_Ctrl) < 2000)
    return;
  uwTick_RTC_Speed_Ctrl = uwTick; // 1s执行一次

  // // 获取当前时间和日期
  // HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
  // HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);

  // printf("Date: %02d-%02d-%02d Time: %02d:%02d:%02d\r\n", RTC_Date.Year, RTC_Date.Month, RTC_Date.Date,
  //        RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);

  seg_disp_flag ^= 0x01;
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
