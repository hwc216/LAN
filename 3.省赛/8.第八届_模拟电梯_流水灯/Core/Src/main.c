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

__IO uint32_t uwTick_count_time;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*全局变量区*/
uint8_t ucLED;
uint32_t lastTick = 0;
uint32_t LED_Delay = 200; // LED更新间隔设置为200毫秒

uint8_t LCD_String_Disp[21];  // 显示屏显示数据存储数组
uint8_t LCD_Disp_Changle = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Changle = 0; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[4] = {10, 10}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10}; // EEPROM数据显示数组

uint8_t Current_platform = 1;
uint8_t Target_platform;
uint8_t Statue_flag = 0;
uint8_t Up_flag, Down_flag;
uint8_t LED_left = 4, LED_right = 7;
uint8_t Key_press_flag;

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
void DT_Proc(void);
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

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1，PB12引脚
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
    DT_Proc();

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
  sprintf((char *)LCD_String_Disp, "Current platform");
  LCD_DisplayStringAtLineColumn(Line2, Col3, LCD_String_Disp);

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
  sprintf((char *)LCD_String_Disp, "%d", Current_platform);
  LCD_DisplayStringAtLineColumn(Line4, Col10, LCD_String_Disp);

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
  sprintf((char *)LCD_String_Disp, "%02d: %02d: %02d", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
  LCD_DisplayStringAtLineColumn(Line6, Col6, LCD_String_Disp);

  memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
  sprintf((char *)LCD_String_Disp, "Statue_flag:%d", Statue_flag);
  LCD_DisplayStringAtLineColumn(Line8, Col3, LCD_String_Disp);
}
/*****************************************按键控制函数***************************************************/
void KEY_Proc(void)
{
  if ((uwTick - uwTick_KEY_Speed_Ctrl) < 50)
    return;
  uwTick_KEY_Speed_Ctrl = uwTick;

  if (Statue_flag == 1 || Statue_flag == 0) // 在状态1和状态2都能进行按键操作
  {
    if (single_key_flag[0] == 1) // 按键1单击
    {
      if (Current_platform != 1)
      {
        ucLED |= 0x01;
        uwTick_count_time = uwTick;
        Target_platform |= 0x01;
        Key_press_flag = 1;
      }
      single_key_flag[0] = 0;
    }

    if (single_key_flag[1] == 1) // 按键2单击
    {
      if (Current_platform != 2)
      {
        ucLED |= 0x02;
        uwTick_count_time = uwTick;
        Target_platform |= 0x02;
        Key_press_flag = 1;
      }
      single_key_flag[1] = 0;
    }

    if (single_key_flag[2] == 1) // 按键3单击
    {
      if (Current_platform != 3)
      {
        ucLED |= 0x04;
        uwTick_count_time = uwTick;
        Target_platform |= 0x04;
        Key_press_flag = 1;
      }
      single_key_flag[2] = 0;
    }

    if (single_key_flag[3] == 1) // 按键4单击
    {
      if (Current_platform != 4)
      {
        ucLED |= 0x08;
        uwTick_count_time = uwTick;
        Target_platform |= 0x08;
        Key_press_flag = 1;
      }
      single_key_flag[3] = 0;
    }

    if (Key_press_flag)
      Statue_flag = 1;
  }
}

void DT_Proc(void)
{
  if (Statue_flag)
  {
    switch (Statue_flag)
    {
    case 1: // 判断是否最后一个按键按下且1s
      if (uwTick - uwTick_count_time > 1000)
      {
        Statue_flag = 2;
        Key_press_flag = 0;
      }
      else
        Statue_flag = 1;
      break;

    case 2:                                     // 电梯关门，持续4s
      PWM_Set_Duty(&htim2, TIM_CHANNEL_1, 0);   // PA5引脚低电平 关门
      PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 50); // PA7引脚 2Khz 50%占空比
      uwTick_count_time = uwTick;
      Statue_flag = 3;
      break;

    case 3:
      if (uwTick - uwTick_count_time > 4000)
      {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // 停止PA5输出
        HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // 停止PA7输出
        Statue_flag = 4;
      }
      break;

    case 4:
      if (Target_platform > (1 << (Current_platform - 1)))
      {
        Up_flag = 1;
        PWM_Set_Duty(&htim2, TIM_CHANNEL_1, 100);
        PWM_Set_Duty(&htim16, TIM_CHANNEL_1, 80);
      }
      else if (Target_platform < (1 << (Current_platform - 1)))
      {
        Down_flag = 1;
        PWM_Set_Duty(&htim2, TIM_CHANNEL_1, 0);
        PWM_Set_Duty(&htim16, TIM_CHANNEL_1, 60);
      }
      uwTick_count_time = uwTick;
      Statue_flag = 5;
      break;

    case 5:
      if (uwTick - uwTick_count_time >= 6000)
      {
        if (Up_flag == 1)
        {
          Current_platform += 1;
          Up_flag = 0;
        }
        else if (Down_flag == 1)
        {
          Current_platform -= 1;
          Down_flag = 0;
        }
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        ucLED &= 0x0f;
        LED_left = 4;
        LED_right = 7;

        Statue_flag = 6;
      }
      else
      {
        uint32_t currentTick = HAL_GetTick();      // 获取当前滴答计数
        if ((currentTick - lastTick) >= LED_Delay) // 检查是否已过200ms
        {
          lastTick = currentTick; // 更新上次更新的时间

          if (Up_flag)
          {
            ucLED &= 0x0f;
            ucLED |= (1 << LED_left);
            LED_left++;
            if (LED_left == 8)
              LED_left = 4;
          }
          else if (Down_flag)
          {
            ucLED &= 0x0f;
            ucLED |= (1 << LED_right);
            LED_right--;
            if (LED_right == 3)
              LED_right = 7;
          }
        }
      }
      break;

    case 6:
      if ((Target_platform & (1 << (Current_platform - 1))) == (1 << (Current_platform - 1))) // 判断是否到达设置的楼层
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100); // 开门 PA5 高电平
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);          // TIM2_CH1 -> PA5 高电平 升降器开门

        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 60);
        HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // TIM17_CH1 -> PA7 PWM输出频率2khz 开门 duty：60%

        Target_platform &= ~(1 << (Current_platform - 1)); // 当前设置楼层到达 清除该楼层
        ucLED &= ~(1 << (Current_platform - 1));           // 对应到达楼层LED熄灭
        uwTick_Time_Count = uwTick;                        // 开始计时 用于判断开门时间6s
        Statue_flag = 7;                                   // 进入状态7
      }
      else // 没有到达设置楼层
      {
        Statue_flag = 4; // 返回状态4
      }
      break;

    case 7:                                     // 状态7
      if ((uwTick - uwTick_Time_Count) >= 4000) // 等待关门时间4s
      {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // 关闭 PA5
        HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // 关闭PA7 PWM 信号
        uwTick_Time_Count = uwTick;               // 开始计时 用于判断如果还有其他楼层时计时2s
        Statue_flag = 8;                          // 进入状态8
      }
      break;

    case 8:                // 状态8
      if (Target_platform) // 还有其他需要到达的楼层
      {
        if ((uwTick - uwTick_Time_Count) >= 2000) // 等待2s后关门
          Statue_flag = 2;                        // 返回状态2
      }
      else // 无其他楼层
      {
        Statue_flag = 0; // 回到状态0 等待按键设置需要到达的楼层信息
      }
      break;
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
