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

uint8_t LCD_String_Disp[21]; // 显示屏显示数据存储数组
uint8_t LCD_Disp_Change = 0; // 显示屏切换界面变量
uint8_t LCD_Line_Change = 0; // 显示屏选择行 1-第一行 2-第二行 3-第三行 默认为0不选择 无高亮文本

uint8_t PA6_duty = 10, PA7_duty = 10;
uint16_t PA6_Frq = 100, PA7_Frq = 200;

uint8_t EEPROM_Save[4] = {10, 10}; // EEPROM芯片中存取的数据
uint8_t EEPROM_Disp[4] = {10, 10}; // EEPROM数据显示数组

// 用于存储时间和日期的结构体
RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};
char dateTimeBuffer[50]; // 存储日期时间字符串

float CNBR_Tips = 3.5f, VNBR_Tips = 2.0f;
uint8_t CNBR_Num = 0, VNBR_Num = 0, IDLE_Num = 8;
float All_tips = 0.0f;
uint8_t PA7_Ctrl = 0;
char Rxdata_Check[];

uint8_t Car_location_ok = 0;
uint8_t Car_In_out = 0;
long long All_Time_Sec = 0;
uint32_t All_Time_hour = 0;

typedef struct
{
  uint8_t Year;
  uint8_t Month;
  uint8_t Day;
  uint8_t Hour;
  uint8_t Min;
  uint8_t Sec;
  char Car_type[10];
  uint8_t is_null;

} Car;

Car Car_location[9];

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
void USART_Proc(void);
uint8_t Check_Proc(void);
uint8_t Check_Car_location(void);
uint8_t Check_Income_Outcome(void);

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

  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);

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

  if (PA7_Ctrl == 0)
  {
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 0);
    ucLED &= 0x01;
  }
  else
  {
    PWM_Set_Duty(&htim17, TIM_CHANNEL_1, 20);
    PWM_Set_Frequency(&htim17, TIM_CHANNEL_1, 2000); // 设置PWM频率为1kHz
    ucLED |= 0x02;
  }

  if (IDLE_Num > 0)
    ucLED |= 0x02;
  else
    ucLED &= 0x01;

  // ucLED ^= 0x01;  //异或运算 相同为0，不同为1

  LED_Disp(ucLED);
}

/*****************************************显示屏控制函数***************************************************/
void LCD_Proc(void)
{
  if ((uwTick - uwTick_LCD_Speed_Ctrl) < 300)
    return;
  uwTick_LCD_Speed_Ctrl = uwTick;

  if (LCD_Disp_Change == 0)
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Data  ");
    LCD_DisplayStringAtLineColumn(Line2, Col8, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "CNBR:%d  ", CNBR_Num);
    LCD_DisplayStringAtLineColumn(Line4, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "VNBR:%d  ", VNBR_Num);
    LCD_DisplayStringAtLineColumn(Line6, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "IDLE:%d  ", IDLE_Num);
    LCD_DisplayStringAtLineColumn(Line8, Col4, LCD_String_Disp);
  }
  else
  {
    LCD_SetTextColor(White); // 初始文本颜色 防止受上一个对文本颜色的改变影响

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "Para  ");
    LCD_DisplayStringAtLineColumn(Line2, Col8, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "CNBR:%.2f", CNBR_Tips);
    LCD_DisplayStringAtLineColumn(Line4, Col4, LCD_String_Disp);

    memset(LCD_String_Disp, 0, sizeof(LCD_String_Disp));
    sprintf((char *)LCD_String_Disp, "VNBR:%.2f", VNBR_Tips);
    LCD_DisplayStringAtLineColumn(Line6, Col4, LCD_String_Disp);
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
    LCD_Disp_Change ^= 1;     // 0-主界面 1-次界面
    if (LCD_Disp_Change == 0) // 返回主界面时保存设置的值
    {
    }
    LCD_Clear(Black);
    single_key_flag[0] = 0;
  }

  if (single_key_flag[1] == 1) // 按键2单击
  {
    if (LCD_Disp_Change)
    {
      CNBR_Tips += 0.5f;
      VNBR_Tips += 0.5f;
    }
    single_key_flag[1] = 0;
  }

  if (single_key_flag[2] == 1) // 按键3单击
  {
    if (LCD_Disp_Change)
    {
      CNBR_Tips -= 0.5f;
      VNBR_Tips -= 0.5f;
      if (CNBR_Tips < 0.0f)
      {
        CNBR_Tips = 0.0f;
      }

      if (VNBR_Tips < 0.0f)
      {
        VNBR_Tips = 0.0f;
      }
    }
    single_key_flag[2] = 0;
  }

  if (single_key_flag[3] == 1) // 按键4单击
  {
    PA7_Ctrl ^= 1;
    single_key_flag[3] = 0;
  }
}

/*****************************************串口接收函数***************************************************/
void USART_Proc(void)
{

  if (rx_pointer > 0)
  {
    if (rx_pointer == 22)
    {
      if (Check_Proc())
      {
        Car_In_out = Check_Income_Outcome();
        if (Car_In_out == 99)
        {
          Car_location_ok = Check_Car_location();
          if (Car_location_ok != 99)
          {
            extract_substring(rxdata, 0, 9, Car_location[Car_location_ok].Car_type);
            printf("Income_Car_Name:%s Position:%d ok! \r\n", Car_location[Car_location_ok].Car_type, Car_location_ok);
            Car_location[Car_location_ok].Year = (rxdata[10] - '0') * 10 + (rxdata[11] - '0');
            Car_location[Car_location_ok].Month = (rxdata[12] - '0') * 10 + (rxdata[13] - '0');
            Car_location[Car_location_ok].Day = (rxdata[14] - '0') * 10 + (rxdata[15] - '0');
            Car_location[Car_location_ok].Hour = (rxdata[16] - '0') * 10 + (rxdata[17] - '0');
            Car_location[Car_location_ok].Min = (rxdata[18] - '0') * 10 + (rxdata[19] - '0');
            Car_location[Car_location_ok].Sec = (rxdata[20] - '0') * 10 + (rxdata[21] - '0');
            Car_location[Car_location_ok].is_null = 1;
            if (rxdata[0] == 'C')
            {
              CNBR_Num++;
              IDLE_Num--;
            }
            else
            {
              VNBR_Num++;
              IDLE_Num--;
            }
          }
          else
          {
            printf("Position is Full\r\n");
          }
        }
        else
        {
          uint8_t year, month, day, hour, min, sec;
          char Sent_type[15];
          year = (rxdata[10] - '0') * 10 + (rxdata[11] - '0');
          month = (rxdata[12] - '0') * 10 + (rxdata[13] - '0');
          day = (rxdata[14] - '0') * 10 + (rxdata[15] - '0');
          hour = (rxdata[16] - '0') * 10 + (rxdata[17] - '0');
          min = (rxdata[18] - '0') * 10 + (rxdata[19] - '0');
          sec = (rxdata[20] - '0') * 10 + (rxdata[21] - '0');

          if ((year >= Car_location[Car_In_out].Year) && month >= Car_location[Car_In_out].Month && day >= Car_location[Car_In_out].Day &&
              hour >= Car_location[Car_In_out].Hour && min >= Car_location[Car_In_out].Min && sec >= Car_location[Car_In_out].Sec) // 出去时间必须大于等于进来时间
          {
            All_Time_Sec = timeDifference(Car_location[Car_In_out].Year + 2000, Car_location[Car_In_out].Month, Car_location[Car_In_out].Day, Car_location[Car_In_out].Hour, Car_location[Car_In_out].Min, Car_location[Car_In_out].Sec, year + 2000, month, day, hour, min, sec);

            if (All_Time_Sec % 3600 != 0)
            {
              All_Time_hour = All_Time_Sec / 3600 + 1;
            }
            else
            {
              All_Time_hour = All_Time_Sec / 3600;
            }

            if (rxdata[0] == 'C')
            {
              All_tips = All_Time_hour * CNBR_Tips;
              CNBR_Num--;
              IDLE_Num++;
            }
            else if (rxdata[0] == 'V')
            {
              All_tips = All_Time_hour * VNBR_Tips;
              VNBR_Num--;
              IDLE_Num++;
            }

            extract_substring(rxdata, 0, 9, Sent_type);
            printf("%s:%d:%.2f\r\n", Sent_type, All_Time_hour, All_tips);
            Car_location[Car_In_out].is_null = 0;
          }
          else
            printf("Error\r\n");
        }
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

uint8_t Check_Proc(void)
{
  uint8_t year, month, day, hour, min, sec;

  if (strncmp("CNBR", rxdata, 4) != 0 && strncmp("VNBR", rxdata, 4) != 0) // strncmp 检测前几位是否相等
    return 0;
  printf("1\r\n");

  if (rxdata[4] != ':' || rxdata[9] != ':')
    return 0;
  printf("2\r\n");

  if (checkStringSegment(rxdata, MODE_DIGITS, 10, 12) != 1)
    return 0;
  printf("4\r\n");

  year = (rxdata[10] - '0') * 10 + (rxdata[11] - '0');
  month = (rxdata[12] - '0') * 10 + (rxdata[13] - '0');
  day = (rxdata[14] - '0') * 10 + (rxdata[15] - '0');
  hour = (rxdata[16] - '0') * 10 + (rxdata[17] - '0');
  min = (rxdata[18] - '0') * 10 + (rxdata[19] - '0');
  sec = (rxdata[20] - '0') * 10 + (rxdata[21] - '0');

  // 检查月份
  if (month < 1 || month > 12)
  {
    return 0;
  }
  printf("5\r\n");

  // 检查每个月的天数，包括闰年2月的情况
  if (day < 1 || day > 31 ||
      (month == 2 && day > (isLeapYear(year) ? 29 : 28)) ||
      ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30))
  {
    return 0;
  }

  printf("6\r\n");

  // 检查小时、分钟和秒是否在有效范围内
  if (hour > 23 || min > 59 || sec > 59)
  {
    return 0;
  }

  printf("7\r\n");

  return 1;
}

uint8_t Check_Income_Outcome(void)
{
  char Income_Car_type[10];
  extract_substring(rxdata, 0, 9, Income_Car_type);

  for (int i = 0; i < 8; i++)
  {
    if ((strcmp(Car_location[i].Car_type, Income_Car_type) == 0) && Car_location[i].is_null == 1)
    {
      return i;
    }
  }

  return 99; // 当前车辆没有进入停车场
}

uint8_t Check_Car_location(void) // 检测是否还有车位，并返回车位的位置
{
  for (int i = 0; i < 8; i++)
  {
    if (Car_location[i].is_null != 1)
    {
      return i;
    }
  }

  return 99; // 无空车位
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
