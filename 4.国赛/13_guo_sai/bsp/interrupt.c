#include "stm32g4xx.h" // Device header
#include "interrupt.h"
#include "usart.h"
#include "string.h"
#include "ctype.h"
#include "stdio.h"
#include "time.h"

#define N 1

#if N == 0 // 使用主板上的端口来进行输入捕获  双通道捕获 频率和占空比
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
     *frq = (uint16_t)((1000000) / *ccrl_vala);
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

#elif N == 1 // 使用拓展版上的端口来进行输入捕获 捕获频率

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

// 保存PWM的数据值 RP[0]-PWM1的值 RP[1]-PWM2的值
uint32_t RP[4] = {0, 0, 0, 0};
// 定时器的回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
     // 保存TIMx_CCR的值
     uint32_t cclValue = 0;

     // 定时器16时执行该段
     if (htim->Instance == TIM16)
     {
          cclValue = __HAL_TIM_GET_COUNTER(&htim16);
          __HAL_TIM_SetCounter(&htim16, 0);
          RP[0] = 1000000 / cclValue;
          HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
     }
     // 定时器3时执行该段
     if (htim->Instance == TIM17)
     {
          cclValue = __HAL_TIM_GET_COUNTER(&htim17);
          __HAL_TIM_SetCounter(&htim17, 0);
          RP[1] = 1000000 / cclValue;
          HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_2);
     }

     // 定时器2时执行该段
     if (htim->Instance == TIM2)
     {
          cclValue = __HAL_TIM_GET_COUNTER(&htim2);
          __HAL_TIM_SetCounter(&htim2, 0);
          RP[2] = 1000000 / cclValue;
          HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
     }

     // 定时器15时执行该段
     if (htim->Instance == TIM15)
     {
          cclValue = __HAL_TIM_GET_COUNTER(&htim15);
          __HAL_TIM_SetCounter(&htim15, 0);
          RP[3] = 1000000 / cclValue;
          HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
     }
}

#elif N == 2 // 单通道双边捕获 测量占空比

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;

// 记录定时器上升沿 下降沿计数值
struct date
{
     // 记录计数值  偶数为上升沿 计数的下降沿
     uint32_t count[3];
     // 用于记录当前应该是上升沿触发还是下将沿触发
     char edge_flag;
     // 记录数据采集数量
     int number;
};

// 保存定时器输入捕获时需要的相关变量以及最终结果
struct date time16Data, time3Data;
// 保存PWM的数据值 RP[0]-PWM1的值  RP[1]-PWM2的值
uint32_t RP[4] = {0, 0, 0, 0};

// 定时器的回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
     // 保存TIMx_CCR的值
     uint32_t cclValue = 0;
     // 定时器16时执行该段
     if (htim->Instance == TIM16)
     {
          // 本次为上升沿触发
          if (time16Data.edge_flag % 2 == 0)
          {
               // 获取本次上升沿计数值
               time16Data.count[time16Data.number++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
               // 采集一次下降沿、两次上升沿数据完成
               if (time16Data.number == 3)
               {
                    RP[0] = (((time16Data.count[1] - time16Data.count[0]) * 1.0) / (time16Data.count[2] - time16Data.count[0])) * 100;
                    // 将定时器的计数值设置成0
                    __HAL_TIM_SetCounter(htim, 0);
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
                    // 重新开启定时器
                    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
                    time16Data.edge_flag = 0;
                    time16Data.number = 0;
               }
               else
               {
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
                    // 将下次触发方式设置为下降沿触发
                    time16Data.edge_flag += 1;
               }
          }
          // 下降沿触发
          else
          {
               // 读取本轮的计数值
               time16Data.count[time16Data.number++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
               // 修改触发方式为上升沿触发
               __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
               time16Data.edge_flag += 1;
          }

          HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
     }
     // 定时器3时执行该段
     if (htim->Instance == TIM3)
     {
          // 本次为上升沿触发
          if (time3Data.edge_flag % 2 == 0)
          {
               // 获取本次上升沿计数值
               time3Data.count[time3Data.number++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
               // 采集一次下降沿、两次上升沿数据完成
               if (time3Data.number == 3)
               {
                    RP[1] = (((time3Data.count[1] - time3Data.count[0]) * 1.0) / (time3Data.count[2] - time3Data.count[0])) * 100;
                    // 将定时器的计数值设置成0
                    __HAL_TIM_SetCounter(htim, 0);
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
                    // 重新开启定时器
                    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
                    time3Data.edge_flag = 0;
                    time3Data.number = 0;
               }
               else
               {
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
                    // 将下次触发方式设置为下降沿触发
                    time3Data.edge_flag += 1;
               }
          }
          // 下降沿触发
          else
          {
               // 读取本轮的计数值
               time3Data.count[time3Data.number++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
               // 修改触发方式为上升沿触发
               __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
               time3Data.edge_flag += 1;
          }
          HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
     }
     // 定时器2时执行该段
     if (htim->Instance == TIM2)
     {
          cclValue = __HAL_TIM_GET_COUNTER(&htim2);
          __HAL_TIM_SetCounter(&htim2, 0);
          RP[2] = 1000000 / cclValue;
          HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
     }
}

#endif

#define RX_BUFFER_SIZE 30 // 定义接收缓冲区的大小

char rxdata[RX_BUFFER_SIZE];  // 接收缓冲区
uint8_t rxdat;                // 当前接收到的字节
unsigned char rx_pointer = 0; // 接收缓冲区的当前索引
int rx_complete = 0;          // 接收完成标志

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     if (huart->Instance == USART1) // 确保是正确的UART实例触发了中断
     {
          if (rx_pointer < RX_BUFFER_SIZE - 1) // 检查是否有足够的缓冲区空间
          {
               rxdata[rx_pointer++] = rxdat; // 存储接收到的字节
               rxdata[rx_pointer] = '\0';    // 添加字符串结束符
          }
          else
          {
               rx_pointer = 0;  // 缓冲区已满，重置指针
               rx_complete = 1; // 设置接收完成标志
          }

          // 重新启动串口接收中断，准备接收下一个字节
          HAL_UART_Receive_IT(huart, &rxdat, 1);
     }
}

// 检测指定字符串位置的字符类型
int checkStringSegment(const char *str, CheckMode mode, size_t start, size_t length)
{
     if (str == NULL)
     {
          return 0; // 空指针，返回错误
     }

     const char *p = str + start;  // 指向要检查的起始位置
     const char *end = p + length; // 计算结束位置

     while (p < end && *p)
     {
          switch (mode)
          {
          case MODE_DIGITS:
               if (!isdigit((unsigned char)*p))
               {
                    return 0;
               }
               break;
          case MODE_ALPHA:
               if (!isalpha((unsigned char)*p))
               {
                    return 0;
               }
               break;
          case MODE_ALPHANUM:
               if (!isalnum((unsigned char)*p))
               {
                    return 0;
               }
               break;
          default:
               return 0; // 未知的检测模式
          }
          p++;
     }
     return 1; // 全部字符检查通过
}

// 函数用于检测年份是否为闰年
// 如果是闰年，则返回1；如果不是，则返回0。
// 年份需要为4位数
int isLeapYear(int year)
{
     // 闰年规则：
     // 1. 如果年份能被4整除，但不能被100整除，则是闰年。
     // 2. 如果年份能被400整除，则是闰年。
     if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
     {
          return 1;
     }
     else
     {
          return 0;
     }
}

// 从 `str` 的 `start_pos` 位置开始提取长度为 `len` 的子字符串，并将其存储在 `result` 中。
// 确保 `result` 有足够的空间，并在使用后以 '\0'（空字符）结束。
void extract_substring(const char *str, int start_pos, int len, char *result)
{
     int str_length = strlen(str);

     // 检查 `start_pos` 是否在字符串长度范围内
     if (start_pos < 0 || start_pos >= str_length)
     {
          // 起始位置无效时，返回空字符串
          *result = '\0';
     }
     else
     {
          // 调整长度，以防超出源字符串的末尾
          if (start_pos + len > str_length)
          {
               len = str_length - start_pos;
          }
          strncpy(result, str + start_pos, len);
          // 在字符串末尾添加空字符
          result[len] = '\0';
     }
}

// 计算两个时间点之间相差的秒数
// 将日期和时间转换为自1970年1月1日以来的秒数
long long convertToSeconds(int year, int month, int day, int hour, int minute, int second)
{
     struct tm timeStruct = {0};
     long long totalSeconds;

     timeStruct.tm_year = year - 1900; // tm_year是从1900年起的年数
     timeStruct.tm_mon = month - 1;    // tm_mon是从0开始的月数
     timeStruct.tm_mday = day;
     timeStruct.tm_hour = hour;
     timeStruct.tm_min = minute;
     timeStruct.tm_sec = second;
     timeStruct.tm_isdst = -1; // 自动判断是否为夏令时

     totalSeconds = mktime(&timeStruct);

     return totalSeconds;
}

// 计算两个时间点之间相差的秒数
long long timeDifference(int year1, int month1, int day1, int hour1, int minute1, int second1,
                         int year2, int month2, int day2, int hour2, int minute2, int second2)
{
     long long seconds1 = convertToSeconds(year1, month1, day1, hour1, minute1, second1);
     long long seconds2 = convertToSeconds(year2, month2, day2, hour2, minute2, second2);

     return seconds2 - seconds1;
}
