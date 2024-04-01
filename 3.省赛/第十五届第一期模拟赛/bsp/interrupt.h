#ifndef _interrupt_h
#define _interrupt_h

#include "main.h"

typedef enum
{
    MODE_DIGITS,  // 检测字符串是否全是数字
    MODE_ALPHA,   // 检测字符串是否全是字母
    MODE_ALPHANUM // 检测字符串是否只包含字母和数字
} CheckMode;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

int checkStringSegment(const char *str, CheckMode mode, size_t start, size_t length);

int isLeapYear(int year);

void extract_substring(const char *str, int start_pos, int len, char *result);

long long timeDifference(int year1, int month1, int day1, int hour1, int minute1, int second1,
                         int year2, int month2, int day2, int hour2, int minute2, int second2);

#endif
