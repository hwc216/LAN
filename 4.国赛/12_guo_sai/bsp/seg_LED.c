#include "stm32g4xx.h" // Device header
#include "seg_LED.h"
#include "stdint.h"

const uint8_t Seg7[17] = {
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8
    0x6f, // 9
    0x77, // A
    0x7c, // b
    0x39, // C
    0x5e, // d
    0x79, // E
    0x71, // F
    0x00  // Clear
};

// 辅助函数，用于发送单个数字数据到七段显示器
void SendToDisplay(uint8_t segData)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (segData & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        segData <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    }
}

// 显示函数，现在包括小数点控制
void SEG_Display(uint8_t SET1, uint8_t SET2, uint8_t SET3, uint8_t DP1, uint8_t DP2, uint8_t DP3)
{
    // 将小数点的控制合并到显示值中
    uint8_t segData1 = Seg7[SET1] | (DP1 ? 0x80 : 0x00);
    uint8_t segData2 = Seg7[SET2] | (DP2 ? 0x80 : 0x00);
    uint8_t segData3 = Seg7[SET3] | (DP3 ? 0x80 : 0x00);

    // 发送数据到七段显示器
    SendToDisplay(segData3); // 第三位数
    SendToDisplay(segData2); // 第二位数
    SendToDisplay(segData1); // 第一位数

    // 选中特定的位并激活显示
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}
