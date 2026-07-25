#include "TM1637.h"
#include "delay.h"
#include <stdbool.h>

/*
 * 7 段数码管编码 (0~9, A~F)
 * 段位排列:  DP-G-F-E-D-C-B-A
 *            bit7-bit6-bit5-bit4-bit3-bit2-bit1-bit0
 */
static const uint8_t digitToSegment[] = {
    /* 0 */ 0x3F, /* 1 */ 0x06, /* 2 */ 0x5B, /* 3 */ 0x4F,
    /* 4 */ 0x66, /* 5 */ 0x6D, /* 6 */ 0x7D, /* 7 */ 0x07,
    /* 8 */ 0x7F, /* 9 */ 0x6F, /* A */ 0x77, /* B */ 0x7C,
    /* C */ 0x39, /* D */ 0x5E, /* E */ 0x79, /* F */ 0x71
};

static uint8_t s_brightness = TM1637_BRIGHTNESS_MAX;

/* ---- 底层时序 ---- */

/**
 * @brief  产生 TM1637 通信起始信号
 *         时序: DIO 先拉低 → CLK 拉低
 */
static void TM1637_Start(void)
{
    TM1637_DIO_Clr();
    delay_us(2);
    TM1637_CLK_Clr();
    delay_us(2);
}

/**
 * @brief  产生 TM1637 通信停止信号
 *         时序: DIO 拉低 → CLK 拉高 → DIO 拉高
 */
static void TM1637_Stop(void)
{
    TM1637_DIO_Clr();
    delay_us(2);
    TM1637_CLK_Set();
    delay_us(2);
    TM1637_DIO_Set();
    delay_us(2);
}

/**
 * @brief  向 TM1637 写入一个字节（LSB first）并读取 ACK
 * @param  b 要写入的字节
 * @return true  ACK 接收成功（DIO 被从机拉低）
 *         false 无 ACK
 */
static bool TM1637_WriteByte(uint8_t b)
{
    uint8_t i;

    /* 发送 8 个数据位，LSB first */
    for (i = 0; i < 8; i++) {
        TM1637_CLK_Clr();
        delay_us(2);

        if (b & 0x01) {
            TM1637_DIO_Set();
        } else {
            TM1637_DIO_Clr();
        }
        delay_us(2);

        TM1637_CLK_Set();
        delay_us(2);

        b >>= 1;
    }

    /* 第 9 个时钟：读取 ACK（从机拉低 DIO 表示应答） */
    TM1637_CLK_Clr();
    delay_us(2);

    TM1637_DIO_IN();                        // DIO 切换为输入
    delay_us(2);

    TM1637_CLK_Set();
    delay_us(2);

    bool ack = (TM1637_DIO_Read() == 0);    // LOW → ACK

    TM1637_DIO_OUT();                       // DIO 恢复输出
    delay_us(2);

    TM1637_CLK_Clr();
    delay_us(2);

    return ack;
}

/* ---- 上层 API ---- */

void TM1637_Init(void)
{
    /* 初始化 CLK 和 DIO 引脚为推挽输出 */
    TM1637_CLK_INIT();
    TM1637_DIO_OUT();

    /* 初始状态：CLK 和 DIO 均为高电平（总线空闲） */
    TM1637_CLK_Set();
    TM1637_DIO_Set();

    s_brightness = TM1637_BRIGHTNESS_MAX;
}

void TM1637_SetBrightness(uint8_t brightness)
{
    s_brightness = brightness & 0x07;
}

void TM1637_SetSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    uint8_t i;

    /* 设置数据写入模式（自动地址递增） */
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_Stop();

    /* 设置起始地址并发送段码数据 */
    TM1637_Start();
    TM1637_WriteByte(0xC0 | pos);

    for (i = 0; i < length; i++) {
        TM1637_WriteByte(segments[i]);
    }

    TM1637_Stop();

    /* 设置亮度（显示开） */
    TM1637_Start();
    TM1637_WriteByte(0x88 | s_brightness);
    TM1637_Stop();
}

void TM1637_Clear(void)
{
    const uint8_t blank[] = {0x00, 0x00, 0x00, 0x00};
    TM1637_SetSegments(blank, 4, 0);
}

uint8_t TM1637_EncodeDigit(uint8_t digit)
{
    return digitToSegment[digit & 0x0F];
}

uint8_t TM1637_EncodeDigitWithDot(uint8_t digit)
{
    return digitToSegment[digit & 0x0F] | 0x80;
}

void TM1637_ShowNumber(uint16_t num)
{
    uint8_t segments[4];
    uint8_t i;

    /* 分解为 4 个数字，高位不足补 0 */
    for (i = 0; i < 4; i++) {
        segments[3 - i] = TM1637_EncodeDigit(num % 10);
        num /= 10;
    }

    TM1637_SetSegments(segments, 4, 0);
}

void TM1637_ShowNumberDot(uint16_t num, uint8_t dotPos)
{
    uint8_t segments[4];
    uint8_t i;

    for (i = 0; i < 4; i++) {
        segments[3 - i] = TM1637_EncodeDigit(num % 10);
        num /= 10;
    }

    /* 在指定位置加上小数点 */
    if (dotPos < 4) {
        segments[dotPos] |= 0x80;
    }

    TM1637_SetSegments(segments, 4, 0);
}
