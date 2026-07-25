#ifndef __TM1637_H__
#define __TM1637_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * TM1637 四位数码管驱动
 *
 * 使用时需要在 SysConfig 中将对应 GPIO 引脚配置为输出（默认高电平），
 * 或者参考本文件的初始化宏手动初始化。
 *
 * 引脚定义（根据实际接线修改）：
 *   TM1637_CLK_PIN     - 时钟线
 *   TM1637_DIO_PIN     - 数据线
 *   TM1637_CLK_IOMUX   - CLK 引脚对应的 IOMUX_PINCMx
 *   TM1637_DIO_IOMUX   - DIO 引脚对应的 IOMUX_PINCMx
 *   TM1637_PORT        - GPIO 端口 (GPIOA / GPIOB / ...)
 */

// ============ 用户配置：根据实际接线修改 ============
#define TM1637_PORT             GPIOA
#define TM1637_CLK_PIN          DL_GPIO_PIN_8   // PA8
#define TM1637_CLK_IOMUX        IOMUX_PINCM9    // PA8 对应的 IOMUX (按实际修改)
#define TM1637_DIO_PIN          DL_GPIO_PIN_9   // PA9
#define TM1637_DIO_IOMUX        IOMUX_PINCM10   // PA9 对应的 IOMUX (按实际修改)
// ==================================================

// --- GPIO 操作宏 ---
#define TM1637_CLK_Set()        DL_GPIO_setPins(TM1637_PORT, TM1637_CLK_PIN)
#define TM1637_CLK_Clr()        DL_GPIO_clearPins(TM1637_PORT, TM1637_CLK_PIN)
#define TM1637_DIO_Set()        DL_GPIO_setPins(TM1637_PORT, TM1637_DIO_PIN)
#define TM1637_DIO_Clr()        DL_GPIO_clearPins(TM1637_PORT, TM1637_DIO_PIN)
#define TM1637_DIO_Read()       ((DL_GPIO_readPins(TM1637_PORT, TM1637_DIO_PIN) & TM1637_DIO_PIN) ? 1 : 0)

// DIO 输出模式
#define TM1637_DIO_OUT()        { \
    DL_GPIO_initDigitalOutputFeatures(TM1637_DIO_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(TM1637_PORT, TM1637_DIO_PIN); \
}

// DIO 输入模式（用于接收 ACK）
#define TM1637_DIO_IN()         { \
    DL_GPIO_disableOutput(TM1637_PORT, TM1637_DIO_PIN); \
    DL_GPIO_initDigitalInputFeatures(TM1637_DIO_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE); \
}

// CLK 输出初始化
#define TM1637_CLK_INIT()       { \
    DL_GPIO_initDigitalOutputFeatures(TM1637_CLK_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(TM1637_PORT, TM1637_CLK_PIN); \
}

// --- 亮度等级 ---
#define TM1637_BRIGHTNESS_MIN   0
#define TM1637_BRIGHTNESS_MAX   7

// --- 函数声明 ---

/**
 * @brief  初始化 TM1637（设置 CLK/DIO 引脚为输出，初始高电平）
 */
void TM1637_Init(void);

/**
 * @brief  设置显示亮度
 * @param brightness 亮度 0~7（0 最暗，7 最亮）
 */
void TM1637_SetBrightness(uint8_t brightness);

/**
 * @brief  将段码数据发送到 TM1637
 * @param segments   段码数组（每个元素对应一位数码管的段码）
 * @param length     段码数量（最大 4）
 * @param pos        起始位置（0~3，0 为最左位）
 */
void TM1637_SetSegments(const uint8_t segments[], uint8_t length, uint8_t pos);

/**
 * @brief  清屏（全部熄灭）
 */
void TM1637_Clear(void);

/**
 * @brief  将数字 0~15 编码为 7 段码（不含小数点）
 * @param digit 数字 (0~15, 10~15 对应 A~F)
 * @return 7 段码
 */
uint8_t TM1637_EncodeDigit(uint8_t digit);

/**
 * @brief  将数字 0~15 编码为 7 段码（带小数点）
 * @param digit 数字 (0~15)
 * @return 7 段码（最高位为小数点）
 */
uint8_t TM1637_EncodeDigitWithDot(uint8_t digit);

/**
 * @brief  直接显示一个四位数（不含小数点）
 * @param num 要显示的数字 (0~9999)
 */
void TM1637_ShowNumber(uint16_t num);

/**
 * @brief  直接显示一个四位数（带小数点位置控制）
 * @param num         要显示的数字 (0~9999)
 * @param dotPos      小数点位置 (0~3, 0xFF 表示无小数点)
 */
void TM1637_ShowNumberDot(uint16_t num, uint8_t dotPos);

#endif /* __TM1637_H__ */
