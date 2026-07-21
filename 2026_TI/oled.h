#ifndef __OLED_H
#define __OLED_H			  	 

#include "ti_msp_dl_config.h"
#include <stdint.h>

// --- 引脚定义 (PA0/PA1) ---
#define OLED_PORT           GPIOA
#define OLED_SCL_PIN        DL_GPIO_PIN_1   // PA1
#define OLED_SCL_IOMUX      IOMUX_PINCM2    // PA1 对应的 IOMUX
#define OLED_SDA_PIN        DL_GPIO_PIN_0   // PA0
#define OLED_SDA_IOMUX      IOMUX_PINCM1    // PA0 对应的 IOMUX

// --- I2C 模拟操作宏 ---
#define OLED_SCL_Set()      DL_GPIO_setPins(OLED_PORT, OLED_SCL_PIN)
#define OLED_SCL_Clr()      DL_GPIO_clearPins(OLED_PORT, OLED_SCL_PIN)
#define OLED_SDA_Set()      DL_GPIO_setPins(OLED_PORT, OLED_SDA_PIN)
#define OLED_SDA_Clr()      DL_GPIO_clearPins(OLED_PORT, OLED_SDA_PIN)

#define OLED_READ_SDA()     ((DL_GPIO_readPins(OLED_PORT, OLED_SDA_PIN) & OLED_SDA_PIN) ? 1 : 0)

// SDA 输出模式：开启高驱动强度和上拉
#define OLED_SDA_OUT()      { \
    DL_GPIO_initDigitalOutputFeatures(OLED_SDA_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(OLED_PORT, OLED_SDA_PIN); \
}

// SDA 输入模式：开启上拉
#define OLED_SDA_IN()       { \
    DL_GPIO_disableOutput(OLED_PORT, OLED_SDA_PIN); \
    DL_GPIO_initDigitalInputFeatures(OLED_SDA_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, \
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE); \
}

// SCL 同样使用高驱动强度
#define OLED_SCL_INIT()     { \
    DL_GPIO_initDigitalOutputFeatures(OLED_SCL_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(OLED_PORT, OLED_SCL_PIN); \
}

#define OLED_CMD  0
#define OLED_DATA 1

// 函数声明
void OLED_Init(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
void OLED_ShowFloatNum(uint8_t x, uint8_t y, double num, uint8_t intlen, uint8_t fralen, uint8_t size1, uint8_t mode);
void OLED_Update(void);
void OLED_Show_String(uint8_t han, uint8_t lie, uint8_t *string);

#endif
