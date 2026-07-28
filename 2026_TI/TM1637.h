#ifndef TM1637_H
#define TM1637_H

#include "board_config.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define TM1637_CLK_Set() \
    DL_GPIO_setPins(TM1637_PORT, TM1637_CLK_PIN)
#define TM1637_CLK_Clr() \
    DL_GPIO_clearPins(TM1637_PORT, TM1637_CLK_PIN)
#define TM1637_DIO_Set() \
    DL_GPIO_setPins(TM1637_PORT, TM1637_DIO_PIN)
#define TM1637_DIO_Clr() \
    DL_GPIO_clearPins(TM1637_PORT, TM1637_DIO_PIN)
#define TM1637_DIO_Read() \
    ((DL_GPIO_readPins(TM1637_PORT, TM1637_DIO_PIN) & TM1637_DIO_PIN) ? 1 : 0)

#define TM1637_DIO_OUT() do { \
    DL_GPIO_initDigitalOutputFeatures(TM1637_DIO_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(TM1637_PORT, TM1637_DIO_PIN); \
} while (0)

#define TM1637_DIO_IN() do { \
    DL_GPIO_disableOutput(TM1637_PORT, TM1637_DIO_PIN); \
    DL_GPIO_initDigitalInputFeatures(TM1637_DIO_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE); \
} while (0)

#define TM1637_CLK_INIT() do { \
    DL_GPIO_initDigitalOutputFeatures(TM1637_CLK_IOMUX, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, \
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE); \
    DL_GPIO_enableOutput(TM1637_PORT, TM1637_CLK_PIN); \
} while (0)

#define TM1637_BRIGHTNESS_MIN   0
#define TM1637_BRIGHTNESS_MAX   7

void TM1637_Init(void);
void TM1637_SetBrightness(uint8_t brightness);
void TM1637_SetSegments(const uint8_t segments[], uint8_t length, uint8_t pos);
void TM1637_Clear(void);
uint8_t TM1637_EncodeDigit(uint8_t digit);
uint8_t TM1637_EncodeDigitWithDot(uint8_t digit);
void TM1637_ShowNumber(uint16_t num);
void TM1637_ShowNumberDot(uint16_t num, uint8_t dotPos);

#endif /* TM1637_H */
