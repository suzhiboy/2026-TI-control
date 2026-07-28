/**
 ****************************************************************************************************
 * @file        demo_lora.c
 * @author      TI_2026_Group_D
 * @version     V2.0
 * @date        2026-07-28
 * @brief       PB22 GPIO 输出控制示例 (原 LORA 演示, 现改为 PB22 高低电平控制)
 * @note        硬件平台: MSPM0G3507
 *
 *              功能说明:
 *              - 初始化 PB22 为推挽输出
 *              - 主循环中交替控制 PB22 高低电平 (500ms 间隔)
 *              - PB22 引脚对应 IOMUX_PINCM50
 ****************************************************************************************************
 */

#include "ti_msp_dl_config.h"
#include "delay.h"

/*===========================================================================
 * PB22 引脚定义
 *
 * MSPM0G3507 LQFP-64 封装中:
 *   PB22 -> IOMUX_PINCM50
 *===========================================================================*/
#define DEMO_PB22_PORT          GPIOB
#define DEMO_PB22_PIN           DL_GPIO_PIN_22
#define DEMO_PB22_IOMUX         IOMUX_PINCM50

/*===========================================================================
 * 引脚操作宏
 *===========================================================================*/
#define PB22_Set()              DL_GPIO_setPins(DEMO_PB22_PORT, DEMO_PB22_PIN)
#define PB22_Clr()              DL_GPIO_clearPins(DEMO_PB22_PORT, DEMO_PB22_PIN)
#define PB22_Toggle()           DL_GPIO_togglePins(DEMO_PB22_PORT, DEMO_PB22_PIN)

/**
 * @brief   PB22 控制演示 (取代原 LORA 演示)
 * @param   无
 * @retval  无
 */
void demo_lora_run(void)
{
    /* 初始化 PB22 为推挽输出, 初始低电平 */
    DL_GPIO_initDigitalOutputFeatures(DEMO_PB22_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(DEMO_PB22_PORT, DEMO_PB22_PIN);
    PB22_Clr();

    /* 主循环: 500ms 交替高低电平 */
    while (1)
    {
        PB22_Set();             /* PB22 输出高电平 */
        delay_ms(500);

        PB22_Clr();             /* PB22 输出低电平 */
        delay_ms(500);
    }
}