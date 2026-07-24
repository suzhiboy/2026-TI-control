/**
 ****************************************************************************************************
 * @file        demo_lora.c
 * @author      TI_2026_Group_D
 * @version     V1.0
 * @date        2026-07-24
 * @brief       ATK-LORA-01 模块使用示例
 * @note        硬件平台: MSPM0G3507
 *
 *              功能说明:
 *              - 初始化 LORA 模块
 *              - 配置模块参数 (地址/信道/功率/速率等)
 *              - 按键发送数据
 *              - 接收并打印数据
 *
 *              SysConfig 配置步骤:
 *              1. 添加 UART (UART2):
 *                 - TX: PA8, RX: PA9
 *                 - Baud Rate: 115200
 *                 - Data Bits: 8
 *                 - Stop Bits: 1
 *                 - Parity: None
 *                 - 勾选 "Enable RX Interrupt"
 *                 - 重命名实例为 "UART_2"
 *
 *              2. 添加 Timer (TIMER_0):
 *                 - Counter Mode: Down
 *                 - Period: 31999 (32MHz/32000 = 1ms)
 *                 - 勾选 "Enable Zero Event Interrupt"
 *                 - 重命名实例为 "TIMER_0"
 *
 *              3. 添加 GPIO 引脚:
 *                 - PB0: Output (MD0)
 *                 - PB1: Input, Pull-up (AUX)
 ****************************************************************************************************
 */

#include "atk_mw1278d.h"
#include "oled.h"
#include "delay.h"
#include <stdio.h>

/*===========================================================================
 * LORA 模块默认配置参数 (可根据需求修改)
 *===========================================================================*/
#define DEMO_ADDR       0                               /* 设备地址 */
#define DEMO_WLRATE     ATK_MW1278D_WLRATE_19K2         /* 无线速率 */
#define DEMO_CHANNEL    0                               /* 信道 (0~83) */
#define DEMO_TPOWER     ATK_MW1278D_TPOWER_20DBM        /* 发射功率 */
#define DEMO_WORKMODE   ATK_MW1278D_WORKMODE_NORMAL     /* 工作模式 */
#define DEMO_TMODE      ATK_MW1278D_TMODE_TT            /* 传输模式 (透传) */
#define DEMO_WLTIME     ATK_MW1278D_WLTIME_1S           /* 唤醒时间 */
#define DEMO_UARTRATE   ATK_MW1278D_UARTRATE_115200BPS  /* UART 波特率 */
#define DEMO_UARTPARI   ATK_MW1278D_UARTPARI_NONE       /* UART 校验位 */

/*===========================================================================
 * 中断向量挂接 (在相应的中断向量函数中调用)
 *
 * 将以下函数放到 ti_msp_dl_config.c 生成的中断处理函数中:
 *
 * void UART_2_INST_IRQHandler(void)
 * {
 *     ATK_MW1278D_UART_IRQHandler();
 * }
 *
 * void TIMER_0_INST_IRQHandler(void)
 * {
 *     ATK_MW1278D_TIM_IRQHandler();
 * }
 *===========================================================================*/

/**
 * @brief   LORA 模块演示函数
 * @param   无
 * @retval  无
 */
void demo_lora_run(void)
{
    uint8_t ret;
    uint8_t *buf;

    OLED_Show_String(1, 1, "LORA Init...");

    /* 初始化 LORA 模块 (默认 115200 波特率) */
    ret = atk_mw1278d_init(115200);
    if (ret != ATK_MW1278D_EOK)
    {
        OLED_Show_String(2, 1, "LORA Init FAIL!");
        printf("[LORA] Init failed!\r\n");
        while (1)
        {
            delay_ms(500);
        }
    }
    OLED_Show_String(2, 1, "LORA Init OK  ");

    /* 进入配置模式, 配置 LORA 模块参数 */
    atk_mw1278d_enter_config();

    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);

    atk_mw1278d_exit_config();

    if (ret != 0)
    {
        OLED_Show_String(3, 1, "LORA Cfg FAIL!");
        printf("[LORA] Config failed!\r\n");
        while (1)
        {
            delay_ms(500);
        }
    }

    OLED_Show_String(3, 1, "LORA Cfg OK  ");
    printf("[LORA] Config success!\r\n");

    /* 清空接收缓冲区 */
    atk_mw1278d_uart_rx_restart();

    /* 主循环 */
    while (1)
    {
        /* 接收无线数据 */
        buf = atk_mw1278d_uart_rx_get_frame();
        if (buf != NULL)
        {
            printf("[LORA RX]: %s\r\n", buf);
            OLED_Show_String(4, 1, "                ");  /* 清除指定行 */
            OLED_Show_String(4, 1, buf);

            /* 重新开始接收下一帧 */
            atk_mw1278d_uart_rx_restart();
        }

        delay_ms(10);
    }
}

/**
 * @brief   发送 LORA 数据
 * @param   data: 要发送的字符串
 * @retval  无
 * @note    调用前确保模块不在配置模式 (MD0=0)
 */
void lora_send_string(char *data)
{
    /* 等待模块空闲 */
    if (atk_mw1278d_free() != ATK_MW1278D_EBUSY)
    {
        atk_mw1278d_uart_printf("%s", data);
        printf("[LORA TX]: %s\r\n", data);
    }
    else
    {
        printf("[LORA] Module busy!\r\n");
    }
}

/**
 * @brief   发送 LORA 数据 (带目标地址, 定点传输模式)
 * @param   addr: 目标设备地址 (2字节)
 *          data: 要发送的数据
 * @retval  无
 * @note    需要在定点传输模式下使用 (ATK_MW1278D_TMODE_DT)
 */
void lora_send_to_addr(uint16_t addr, char *data)
{
    if (atk_mw1278d_free() != ATK_MW1278D_EBUSY)
    {
        /* 定点传输格式: 目标地址(2字节) + 数据 */
        uint8_t addr_h = (uint8_t)(addr >> 8) & 0xFF;
        uint8_t addr_l = (uint8_t)(addr & 0xFF);

        DL_UART_transmitDataBlocking(ATK_MW1278D_UART_INST, &addr_h, 1);
        DL_UART_transmitDataBlocking(ATK_MW1278D_UART_INST, &addr_l, 1);
        DL_UART_transmitDataBlocking(ATK_MW1278D_UART_INST,
                                     (uint8_t *)data, strlen(data));

        printf("[LORA TX -> 0x%04X]: %s\r\n", addr, data);
    }
    else
    {
        printf("[LORA] Module busy!\r\n");
    }
}
