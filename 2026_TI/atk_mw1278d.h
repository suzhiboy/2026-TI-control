/**
 ****************************************************************************************************
 * @file        atk_mw1278d.h
 * @author      TI_2026_Group_D
 * @version     V1.0
 * @date        2026-07-24
 * @brief       ATK-LORA-01 (ATK-MW1278D) 模块底层驱动
 * @note        适用于 MSPM0G3507 + TI DriverLib (SysConfig)
 *              模块型号: 正点原子 ATK-LORA-01 (核心芯片: MW1278D)
 *
 *              硬件接线说明:
 *              ATK-LORA-01    MSPM0G3507
 *              VCC            5V
 *              GND            GND
 *              TXD            PA9  (UART2 RX)
 *              RXD            PA8  (UART2 TX)
 *              MD0            PB0  (GPIO 输出)
 *              AUX            PB1  (GPIO 输入)
 *
 *              工作模式:
 *              - MD0=1: 配置模式 (AT 指令模式)
 *              - MD0=0: 透传模式 (数据透明传输)
 *              - AUX=0: 模块空闲
 *              - AUX=1: 模块忙碌
 ****************************************************************************************************
 */

#ifndef __ATK_MW1278D_H
#define __ATK_MW1278D_H

#include "ti_msp_dl_config.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 引脚定义 (根据实际接线修改)
 *===========================================================================*/

/* --- MD0 引脚 (模式选择) --- */
#define ATK_MW1278D_MD0_PORT           GPIOB
#define ATK_MW1278D_MD0_PIN            DL_GPIO_PIN_0       /* PB0 */
#define ATK_MW1278D_MD0_IOMUX          IOMUX_PINCM16       /* PB0 对应的 IOMUX */

/* --- AUX 引脚 (模块状态指示) --- */
#define ATK_MW1278D_AUX_PORT           GPIOB
#define ATK_MW1278D_AUX_PIN            DL_GPIO_PIN_1       /* PB1 */
#define ATK_MW1278D_AUX_IOMUX          IOMUX_PINCM17       /* PB1 对应的 IOMUX */

/* --- MD0 操作宏 --- */
#define ATK_MW1278D_MD0_Set()          DL_GPIO_setPins(ATK_MW1278D_MD0_PORT, ATK_MW1278D_MD0_PIN)
#define ATK_MW1278D_MD0_Clr()          DL_GPIO_clearPins(ATK_MW1278D_MD0_PORT, ATK_MW1278D_MD0_PIN)

/* --- AUX 读取宏 --- */
#define ATK_MW1278D_AUX_Read()         ((DL_GPIO_readPins(ATK_MW1278D_AUX_PORT, ATK_MW1278D_AUX_PIN) & ATK_MW1278D_AUX_PIN) ? 1 : 0)

/*===========================================================================
 * UART 配置 (通过 SysConfig 配置 UART2)
 *===========================================================================*/
#define ATK_MW1278D_UART_INST          UART_2_INST         /* 使用的 UART 外设实例 */
#define ATK_MW1278D_UART_INT_IRQN      UART_2_INST_INT_IRQN

/* UART 接收缓冲区大小 */
#define ATK_MW1278D_UART_RX_BUF_SIZE   256
#define ATK_MW1278D_UART_TX_BUF_SIZE   256

/*===========================================================================
 * 定时器配置 (用于接收帧超时检测, 通过 SysConfig 配置)
 *===========================================================================*/
#define ATK_MW1278D_TIMER_INST         TIMER_0_INST        /* 使用的定时器外设实例 */

/*===========================================================================
 * AT 指令超时 (毫秒)
 *===========================================================================*/
#define ATK_MW1278D_AT_TIMEOUT         500

/*===========================================================================
 * 使能状态枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_DISABLE             = 0x00,
    ATK_MW1278D_ENABLE,
} atk_mw1278d_enable_t;

/*===========================================================================
 * 发射功率枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_TPOWER_11DBM        = 0,   /* 11dBm */
    ATK_MW1278D_TPOWER_14DBM        = 1,   /* 14dBm */
    ATK_MW1278D_TPOWER_17DBM        = 2,   /* 17dBm */
    ATK_MW1278D_TPOWER_20DBM        = 3,   /* 20dBm (默认) */
} atk_mw1278d_tpower_t;

/*===========================================================================
 * 工作模式枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_WORKMODE_NORMAL     = 0,   /* 一般模式 (默认) */
    ATK_MW1278D_WORKMODE_WAKEUP     = 1,   /* 唤醒模式 */
    ATK_MW1278D_WORKMODE_LOWPOWER   = 2,   /* 省电模式 */
    ATK_MW1278D_WORKMODE_SIGNAL     = 3,   /* 信号强度模式 */
} atk_mw1278d_workmode_t;

/*===========================================================================
 * 传输模式枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_TMODE_TT            = 0,   /* 透传传输 (默认) */
    ATK_MW1278D_TMODE_DT            = 1,   /* 定点传输 */
} atk_mw1278d_tmode_t;

/*===========================================================================
 * 无线速率枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_WLRATE_0K3          = 0,   /* 0.3Kbps (定点模式专用) */
    ATK_MW1278D_WLRATE_1K2          = 1,   /* 1.2Kbps */
    ATK_MW1278D_WLRATE_2K4          = 2,   /* 2.4Kbps */
    ATK_MW1278D_WLRATE_4K8          = 3,   /* 4.8Kbps */
    ATK_MW1278D_WLRATE_9K6          = 4,   /* 9.6Kbps */
    ATK_MW1278D_WLRATE_19K2         = 5,   /* 19.2Kbps (默认) */
} atk_mw1278d_wlrate_t;

/*===========================================================================
 * 唤醒时间枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_WLTIME_1S           = 0,   /* 1秒 (默认) */
    ATK_MW1278D_WLTIME_2S           = 1,   /* 2秒 */
} atk_mw1278d_wltime_t;

/*===========================================================================
 * 串口通信波特率枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_UARTRATE_1200BPS    = 0,   /* 1200bps */
    ATK_MW1278D_UARTRATE_2400BPS    = 1,   /* 2400bps */
    ATK_MW1278D_UARTRATE_4800BPS    = 2,   /* 4800bps */
    ATK_MW1278D_UARTRATE_9600BPS    = 3,   /* 9600bps */
    ATK_MW1278D_UARTRATE_19200BPS   = 4,   /* 19200bps */
    ATK_MW1278D_UARTRATE_38400BPS   = 5,   /* 38400bps */
    ATK_MW1278D_UARTRATE_57600BPS   = 6,   /* 57600bps */
    ATK_MW1278D_UARTRATE_115200BPS  = 7,   /* 115200bps (默认) */
} atk_mw1278d_uartrate_t;

/*===========================================================================
 * 串口校验位枚举
 *===========================================================================*/
typedef enum
{
    ATK_MW1278D_UARTPARI_NONE       = 0,   /* 无校验 (默认) */
    ATK_MW1278D_UARTPARI_EVEN       = 1,   /* 偶校验 */
    ATK_MW1278D_UARTPARI_ODD        = 2,   /* 奇校验 */
} atk_mw1278d_uartpari_t;

/*===========================================================================
 * 错误码定义
 *===========================================================================*/
#define ATK_MW1278D_EOK             0       /* 没有错误 */
#define ATK_MW1278D_ERROR           1       /* 通用错误 */
#define ATK_MW1278D_ETIMEOUT        2       /* 超时错误 */
#define ATK_MW1278D_EINVAL          3       /* 无效参数 */
#define ATK_MW1278D_EBUSY           4       /* 模块忙碌 */

/*===========================================================================
 * 函数声明 - 初始化和基础操作
 *===========================================================================*/

/**
 * @brief   ATK-MW1278D 模块初始化
 * @param   baudrate: UART 通信波特率 (如 115200)
 * @retval  ATK_MW1278D_EOK   - 初始化成功
 *          ATK_MW1278D_ERROR - 初始化失败
 */
uint8_t atk_mw1278d_init(uint32_t baudrate);

/**
 * @brief   进入配置模式 (MD0 拉高, 可发送 AT 指令)
 * @param   无
 * @retval  无
 */
void atk_mw1278d_enter_config(void);

/**
 * @brief   退出配置模式 (MD0 拉低, 进入透传模式)
 * @param   无
 * @retval  无
 */
void atk_mw1278d_exit_config(void);

/**
 * @brief   判断模块是否空闲 (AUX=0 空闲, AUX=1 忙碌)
 * @param   无
 * @retval  ATK_MW1278D_EOK  - 模块空闲
 *          ATK_MW1278D_EBUSY - 模块忙碌
 */
uint8_t atk_mw1278d_free(void);

/*===========================================================================
 * 函数声明 - AT 指令操作
 *===========================================================================*/

/**
 * @brief   发送 AT 指令并等待应答
 * @param   cmd    : 要发送的 AT 指令字符串
 *          ack    : 期望的应答关键字 (如 "OK"), 为 NULL 则不等待
 *          timeout: 等待超时时间 (毫秒)
 * @retval  ATK_MW1278D_EOK     - 收到期望应答
 *          ATK_MW1278D_ETIMEOUT - 超时未收到应答
 */
uint8_t atk_mw1278d_send_at_cmd(char *cmd, char *ack, uint32_t timeout);

/**
 * @brief   AT 指令通讯测试 (发送 "AT" 等待 "OK")
 * @param   无
 * @retval  ATK_MW1278D_EOK   - 通讯正常
 *          ATK_MW1278D_ERROR - 通讯失败
 */
uint8_t atk_mw1278d_at_test(void);

/**
 * @brief   配置指令回显 (ATE0: 关闭回显, ATE1: 开启回显)
 * @param   enable: ATK_MW1278D_DISABLE/ATK_MW1278D_ENABLE
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_echo_config(atk_mw1278d_enable_t enable);

/**
 * @brief   软件复位模块
 * @param   无
 * @retval  ATK_MW1278D_EOK   - 复位成功
 *          ATK_MW1278D_ERROR - 复位失败
 */
uint8_t atk_mw1278d_sw_reset(void);

/**
 * @brief   配置 Flash 保存 (AT+FLASH=0: 不保存, AT+FLASH=1: 保存)
 * @param   enable: ATK_MW1278D_DISABLE/ATK_MW1278D_ENABLE
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_flash_config(atk_mw1278d_enable_t enable);

/**
 * @brief   恢复出厂设置
 * @param   无
 * @retval  ATK_MW1278D_EOK   - 恢复成功
 *          ATK_MW1278D_ERROR - 恢复失败
 */
uint8_t atk_mw1278d_default(void);

/*===========================================================================
 * 函数声明 - 模块参数配置
 *===========================================================================*/

/**
 * @brief   配置设备地址
 * @param   addr: 设备地址 (0x0000 ~ 0xFFFF)
 * @retval  ATK_MW1278D_EOK   - 配置成功
 *          ATK_MW1278D_ERROR - 配置失败
 */
uint8_t atk_mw1278d_addr_config(uint16_t addr);

/**
 * @brief   配置发射功率
 * @param   tpower: 发射功率等级
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_tpower_config(atk_mw1278d_tpower_t tpower);

/**
 * @brief   配置工作模式
 * @param   workmode: 工作模式
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_workmode_config(atk_mw1278d_workmode_t workmode);

/**
 * @brief   配置传输模式
 * @param   tmode: 传输模式 (透传/定点)
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_tmode_config(atk_mw1278d_tmode_t tmode);

/**
 * @brief   配置无线速率和信道
 * @param   wlrate : 无线速率
 *          channel: 信道 (0 ~ 83)
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_wlrate_channel_config(atk_mw1278d_wlrate_t wlrate, uint8_t channel);

/**
 * @brief   配置唤醒时间
 * @param   wltime: 唤醒时间
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_wltime_config(atk_mw1278d_wltime_t wltime);

/**
 * @brief   配置模块串口参数 (修改后模块需要重新初始化 MCU 端 UART)
 * @param   baudrate: 波特率
 *          parity  : 校验位
 * @retval  ATK_MW1278D_EOK    - 配置成功
 *          ATK_MW1278D_ERROR  - 配置失败
 *          ATK_MW1278D_EINVAL - 参数无效
 */
uint8_t atk_mw1278d_uart_config(atk_mw1278d_uartrate_t baudrate, atk_mw1278d_uartpari_t parity);

/*===========================================================================
 * 函数声明 - 数据传输
 *===========================================================================*/

/**
 * @brief   通过 LORA 模块发送数据 (UART printf 格式)
 * @param   fmt: 格式化字符串
 *          ...: 可变参数
 * @retval  无
 * @note    该函数在透传模式下使用, 直接通过模块发送无线数据
 */
void atk_mw1278d_uart_printf(char *fmt, ...);

/**
 * @brief   重新开始接收一帧数据
 * @param   无
 * @retval  无
 */
void atk_mw1278d_uart_rx_restart(void);

/**
 * @brief   获取接收到的数据帧
 * @param   无
 * @retval  NULL: 未接收到完整帧
 *          其他: 指向接收缓冲区的指针
 */
uint8_t *atk_mw1278d_uart_rx_get_frame(void);

/**
 * @brief   获取接收到的数据帧长度
 * @param   无
 * @retval  0   : 未接收到完整帧
 *          其他: 帧长度
 */
uint16_t atk_mw1278d_uart_rx_get_frame_len(void);

/**
 * @brief   UART 中断服务函数 (需要在 UART ISR 中调用)
 * @param   无
 * @retval  无
 */
void ATK_MW1278D_UART_IRQHandler(void);

/**
 * @brief   定时器中断服务函数 (需要在 Timer ISR 中调用)
 * @param   无
 * @retval  无
 */
void ATK_MW1278D_TIM_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __ATK_MW1278D_H */
