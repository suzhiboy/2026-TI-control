/**
 ****************************************************************************************************
 * @file        atk_lora_01.c
 * @author      TI_2026_Group_D
 * @version     V2.0
 * @date        2026-07-28
 * @brief       ATK-LORA-01 模块底层驱动实现
 * @note        适用于 MSPM0G3507 + TI DriverLib (SysConfig)
 *
 *              模块操作流程:
 *              1. 初始化: atk_lora_01_init(baudrate)
 *              2. 进入配置模式: atk_lora_01_enter_config()
 *              3. 发送 AT 指令配置参数
 *              4. 退出配置模式: atk_lora_01_exit_config()
 *              5. 透传模式收发数据
 *
 *              帧接收机制:
 *              - 使用 UART 中断逐字节接收数据
 *              - 使用定时器检测帧间超时 (数据流中超过一定时间无新字节则认为一帧结束)
 *              - 定时器超时时间 = 约 100ms (定时器周期 10ms, 计数值 > 10)
 ****************************************************************************************************
 */

#include "atk_lora_01.h"

/*===========================================================================
 * 静态变量
 *===========================================================================*/

/* UART 发送缓冲区 */
static uint8_t g_uart_tx_buf[ATK_LORA_01_UART_TX_BUF_SIZE];

/* UART 接收帧信息结构体 */
static struct
{
    uint8_t buf[ATK_LORA_01_UART_RX_BUF_SIZE];          /* 帧数据缓冲区 */
    struct
    {
        uint16_t len    : 15;                           /* 帧接收长度, sta[14:0] */
        uint16_t finsh  : 1;                            /* 帧接收完成标志, sta[15] */
    } sta;                                              /* 帧状态信息 */
} g_uart_rx_frame = {0};

/* 定时器计数值 (用于帧超时检测) */
static volatile uint32_t g_timer_tick = 0;

/*===========================================================================
 * 硬件引脚初始化 (静态函数)
 *===========================================================================*/

/**
 * @brief   初始化 MD0 和 AUX 引脚
 * @param   无
 * @retval  无
 */
static void atk_lora_01_hw_init(void)
{
    /* MD0 引脚初始化 - 推挽输出, 初始低电平 (透传模式) */
    DL_GPIO_initDigitalOutputFeatures(ATK_LORA_01_MD0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(ATK_LORA_01_MD0_PORT, ATK_LORA_01_MD0_PIN);
    ATK_LORA_01_MD0_Clr();  /* 默认进入透传模式 */

    /* AUX 引脚初始化 - 输入模式, 上拉 */
    DL_GPIO_initDigitalInputFeatures(ATK_LORA_01_AUX_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/*===========================================================================
 * UART 通信底层
 *===========================================================================*/

/**
 * @brief   LORA UART 格式化输出 (类似 printf)
 * @param   fmt: 格式化字符串
 *          ...: 可变参数
 * @retval  无
 */
void atk_lora_01_uart_printf(char *fmt, ...)
{
    va_list ap;
    uint16_t len;

    va_start(ap, fmt);
    vsnprintf((char *)g_uart_tx_buf, ATK_LORA_01_UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    len = strlen((const char *)g_uart_tx_buf);
    for (uint16_t i = 0; i < len; i++)
    {
        DL_UART_transmitDataBlocking(ATK_LORA_01_UART_INST, g_uart_tx_buf[i]);
    }
}

/**
 * @brief   重新开始接收一帧数据
 * @param   无
 * @retval  无
 */
void atk_lora_01_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len   = 0;
    g_uart_rx_frame.sta.finsh = 0;
}

/**
 * @brief   获取接收到的数据帧
 * @param   无
 * @retval  NULL: 未接收到完整帧
 *          其他: 指向接收缓冲区的指针 (以 '\0' 结尾)
 */
uint8_t *atk_lora_01_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
        return NULL;
    }
}

/**
 * @brief   获取接收到的数据帧长度
 * @param   无
 * @retval  0   : 未接收到完整帧
 *          其他: 帧长度
 */
uint16_t atk_lora_01_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }
    else
    {
        return 0;
    }
}

/*===========================================================================
 * 模块基础控制
 *===========================================================================*/

/**
 * @brief   进入配置模式 (MD0=1)
 * @param   无
 * @retval  无
 */
void atk_lora_01_enter_config(void)
{
    ATK_LORA_01_MD0_Set();
    delay_ms(10);  /* 等待模式切换稳定 */
}

/**
 * @brief   退出配置模式 (MD0=0)
 * @param   无
 * @retval  无
 */
void atk_lora_01_exit_config(void)
{
    ATK_LORA_01_MD0_Clr();
    delay_ms(10);  /* 等待模式切换稳定 */
}

/**
 * @brief   判断模块是否空闲
 * @param   无
 * @retval  ATK_LORA_01_EOK  - 模块空闲 (AUX=0)
 *          ATK_LORA_01_EBUSY - 模块忙碌 (AUX=1)
 */
uint8_t atk_lora_01_free(void)
{
    if (ATK_LORA_01_AUX_Read() != 0)
    {
        return ATK_LORA_01_EBUSY;
    }

    return ATK_LORA_01_EOK;
}

/*===========================================================================
 * AT 指令操作
 *===========================================================================*/

/**
 * @brief   发送 AT 指令并等待应答
 * @param   cmd    : AT 指令字符串
 *          ack    : 期望应答关键字 (如 "OK"), 为 NULL 或 timeout 为 0 则不等待
 *          timeout: 超时时间 (毫秒)
 * @retval  ATK_LORA_01_EOK     - 收到期望应答
 *          ATK_LORA_01_ETIMEOUT - 超时
 */
uint8_t atk_lora_01_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t *ret = NULL;

    /* 等待模块空闲 */
    while (ATK_LORA_01_AUX_Read() != 0)
    {
        delay_ms(1);
    }

    /* 清空接收缓冲区, 准备接收应答 */
    atk_lora_01_uart_rx_restart();

    /* 发送 AT 指令 (带换行) */
    atk_lora_01_uart_printf("%s\r\n", cmd);

    /* 如果不需要等待应答, 直接返回 */
    if ((ack == NULL) || (timeout == 0))
    {
        return ATK_LORA_01_EOK;
    }

    /* 等待应答 */
    while (timeout > 0)
    {
        ret = atk_lora_01_uart_rx_get_frame();
        if (ret != NULL)
        {
            if (strstr((const char *)ret, ack) != NULL)
            {
                return ATK_LORA_01_EOK;
            }
            else
            {
                /* 未匹配到期望应答, 继续接收下一帧 */
                atk_lora_01_uart_rx_restart();
            }
        }
        timeout--;
        delay_ms(1);
    }

    return ATK_LORA_01_ETIMEOUT;
}

/**
 * @brief   AT 指令通讯测试
 * @param   无
 * @retval  ATK_LORA_01_EOK   - 通讯正常
 *          ATK_LORA_01_ERROR - 通讯失败 (重试10次)
 */
uint8_t atk_lora_01_at_test(void)
{
    uint8_t ret;
    uint8_t i;

    for (i = 0; i < 10; i++)
    {
        ret = atk_lora_01_send_at_cmd("AT", "OK", ATK_LORA_01_AT_TIMEOUT);
        if (ret == ATK_LORA_01_EOK)
        {
            return ATK_LORA_01_EOK;
        }
    }

    return ATK_LORA_01_ERROR;
}

/**
 * @brief   配置指令回显
 * @param   enable: ATK_LORA_01_DISABLE (关闭回显) / ATK_LORA_01_ENABLE (开启回显)
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_echo_config(atk_lora_01_enable_t enable)
{
    uint8_t ret;
    char cmd[5] = {0};

    switch (enable)
    {
        case ATK_LORA_01_ENABLE:
        {
            snprintf(cmd, sizeof(cmd), "ATE1");
            break;
        }
        case ATK_LORA_01_DISABLE:
        {
            snprintf(cmd, sizeof(cmd), "ATE0");
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   软件复位模块
 * @param   无
 * @retval  ATK_LORA_01_EOK   - 复位成功
 *          ATK_LORA_01_ERROR - 复位失败
 */
uint8_t atk_lora_01_sw_reset(void)
{
    uint8_t ret;

    ret = atk_lora_01_send_at_cmd("AT+RESET", "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置 Flash 保存
 * @param   enable: ATK_LORA_01_DISABLE (不保存) / ATK_LORA_01_ENABLE (保存)
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_flash_config(atk_lora_01_enable_t enable)
{
    uint8_t ret;
    char cmd[12] = {0};

    switch (enable)
    {
        case ATK_LORA_01_DISABLE:
        {
            snprintf(cmd, sizeof(cmd), "AT+FLASH=0");
            break;
        }
        case ATK_LORA_01_ENABLE:
        {
            snprintf(cmd, sizeof(cmd), "AT+FLASH=1");
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   恢复出厂设置
 * @param   无
 * @retval  ATK_LORA_01_EOK   - 恢复成功
 *          ATK_LORA_01_ERROR - 恢复失败
 */
uint8_t atk_lora_01_default(void)
{
    uint8_t ret;

    ret = atk_lora_01_send_at_cmd("AT+DEFAULT", "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/*===========================================================================
 * 模块参数配置
 *===========================================================================*/

/**
 * @brief   配置设备地址
 * @param   addr: 设备地址 (0x0000 ~ 0xFFFF)
 * @retval  ATK_LORA_01_EOK   - 配置成功
 *          ATK_LORA_01_ERROR - 配置失败
 */
uint8_t atk_lora_01_addr_config(uint16_t addr)
{
    uint8_t ret;
    char cmd[16] = {0};

    snprintf(cmd, sizeof(cmd), "AT+ADDR=%02X,%02X",
             (uint8_t)(addr >> 8) & 0xFF, (uint8_t)addr & 0xFF);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置发射功率
 * @param   tpower: 发射功率等级
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_tpower_config(atk_lora_01_tpower_t tpower)
{
    uint8_t ret;
    char cmd[14] = {0};

    switch (tpower)
    {
        case ATK_LORA_01_TPOWER_11DBM:
        case ATK_LORA_01_TPOWER_14DBM:
        case ATK_LORA_01_TPOWER_17DBM:
        case ATK_LORA_01_TPOWER_20DBM:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+TPOWER=%d", tpower);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置工作模式
 * @param   workmode: 工作模式
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_workmode_config(atk_lora_01_workmode_t workmode)
{
    uint8_t ret;
    char cmd[14] = {0};

    switch (workmode)
    {
        case ATK_LORA_01_WORKMODE_NORMAL:
        case ATK_LORA_01_WORKMODE_WAKEUP:
        case ATK_LORA_01_WORKMODE_LOWPOWER:
        case ATK_LORA_01_WORKMODE_SIGNAL:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%d", workmode);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置传输模式
 * @param   tmode: 传输模式 (透传/定点)
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_tmode_config(atk_lora_01_tmode_t tmode)
{
    uint8_t ret;
    char cmd[12] = {0};

    switch (tmode)
    {
        case ATK_LORA_01_TMODE_TT:
        case ATK_LORA_01_TMODE_DT:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+TMODE=%d", tmode);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置无线速率和信道
 * @note    参数顺序与 AT 指令一致: AT+WLRATE=<channel>,<wlrate>
 * @param   channel: 信道 (0 ~ 83)
 *          wlrate : 无线速率
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_wlrate_channel_config(uint8_t channel, atk_lora_01_wlrate_t wlrate)
{
    uint8_t ret;
    char cmd[16] = {0};

    /* 0.3Kbps 仅定点模式支持 */
    switch (wlrate)
    {
        case ATK_LORA_01_WLRATE_0K3:
        case ATK_LORA_01_WLRATE_1K2:
        case ATK_LORA_01_WLRATE_2K4:
        case ATK_LORA_01_WLRATE_4K8:
        case ATK_LORA_01_WLRATE_9K6:
        case ATK_LORA_01_WLRATE_19K2:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    if (channel > 83)
    {
        return ATK_LORA_01_EINVAL;
    }

    /* AT+WLRATE=<channel>,<wlrate> */
    snprintf(cmd, sizeof(cmd), "AT+WLRATE=%d,%d", channel, wlrate);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置唤醒时间
 * @param   wltime: 唤醒时间
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_wltime_config(atk_lora_01_wltime_t wltime)
{
    uint8_t ret;
    char cmd[14] = {0};

    switch (wltime)
    {
        case ATK_LORA_01_WLTIME_1S:
        case ATK_LORA_01_WLTIME_2S:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+WLTIME=%d", wltime);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/**
 * @brief   配置模块串口参数
 * @note    修改波特率/校验位后, 需要重新初始化 MCU 端串口并重新调用 atk_lora_01_init()
 * @param   baudrate: 波特率
 *          parity  : 校验位
 * @retval  ATK_LORA_01_EOK    - 配置成功
 *          ATK_LORA_01_ERROR  - 配置失败
 *          ATK_LORA_01_EINVAL - 参数无效
 */
uint8_t atk_lora_01_uart_config(atk_lora_01_uartrate_t baudrate, atk_lora_01_uartpari_t parity)
{
    uint8_t ret;
    char cmd[14] = {0};

    switch (baudrate)
    {
        case ATK_LORA_01_UARTRATE_1200BPS:
        case ATK_LORA_01_UARTRATE_2400BPS:
        case ATK_LORA_01_UARTRATE_4800BPS:
        case ATK_LORA_01_UARTRATE_9600BPS:
        case ATK_LORA_01_UARTRATE_19200BPS:
        case ATK_LORA_01_UARTRATE_38400BPS:
        case ATK_LORA_01_UARTRATE_57600BPS:
        case ATK_LORA_01_UARTRATE_115200BPS:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    switch (parity)
    {
        case ATK_LORA_01_UARTPARI_NONE:
        case ATK_LORA_01_UARTPARI_EVEN:
        case ATK_LORA_01_UARTPARI_ODD:
        {
            break;
        }
        default:
        {
            return ATK_LORA_01_EINVAL;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+UART=%d,%d", baudrate, parity);

    ret = atk_lora_01_send_at_cmd(cmd, "OK", ATK_LORA_01_AT_TIMEOUT);
    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/*===========================================================================
 * 模块初始化
 *===========================================================================*/

/**
 * @brief   ATK-LORA-01 模块初始化
 * @param   baudrate: UART 通信波特率 (如 115200)
 * @retval  ATK_LORA_01_EOK   - 初始化成功
 *          ATK_LORA_01_ERROR - 初始化失败
 */
uint8_t atk_lora_01_init(uint32_t baudrate)
{
    uint8_t ret;
    uint32_t wait;

    /* 硬件引脚初始化 */
    atk_lora_01_hw_init();

    /* UART 中断使能 */
    NVIC_EnableIRQ(ATK_LORA_01_UART_INT_IRQN);

    /* 等待模块空闲 (AUX=0), 超时约 500ms */
    wait = 500;
    while (ATK_LORA_01_AUX_Read() != 0)
    {
        delay_ms(1);
        if (--wait == 0)
        {
            break;
        }
    }

    /* 进入配置模式并测试 AT 通讯 */
    atk_lora_01_enter_config();
    ret = atk_lora_01_at_test();
    atk_lora_01_exit_config();

    if (ret != ATK_LORA_01_EOK)
    {
        return ATK_LORA_01_ERROR;
    }

    return ATK_LORA_01_EOK;
}

/*===========================================================================
 * 中断服务函数
 *===========================================================================*/

/**
 * @brief   LORA UART 中断服务函数
 * @note    需要在 UART 中断向量中调用此函数
 *          在 SysConfig 中配置 UART 中断后, 在对应的 IRQHandler 中调用
 * @param   无
 * @retval  无
 *
 * 使用示例 (在生成的 ti_msp_dl_config.c 或用户中断文件中):
 * void UART_2_INST_IRQHandler(void)
 * {
 *     ATK_LORA_01_UART_IRQHandler();
 * }
 */
void ATK_LORA_01_UART_IRQHandler(void)
{
    uint8_t tmp;

    switch (DL_UART_getPendingInterrupt(ATK_LORA_01_UART_INST))
    {
        case DL_UART_IIDX_RX:
        {
            /* 读取接收到的字节 */
            tmp = DL_UART_receiveData(ATK_LORA_01_UART_INST);

            /* 判断接收缓冲区是否溢出 (预留一位放 '\0') */
            if (g_uart_rx_frame.sta.len < (ATK_LORA_01_UART_RX_BUF_SIZE - 1))
            {
                /* 重置定时器计数值, 重新开始计时 */
                g_timer_tick = 0;

                /* 如果是帧的第一字节, 启动定时器 */
                if (g_uart_rx_frame.sta.len == 0)
                {
                    DL_Timer_startCounter(ATK_LORA_01_TIMER_INST);
                }

                /* 写入缓冲区 */
                g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;
                g_uart_rx_frame.sta.len++;
            }
            else
            {
                /* 缓冲区溢出, 丢弃之前数据, 重新开始 */
                g_uart_rx_frame.sta.len = 0;
                g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;
                g_uart_rx_frame.sta.len++;
            }
            break;
        }

        default:
            break;
    }
}

/**
 * @brief   LORA 定时器中断服务函数
 * @note    需要在定时器中断向量中调用此函数
 *          在 SysConfig 中配置定时器中断后, 在对应的 IRQHandler 中调用
 * @param   无
 * @retval  无
 *
 * 定时器用于检测帧间超时:
 * - 每收到一个字节, g_timer_tick 清零
 * - 定时器周期性累加 g_timer_tick
 * - 当 g_timer_tick 超过阈值 (如 >10, 约 100ms), 认为一帧结束
 *
 * 使用示例:
 * void TIMER_0_INST_IRQHandler(void)
 * {
 *     ATK_LORA_01_TIM_IRQHandler();
 * }
 */
void ATK_LORA_01_TIM_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(ATK_LORA_01_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
        {
            /* 定时器计数值递增 */
            g_timer_tick++;

            /*
             * 帧超时阈值: 约 100ms
             * 定时器周期由 SysConfig 配置 (当前为 10ms, prescale=256)
             * 当 100ms 内没有新数据, 认为一帧结束
             */
            if (g_timer_tick > 10)
            {
                /* 停止定时器 */
                DL_Timer_stopCounter(ATK_LORA_01_TIMER_INST);

                /* 如果接收缓冲区有数据, 标记帧完成 */
                if (g_uart_rx_frame.sta.len > 0)
                {
                    g_uart_rx_frame.sta.finsh = 1;
                }

                g_timer_tick = 0;
            }
            break;
        }

        default:
            break;
    }
}
