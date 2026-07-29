#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "ti_msp_dl_config.h"

#ifndef STARTUP_SPLASH_DELAY_CYCLES
#define STARTUP_SPLASH_DELAY_CYCLES    (1600000U)
#endif

#ifndef MOTOR_PWM_MAX
#define MOTOR_PWM_MAX                  (2000)
#endif

#ifndef MOTOR_LEFT_FORWARD_AIN1_ACTIVE
#define MOTOR_LEFT_FORWARD_AIN1_ACTIVE (0U)
#endif

#ifndef MOTOR_LEFT_FORWARD_AIN2_ACTIVE
#define MOTOR_LEFT_FORWARD_AIN2_ACTIVE (1U)
#endif

#ifndef MOTOR_RIGHT_FORWARD_BIN1_ACTIVE
#define MOTOR_RIGHT_FORWARD_BIN1_ACTIVE (0U)
#endif

#ifndef MOTOR_RIGHT_FORWARD_BIN2_ACTIVE
#define MOTOR_RIGHT_FORWARD_BIN2_ACTIVE (1U)
#endif

#ifndef LINE_SPEED_PID_OUTPUT_MAX
#define LINE_SPEED_PID_OUTPUT_MAX      (2000.0f)
#endif

#ifndef LINE_SPEED_PID_INTEGRAL_MAX
#define LINE_SPEED_PID_INTEGRAL_MAX    (1000.0f)
#endif

#ifndef SENSOR_SWITCH_DELAY_CYCLES
#define SENSOR_SWITCH_DELAY_CYCLES     (1600U)
#endif

#ifndef PRINT_INST
#define PRINT_INST                     UART0
#endif

#ifndef VISION_UART_INST
#define VISION_UART_INST               UART0
#endif

#ifndef VISION_UART_INST_IRQHandler
#define VISION_UART_INST_IRQHandler    UART0_IRQHandler
#endif

#ifndef VISION_UART_INST_INT_IRQN
#define VISION_UART_INST_INT_IRQN      UART0_INT_IRQn
#endif

#ifndef TM1637_PORT
#define TM1637_PORT                    GPIOA
#endif

#ifndef TM1637_CLK_PIN
#define TM1637_CLK_PIN                 DL_GPIO_PIN_8
#endif

#ifndef TM1637_CLK_IOMUX
#define TM1637_CLK_IOMUX               IOMUX_PINCM9
#endif

#ifndef TM1637_DIO_PIN
#define TM1637_DIO_PIN                 DL_GPIO_PIN_9
#endif

#ifndef TM1637_DIO_IOMUX
#define TM1637_DIO_IOMUX               IOMUX_PINCM10
#endif

/*
 * ATK-LORA-01 模块引脚 / 外设定义
 * 模块型号: 正点原子 ATK-LORA-01 (核心芯片: MW1278D)
 *
 * 硬件接线:
 *   ATK-LORA-01    MSPM0G3507
 *   VCC            ─── 5V
 *   GND            ─── GND
 *   TXD            ─── PA1  (UART0_RX)
 *   RXD            ─── PA0  (UART0_TX)
 *   MD0            ─── PB0  (GPIO 输出, IOMUX_PINCM12)
 *   AUX            ─── PB1  (GPIO 输入, IOMUX_PINCM13)
 *
 * 注意: 需要在 SysConfig 中添加:
 *   - UART 实例 "LORA_UART", 外设 UART0, TX=PA0, RX=PA1, 115200 8N1
 *   - TIMER 实例 "LORA_TIMER", 外设 TIMA1, 10ms periodic
 */
#ifndef BOARD_ATK_LORA_01_MD0_PORT
#define BOARD_ATK_LORA_01_MD0_PORT      GPIOB
#endif

#ifndef BOARD_ATK_LORA_01_MD0_PIN
#define BOARD_ATK_LORA_01_MD0_PIN       DL_GPIO_PIN_0
#endif

/* PB0 = IOMUX_PINCM12 (not IMU601 RX; IMU601 now uses UART1 RX on PB7) */
#ifndef BOARD_ATK_LORA_01_MD0_IOMUX
#define BOARD_ATK_LORA_01_MD0_IOMUX     IOMUX_PINCM12
#endif

#ifndef BOARD_ATK_LORA_01_AUX_PORT
#define BOARD_ATK_LORA_01_AUX_PORT      GPIOB
#endif

#ifndef BOARD_ATK_LORA_01_AUX_PIN
#define BOARD_ATK_LORA_01_AUX_PIN       DL_GPIO_PIN_1
#endif

/* PB1 = IOMUX_PINCM13 (不是 PINCM17! PINCM17=PB4) */
#ifndef BOARD_ATK_LORA_01_AUX_IOMUX
#define BOARD_ATK_LORA_01_AUX_IOMUX     IOMUX_PINCM13
#endif

/* === UART 配置: 使用 UART0 (PA0=TX, PA1=RX) === */
#ifndef BOARD_ATK_LORA_01_UART_INST
#define BOARD_ATK_LORA_01_UART_INST     LORA_UART_INST
#endif

#ifndef BOARD_ATK_LORA_01_UART_INT_IRQN
#define BOARD_ATK_LORA_01_UART_INT_IRQN LORA_UART_INST_INT_IRQN
#endif

/* === TIMER 配置: 使用 TIMA1, 10ms 周期 (帧超时检测) === */
#ifndef BOARD_ATK_LORA_01_TIMER_INST
#define BOARD_ATK_LORA_01_TIMER_INST    LORA_TIMER_INST
#endif

/* === 旧名称兼容 (过渡用, 后续移除) === */
#define BOARD_ATK_MW1278D_MD0_PORT      BOARD_ATK_LORA_01_MD0_PORT
#define BOARD_ATK_MW1278D_MD0_PIN       BOARD_ATK_LORA_01_MD0_PIN
#define BOARD_ATK_MW1278D_MD0_IOMUX     BOARD_ATK_LORA_01_MD0_IOMUX
#define BOARD_ATK_MW1278D_AUX_PORT      BOARD_ATK_LORA_01_AUX_PORT
#define BOARD_ATK_MW1278D_AUX_PIN       BOARD_ATK_LORA_01_AUX_PIN
#define BOARD_ATK_MW1278D_AUX_IOMUX     BOARD_ATK_LORA_01_AUX_IOMUX
#define BOARD_ATK_MW1278D_UART_INST     BOARD_ATK_LORA_01_UART_INST
#define BOARD_ATK_MW1278D_UART_INT_IRQN BOARD_ATK_LORA_01_UART_INT_IRQN
#define BOARD_ATK_MW1278D_TIMER_INST    BOARD_ATK_LORA_01_TIMER_INST

/*
 * 4 键菜单系统 按键引脚定义
 *
 * K1 (PB12): 上一个任务  — 独立 GPIO
 * K2 (PB13): 下一个任务  — 与循迹 AD2 分时复用, RUNNING 时不响应
 * K3 (PB2):  调目标点    — 原 IMU601 TX, syscfg 再生后释放
 * K4 (PB3):  确认/启动   — 原 IMU601 RX, syscfg 再生后释放
 *
 * 注意: PB2/PB3 需要在 SysConfig 中将 IMU601 从 UART3 移到 UART1(PB6/PB7)
 *       并重新生成 ti_msp_dl_config.c/h 后方可作为 GPIO 使用.
 */

#ifndef KEY_K1_PORT
#define KEY_K1_PORT     GPIOB
#endif

#ifndef KEY_K1_PIN
#define KEY_K1_PIN      DL_GPIO_PIN_12
#endif

#ifndef KEY_K2_PORT
#define KEY_K2_PORT     GPIOB
#endif

#ifndef KEY_K2_PIN
#define KEY_K2_PIN      DL_GPIO_PIN_13
#endif

#ifndef KEY_K3_PORT
#define KEY_K3_PORT     GPIOB
#endif

#ifndef KEY_K3_PIN
#define KEY_K3_PIN      DL_GPIO_PIN_2
#endif

#ifndef KEY_K4_PORT
#define KEY_K4_PORT     GPIOB
#endif

#ifndef KEY_K4_PIN
#define KEY_K4_PIN      DL_GPIO_PIN_3
#endif

#endif /* BOARD_CONFIG_H */
