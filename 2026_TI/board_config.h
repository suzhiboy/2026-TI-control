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
 * 4 键菜单系统 按键引脚定义
 *
 * K1 (PB12): 上一个任务  — 独立 GPIO
 * K2 (PB13): 下一个任务  — 与循迹 AD2 分时复用, RUNNING 时不响应
 * K3 (PB2):  调目标点    — 原 IMU601 TX
 * K4 (PB3):  确认/启动   — 原 IMU601 RX
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
