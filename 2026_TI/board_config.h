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
 * Six direct task keys.
 *
 * K1..K6 map to TASK_T1..TASK_T6 in key_menu.c.
 * Pin map: K1=PB14, K2=PB11, K3=PB10, K4=PB1, K5=PB0, K6=PA30.
 * All six pins are listed in empty.syscfg under GPIO_KEY.
 * Buttons are active-low with pull-up input.
 */

#ifndef KEY_K1_PORT
#define KEY_K1_PORT     GPIOB
#endif

#ifndef KEY_K1_PIN
#define KEY_K1_PIN      DL_GPIO_PIN_14
#endif

#ifndef KEY_K1_IOMUX
#define KEY_K1_IOMUX    GPIO_KEY_K1_IOMUX
#endif

#ifndef KEY_K2_PORT
#define KEY_K2_PORT     GPIOB
#endif

#ifndef KEY_K2_PIN
#define KEY_K2_PIN      DL_GPIO_PIN_11
#endif

#ifndef KEY_K2_IOMUX
#define KEY_K2_IOMUX    GPIO_KEY_K2_IOMUX
#endif

#ifndef KEY_K3_PORT
#define KEY_K3_PORT     GPIOB
#endif

#ifndef KEY_K3_PIN
#define KEY_K3_PIN      DL_GPIO_PIN_10
#endif

#ifndef KEY_K3_IOMUX
#define KEY_K3_IOMUX    GPIO_KEY_K3_IOMUX
#endif

#ifndef KEY_K4_PORT
#define KEY_K4_PORT     GPIOB
#endif

#ifndef KEY_K4_PIN
#define KEY_K4_PIN      DL_GPIO_PIN_1
#endif

#ifndef KEY_K4_IOMUX
#define KEY_K4_IOMUX    GPIO_KEY_K4_IOMUX
#endif

#ifndef KEY_K5_PORT
#define KEY_K5_PORT     GPIOB
#endif

#ifndef KEY_K5_PIN
#define KEY_K5_PIN      DL_GPIO_PIN_0
#endif

#ifndef KEY_K5_IOMUX
#define KEY_K5_IOMUX    GPIO_KEY_K5_IOMUX
#endif

#ifndef KEY_K6_PORT
#define KEY_K6_PORT     GPIOA
#endif

#ifndef KEY_K6_PIN
#define KEY_K6_PIN      DL_GPIO_PIN_30
#endif

#ifndef KEY_K6_IOMUX
#define KEY_K6_IOMUX    GPIO_KEY_K6_IOMUX
#endif

#endif /* BOARD_CONFIG_H */
