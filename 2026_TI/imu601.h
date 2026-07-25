#ifndef IMU601_H
#define IMU601_H

#include "ti_msp_dl_config.h"
#include "uart.h"
#include "delay.h"

// 接线
// 汇电籽-601   mspm0g3507
// V           5V
// G           GND
// T           PA25 (UART1 TX)
// R           PA26 (UART1 RX)

/* UART 实例定义 (SysConfig 未配置时使用以下默认值) */
#ifndef IMU601_INST
#define IMU601_INST            UART1           /* UART1 外设 */
#endif

#ifndef IMU601_INST_INT_IRQN
#define IMU601_INST_INT_IRQN   UART1_INT_IRQn  /* UART1 中断号 */
#endif

void IMU601_init();

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;


#endif /* IMU601_H */
