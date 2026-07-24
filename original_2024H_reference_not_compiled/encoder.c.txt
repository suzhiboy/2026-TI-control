#include "encoder.h"
#include "control.h"
#include "mpu6050.h"
#include <ti/devices/msp/msp.h>

// 实例化全局数据中心变量
Encoder_Data_t g_Encoder = {0};

// 内部软件计数器
static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

/**
 * @brief 编码器初始化 (只保留编码器中断)
 */
void Encoder_Init(void) {
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 统一中断服务函数 (纯净版，只处理编码器)
 */
void GROUP1_IRQHandler(void) {
    uint32_t pendingGroup = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
    if (pendingGroup & DL_INTERRUPT_GROUP1_IIDX_GPIOB) {
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);

        // 左轮捕获
        if (gpio_status & DL_GPIO_PIN_7) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);
            // 恢复镜像安装逻辑：前进时左轮 A 领先 B
            if (phase_a != phase_b) left_pulse_count--; else left_pulse_count++;
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
        }
        // 右轮捕获
        if (gpio_status & DL_GPIO_PIN_6) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);
            if (phase_a != phase_b) right_pulse_count++; else right_pulse_count--;
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
        }
    }
}

/**
 * @brief 10ms 数据解算
 */
void Encoder_UpdateData_10ms(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_Encoder.speed_left = left_pulse_count;
    g_Encoder.speed_right = right_pulse_count;
    left_pulse_count = 0;
    right_pulse_count = 0;
    __set_PRIMASK(primask);
    __enable_irq();

    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    // 恢复原始计算方式：因为底层中断处理已保证前进时两者都是正数
    float avg_pulses = (float)(g_Encoder.pulses_left + g_Encoder.pulses_right) / 2.0f;
    g_Encoder.distance_cm = avg_pulses * PULSE_TO_CM;
}

void Encoder_Clear(void) {
    g_Encoder.pulses_left = 0;
    g_Encoder.pulses_right = 0;
    g_Encoder.distance_cm = 0.0f;
    left_pulse_count = 0;
    right_pulse_count = 0;
}
