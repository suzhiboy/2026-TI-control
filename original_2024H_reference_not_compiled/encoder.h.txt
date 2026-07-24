#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

// --- 物理参数定义 (根据你的 310 电机实测修改) ---
#define WHEEL_DIAMETER    4.8f    // 轮子直径 (单位: cm)
#define ENCODER_RES       13.0f   // 编码器单相线数
#define GEAR_RATIO        20.0f   // 电机减速比
#define PI                3.1415926f

// 软件 4 倍频下的换算系数：1个脉冲走多少厘米
// 公式: (轮径 * PI) / (线数 * 减速比 * 4)
#define PULSE_TO_CM  ((WHEEL_DIAMETER * PI) / (ENCODER_RES * GEAR_RATIO * 4.0f))

typedef struct {
    int16_t  speed_left;        // 当前速度 (脉冲/10ms)
    int16_t  speed_right;       // 当前速度 (脉冲/10ms)
    int32_t  pulses_left;       // 累计总脉冲
    int32_t  pulses_right;      // 累计总脉冲
    float    distance_cm;       // 换算后的行驶距离 (单位: cm)
} Encoder_Data_t;

extern Encoder_Data_t g_Encoder;

void Encoder_Init(void);
void Encoder_UpdateData_10ms(void);
void Encoder_Clear(void);

#endif