#ifndef __PID_H_
#define __PID_H_

#include "stdint.h"

// PID 控制器结构体
typedef struct
{
    float target;           // 目标值
    float actual;           // 实际值
    float err;              // 当前误差
    float err_last;         // 上一次误差
    float Kp, Ki, Kd;       // PID 参数
    float integral;         // 误差积分
    float output;           // 最终输出值
    float out_max;          // 输出上限
    float out_min;          // 输出下限
    float i_max;            // 积分限幅
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max, float min, float imax);
float PID_Calc_Positional(PID_TypeDef *pid, float actual);
float PID_Calc_Incremental(PID_TypeDef *pid, float actual);
void PID_Clear(PID_TypeDef *pid);

#endif
