
#include "pid.h"

/**
 * @brief PID 参数初始化
 */
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max, float min, float imax)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->out_max = max;
    pid->out_min = min;
    pid->i_max = imax;
}

void PID_Clear(PID_TypeDef *pid)
{
    pid->target = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0; // 清除位置式 PID 的积分累加
    pid->output = 0;   // 清除增量式 PID 的当前输出（因为增量式是基于此值累加的）
}

/**
 * @brief 位置式 PID 计算
 */
float PID_Calc_Positional(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    pid->err = pid->target - actual;
    
    // 积分累加
    pid->integral += pid->err;
    
    // 积分限幅 (抗饱和)
    if (pid->integral > pid->i_max) pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max) pid->integral = -pid->i_max;
    
    // 计算输出
    pid->output = pid->Kp * pid->err + 
                  pid->Ki * pid->integral + 
                  pid->Kd * (pid->err - pid->err_last);
                  
    pid->err_last = pid->err;
    
    // 输出限幅
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;
    
    return pid->output;
}

/**
 * @brief 增量式 PID 计算
 */
float PID_Calc_Incremental(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    float err = pid->target - actual;
    
    float delta_output = pid->Kp * (err - pid->err_last) + 
                         pid->Ki * err;
                         
    pid->output += delta_output;
    pid->err_last = err;
    
    // 输出限幅
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;
    
    return pid->output;
}

