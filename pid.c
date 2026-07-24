#include "pid.h"

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max, float min, float imax)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->out_max = max;
    pid->out_min = min;
    pid->i_max = imax;
    PID_Clear(pid);
}

void PID_Clear(PID_TypeDef *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calc_Positional(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    pid->err = pid->target - actual;

    pid->integral += pid->err;
    if (pid->integral > pid->i_max) {
        pid->integral = pid->i_max;
    }
    if (pid->integral < -pid->i_max) {
        pid->integral = -pid->i_max;
    }

    pid->output = pid->Kp * pid->err +
                  pid->Ki * pid->integral +
                  pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    }
    if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

    return pid->output;
}

float PID_Calc_Incremental(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    float err = pid->target - actual;

    float delta_output = pid->Kp * (err - pid->err_last) + pid->Ki * err;
    pid->output += delta_output;
    pid->err_last = err;

    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    }
    if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

    return pid->output;
}
