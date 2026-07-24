#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct
{
    float target;
    float actual;
    float err;
    float err_last;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float output;
    float out_max;
    float out_min;
    float i_max;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max, float min, float imax);
void PID_Clear(PID_TypeDef *pid);
float PID_Calc_Positional(PID_TypeDef *pid, float actual);
float PID_Calc_Incremental(PID_TypeDef *pid, float actual);

#endif
