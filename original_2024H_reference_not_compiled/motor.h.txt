#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Set_Motor_Speed_Left(int16_t speed);
void Set_Motor_Speed_Right(int16_t speed);

#endif
