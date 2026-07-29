#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

typedef struct {
    uint16_t cc_left;
    uint16_t cc_right;
    uint8_t ain1;
    uint8_t ain2;
    uint8_t bin1;
    uint8_t bin2;
    uint8_t pwm_left_pin;
    uint8_t pwm_right_pin;
    uint32_t gpioa_dout;
    uint32_t gpioa_doe;
    uint32_t gpiob_dout;
    uint32_t gpiob_doe;
} Motor_DebugStatus;

void Motor_Init(void);
void Set_Motor_Speed_Left(int16_t speed);
void Set_Motor_Speed_Right(int16_t speed);
void Motor_ReadDebugStatus(Motor_DebugStatus *status);

#endif
