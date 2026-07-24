#include "motor.h"

static int16_t last_left = 0;
static int16_t last_right = 0;

void Motor_Init(void)
{
    DL_TimerG_startCounter(PWM_MOTOR_INST);
}

void Set_Motor_Speed_Left(int16_t speed)
{
    if (speed < 0) speed = 0; 

    if (last_left > 500 && speed == 0) {
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
        delay_cycles(16000); 
    }
    last_left = speed;

    if (speed > 2000) speed = 2000;

    if (speed > 0) {
        DL_GPIO_setPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}

void Set_Motor_Speed_Right(int16_t speed)
{
    if (speed < 0) speed = 0;

    if (last_right > 500 && speed == 0) {
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
        delay_cycles(16000);
    }
    last_right = speed;

    if (speed > 2000) speed = 2000;

    if (speed > 0) {
        DL_GPIO_setPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    }
}
