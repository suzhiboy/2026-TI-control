#include "motor.h"
#include "board_config.h"
#include <stdbool.h>

static int16_t clamp_pwm(int16_t speed)
{
    if (speed < 0) {
        return 0;
    }
    if (speed > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }
    return speed;
}

static void motor_write_pin(GPIO_Regs *port, uint32_t pin, bool active)
{
    if (active) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

void Motor_Init(void)
{
    DL_TimerG_startCounter(PWM_MOTOR_INST);
}

void Set_Motor_Speed_Left(int16_t speed)
{
    speed = clamp_pwm(speed);

    if (speed > 0) {
        motor_write_pin(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN,
            MOTOR_LEFT_FORWARD_AIN1_ACTIVE != 0U);
        motor_write_pin(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN,
            MOTOR_LEFT_FORWARD_AIN2_ACTIVE != 0U);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}

void Set_Motor_Speed_Right(int16_t speed)
{
    speed = clamp_pwm(speed);

    if (speed > 0) {
        motor_write_pin(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN,
            MOTOR_RIGHT_FORWARD_BIN1_ACTIVE != 0U);
        motor_write_pin(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN,
            MOTOR_RIGHT_FORWARD_BIN2_ACTIVE != 0U);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    }
}
