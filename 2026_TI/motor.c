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
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
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
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}

void Motor_ReadDebugStatus(Motor_DebugStatus *status)
{
    if (status == NULL) {
        return;
    }

    status->cc_left = (uint16_t)DL_TimerG_getCaptureCompareValue(PWM_MOTOR_INST, DL_TIMER_CC_0_INDEX);
    status->cc_right = (uint16_t)DL_TimerG_getCaptureCompareValue(PWM_MOTOR_INST, DL_TIMER_CC_1_INDEX);
    status->ain1 = !!DL_GPIO_readPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
    status->ain2 = !!DL_GPIO_readPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
    status->bin1 = !!DL_GPIO_readPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
    status->bin2 = !!DL_GPIO_readPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
    status->pwm_left_pin = !!DL_GPIO_readPins(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    status->pwm_right_pin = !!DL_GPIO_readPins(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
    status->gpioa_dout = GPIOA->DOUT31_0;
    status->gpioa_doe = GPIOA->DOE31_0;
    status->gpiob_dout = GPIOB->DOUT31_0;
    status->gpiob_doe = GPIOB->DOE31_0;
}
