#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"
#include "sensor.h"
#include "oled.h"
#include <stdio.h>

volatile uint8_t g_vofa_send_flag = 0;
static uint8_t g_imu_id = 0; 
static uint8_t g_oled_refresh_div = 0;

void Debug_Sensors_Display(void);
extern float pitch2, roll2, Yaw;
extern float filtered_L, filtered_R;

// 按键扫描处理 (改为软件消抖，不干扰中断)
void Scan_Keys(void) {
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3) == 0) {
        delay_ms(20); if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3) == 0) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();MPU6050_ResetBiasCalibration(); g_target_task = TASK_1_AB_STRAIGHT; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3) == 0);
        }
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13) == 0) {
        delay_ms(20); if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13) == 0) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();MPU6050_ResetBiasCalibration(); g_target_task = TASK_2_ABCD_CIRCLE; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13) == 0);
        }
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12) == 0) {
        delay_ms(20); if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12) == 0) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();MPU6050_ResetBiasCalibration(); g_target_task = TASK_3_ACBD_DIAGONAL; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12) == 0);
        }
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2) == 0) {
        delay_ms(20); if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2) == 0) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();MPU6050_ResetBiasCalibration(); g_target_task = TASK_4_FOUR_LAPS; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2) == 0);
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    Control_Init();
    mpu6050_init();
    
    extern uint8_t g_imu_addr;
    extern uint8_t Single_ReadI2C(unsigned char SlaveAddress, unsigned char REG_Address);
    g_imu_id = Single_ReadI2C(g_imu_addr, 0x75);

    DL_UART_Main_enable(UART_BLUETOOTH_INST);
    
    // 关键修复：设置中断优先级
    // 抢占优先级：数字越小优先级越高。
    // 必须让编码器中断(0)打断定时器控制环(2)，防止长达数毫秒的软件 I2C 通信阻塞导致编码器严重丢步！
    NVIC_SetPriority(GPIOB_INT_IRQn, 0);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2);
    
    __enable_irq();
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    
    while (1) 
    {
        Scan_Keys();
        if (g_vofa_send_flag) {
            g_vofa_send_flag = 0;
            Vofa_Send_Debug(); 
        }
        if (++g_oled_refresh_div >= 20) { 
            g_oled_refresh_div = 0;
            Debug_Sensors_Display();
        }
    }
}

// 恢复您的原始排版与原始变量
void Debug_Sensors_Display(void)
{
    char disp_buf[32]; 
    sprintf(disp_buf, "Y:%.1f ID:%02X   ", mpu6050.Yaw, g_imu_id);
    OLED_ShowString(0, 0, (uint8_t *)disp_buf, 16, 1);

    extern float Gyro_Z_Measeure;
    sprintf(disp_buf, "G:%.1f deg/s   ", Gyro_Z_Measeure);
    OLED_ShowString(0, 16, (uint8_t *)disp_buf, 16, 1);

    sprintf(disp_buf, "L:%d R:%d   ", (int)g_Encoder.speed_left, (int)g_Encoder.speed_right);
    OLED_ShowString(0, 32, (uint8_t *)disp_buf, 16, 1);

    sprintf(disp_buf, "D:%.1f M:%d    ", g_Encoder.distance_cm, (int)Car_Mode);
    OLED_ShowString(0, 48, (uint8_t *)disp_buf, 16, 1); 

    OLED_Update();
}

void TIMER_0_INST_IRQHandler(void) {
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            AHRS_Geteuler_WithDt(0.01f);
            Control_Loop(); 
            g_vofa_send_flag = 1; 
            break;
        default:
            break;
    }
}
