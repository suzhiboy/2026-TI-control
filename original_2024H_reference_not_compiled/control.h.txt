#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "ti_msp_dl_config.h"
#include "pid.h"

/* --- 任务模式定义 (全局可见) --- */
#define TASK_IDLE               0
#define TASK_CALIBRATING        1   // 新增：静止校准状态
#define TASK_1_AB_STRAIGHT      2
#define TASK_2_ABCD_CIRCLE      3
#define TASK_3_ACBD_DIAGONAL    4
#define TASK_4_FOUR_LAPS        5
#define TASK_FINISHED           100

/* --- 控制器外部声明 --- */
extern PID_TypeDef pid_line12;
extern PID_TypeDef pid_line34;
extern PID_TypeDef pid_yaw;
extern PID_TypeDef pid_yaw34;
extern PID_TypeDef pid_speed_L;
extern PID_TypeDef pid_speed_R;
extern uint8_t Car_Mode;
extern uint8_t g_target_task;

void Control_Init(void);
void Control_Loop(void);
void Control_Reset(void);
void Reset_Encoder_Distance(void); // 声明重置里程计接口
void Vofa_Send_Debug(void);
#endif
