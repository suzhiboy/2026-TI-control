#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "main.h"
#include <stdio.h>

/* --- 控制器与状态变量 --- */
PID_TypeDef pid_line12;
PID_TypeDef pid_line34;  
PID_TypeDef pid_yaw;
PID_TypeDef pid_yaw34; 
PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;

uint8_t Car_Mode = TASK_IDLE;  
uint8_t g_target_task = TASK_IDLE; 
uint8_t Current_Step = 0;      
uint8_t Lap_Counter = 0;       
uint16_t Feedback_Timer = 0;   

float filtered_L = 0;
float filtered_R = 0;

/* --- 辅助功能函数 --- */

void Reset_Encoder_Distance(void) {
    Encoder_Clear(); 
}

bool Is_On_CrossLine(void) {
    uint8_t s[8];
    Sensor_Read_All(s);
    int count = 0;
    for(int i=0; i<8; i++) if(s[i] == 1) count++;
    
    return (count >= 1); 
}
// 角度归一化函数
float normalize_angle_error(float current, float target) {
    float err = target - current;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// 转弯完成判断函数
bool is_turn_completed(float current_angle, float target_angle, float distance, bool on_line) {
    float angle_err = normalize_angle_error(current_angle, target_angle);
    return (fabsf(angle_err) < 20.0f) && 
           (distance > 30.0f) && 
           (on_line || fabsf(angle_err) < 10.0f);
}

void Trigger_Feedback(void) {
    Feedback_Timer = 5;  // 50ms
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_1);   // 蜂鸣器PB1响（低电平）
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_22);    // LED PB22亮（高电平）
}

void Control_Init(void)
{
    Motor_Init();
    Encoder_Init(); 
    // 循迹 PID 优化参数 (增加积分项 Ki 解决不居中问题)
    //PID_Init(&pid_line, 0.8f, 0.05f, 2.5f, 8.0f, -8.0f, 1.0f);
    PID_Init(&pid_line12, 0.7f, 0.05f, 0.5f, 10.0f,-10.0f, 8.0f);
    PID_Init(&pid_line34, 1.0f, 0.05f, 3.5f, 8.0f,-8.0f, 4.0f);
    PID_Init(&pid_yaw, 1.5f, 0.02f, 1.0f, 10.0f, -10.0f, 2.0f);
    PID_Init(&pid_yaw34, 1.5f, 0.0f, 3.5f, 5.0f, -5.0f, 0.0f);
    PID_Init(&pid_speed_L, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    PID_Init(&pid_speed_R, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    Control_Reset();
}

void Control_Loop(void)
{
    float turn_out = 0;
    float base_speed = 0;
    int16_t final_L_pwm = 0, final_R_pwm = 0;

    Encoder_UpdateData_10ms(); 

    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    // 声光反馈处理
    if (Feedback_Timer > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7); 
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_1);   // 蜂鸣器响（低电平）
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_22);    // LED亮（高电平）
        Feedback_Timer--;
    } else { 
    // 去掉 else if 的条件，倒计时结束无条件关闭
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_7);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);   // 关闭蜂鸣器（高电平）
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_22); // 关闭LED（低电平）
}

    switch (Car_Mode) 
    {
        case TASK_IDLE: 
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            break;

        case TASK_CALIBRATING:
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            if (MPU6050_Is_Calibrated()) {
                Reset_Encoder_Distance();
                Car_Mode = g_target_task; 
            }
            break;

        case TASK_1_AB_STRAIGHT:
            base_speed = 15.0f;
            pid_yaw.target = 0.0f; 
            turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
            pid_speed_L.target = base_speed + turn_out;
            pid_speed_R.target = base_speed - turn_out;
            if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                Trigger_Feedback(); Car_Mode = TASK_FINISHED; 
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
            }
            break;

        case TASK_2_ABCD_CIRCLE:
            if (Current_Step == 0) { // A -> B (直线 100cm)
                base_speed = 15.0f; pid_yaw.target = 0.0f;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 95.0f || Is_On_CrossLine()) {
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            } 
            else if (Current_Step == 1) { // B -> C (弧线循迹)
                base_speed = 9.0f;
                turn_out = PID_Calc_Positional(&pid_line12, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) >= 160.0f || g_Encoder.distance_cm > 100.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { // C -> D (直线 100cm)
                base_speed = 15.0f; 
                float err = 180.0f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 3) { // D -> A (弧线循迹)
                base_speed = 9.0f;
                turn_out = PID_Calc_Positional(&pid_line12, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) <= 20.0f || g_Encoder.distance_cm > 95.0f) { 
                    Trigger_Feedback(); 
                    Car_Mode = TASK_FINISHED; 
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            break;

        case TASK_4_FOUR_LAPS:
            if (Current_Step == 0) { // A -> C 对角线
                base_speed = 14.0f; 
                float target_angle = 38.7f;
                if (g_Encoder.distance_cm > 60.0f) {
                    target_angle=20.0f;
                    base_speed=10.0f;
                }
                float err = target_angle - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw34.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw34, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 离开 A 点 60cm 后才探测 C 点，防止误触发
                if (g_Encoder.distance_cm > 60.0f && Is_On_CrossLine()) { 
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 1) { // C -> B 循迹
                base_speed = 13.0f;//8
                turn_out = PID_Calc_Positional(&pid_line34, Sensor_Get_Error());
                float current_err = Sensor_Get_Error();
                // 当误差极大 (>= 20) 时，说明正在斜线入弯，或者触发了丢线记忆
                if (current_err >= 20.0f || current_err <= -20.0f) {
                    pid_line34.Kp = 2.5f;       // 极其暴力的 P，往死里拽
                    pid_line34.out_max = 16.0f; // 放开限幅！允许输出 16
                    pid_line34.out_min = -16.0f;
                }
                    // 当误差较小 (< 20) 时，说明车头已经顺滑地贴在黑线上了
                else {
                    pid_line34.Kp = 0.6f;       // 恢复温柔的 P，防止画龙
                    pid_line34.out_max = 6.0f;  // 收紧限幅，让它乖乖巡航
                    pid_line34.out_min = -6.0f;
                }
                
                turn_out = PID_Calc_Positional(&pid_line34, current_err);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 里程 > 70cm 且角度接近 180 即认为出弯到达 B 点
                if (g_Encoder.distance_cm > 60.0f && absFloat(mpu6050.Yaw) >= 143.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 2) { // B -> D 对角线
                base_speed = 14.0f; 
                float target_angle = 141.3f;
                if (g_Encoder.distance_cm > 60.0f) {
                    target_angle=160.0f;
                    base_speed=10.0f;
                }
                float err = target_angle - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw34.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw34, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 离开 B 点 60cm 后才探测 D 点
                if (g_Encoder.distance_cm > 60.0f && Is_On_CrossLine()) { 
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 3) { // D -> A 循迹
                base_speed = 13.0f;//8
                turn_out = PID_Calc_Positional(&pid_line34, Sensor_Get_Error());
                float current_err = Sensor_Get_Error();
                // 当误差极大 (>= 20) 时，说明正在斜线入弯，或者触发了丢线记忆
                if (current_err >= 20.0f || current_err <= -20.0f) {
                    pid_line34.Kp = 2.5f;       // 极其暴力的 P，往死里拽
                    pid_line34.out_max = 16.0f; // 放开限幅！允许输出 16
                    pid_line34.out_min = -16.0f;
                }
                    // 当误差较小 (< 20) 时，说明车头已经顺滑地贴在黑线上了
                else {
                    pid_line34.Kp = 0.6f;       // 恢复温柔的 P，防止画龙
                    pid_line34.out_max = 6.0f;  // 收紧限幅，让它乖乖巡航
                    pid_line34.out_min = -6.0f;
                }
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 回到起点 A：里程 > 60cm 且角度回正
                if (g_Encoder.distance_cm > 60.0f || absFloat(mpu6050.Yaw) <= 8.0f) { 
                    Trigger_Feedback(); 
                    Lap_Counter++; // 记录圈数
                    
                    if (Lap_Counter >= 4) { // 跑完4圈，结束任务
                        Car_Mode = TASK_FINISHED; 
                        Lap_Counter = 0; 
                    } else { // 还没跑完，进入下一圈
                        Current_Step = 0; 
                        Reset_Encoder_Distance(); 
                        
                        // 【关键修改】：因为对角线依赖绝对角度 (38.7 和 141.3)
                        // 跑完一圈后陀螺仪一定有漂移误差。必须在这里将当前车头朝向重新归零！
                        // extern void Yaw_Reset(void); 
                        // Yaw_Reset(); 
                    }
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            break;

        case TASK_3_ACBD_DIAGONAL:
            if (Current_Step == 0) { // A -> C 对角线
                base_speed = 10.0f; 
                float target_angle = 38.7f;
                if (g_Encoder.distance_cm > 60.0f) {
                    target_angle=20.0f;
                    base_speed=10.0f;
                }
                float err = target_angle - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw34.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw34, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 离开 A 点 30cm 后才探测 C 点
                if (g_Encoder.distance_cm > 60.0f && Is_On_CrossLine()) { 
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 1) { // C -> B 循迹
                base_speed = 8.0f;
                float current_err = Sensor_Get_Error();
                // 当误差极大 (>= 20) 时，说明正在斜线入弯，或者触发了丢线记忆
                if (current_err >= 20.0f || current_err <= -20.0f) {
                    pid_line34.Kp = 2.5f;       // 极其暴力的 P，往死里拽
                    pid_line34.out_max = 16.0f; // 放开限幅！允许输出 16
                    pid_line34.out_min = -16.0f;
                }
                    // 当误差较小 (< 20) 时，说明车头已经顺滑地贴在黑线上了
                else {
                    pid_line34.Kp = 1.0f;       // 恢复温柔的 P，防止画龙
                    pid_line34.out_max = 6.0f;  // 收紧限幅，让它乖乖巡航
                    pid_line34.out_min = -6.0f;
                }
                turn_out = PID_Calc_Positional(&pid_line34, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm > 57.0f && absFloat(mpu6050.Yaw) >= 141.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 2) { // B -> D 对角线
                base_speed = 10.0f; 
                float target_angle = 141.3f;
                if (g_Encoder.distance_cm > 60.0f) {
                    target_angle=160.0f;
                    base_speed=10.0f;
                }
                float err = target_angle - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw34.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw34, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                // 离开 B 点 60cm 后才探测 D 点
                if (g_Encoder.distance_cm > 60.0f && Is_On_CrossLine()) { 
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            else if (Current_Step == 3) { // D -> A 循迹
                base_speed = 8.0f;
                float current_err = Sensor_Get_Error();
                // 当误差极大 (>= 20) 时，说明正在斜线入弯，或者触发了丢线记忆
                if (current_err >= 20.0f || current_err <= -20.0f) {
                    pid_line34.Kp = 2.5f;       // 极其暴力的 P，往死里拽
                    pid_line34.out_max = 16.0f; // 放开限幅！允许输出 16
                    pid_line34.out_min = -16.0f;
                }
                    // 当误差较小 (< 20) 时，说明车头已经顺滑地贴在黑线上了
                else {
                    pid_line34.Kp = 1.0f;       // 恢复温柔的 P，防止画龙
                    pid_line34.out_max = 6.0f;  // 收紧限幅，让它乖乖巡航
                    pid_line34.out_min = -6.0f;
                }
                turn_out = PID_Calc_Positional(&pid_line34, current_err);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm > 60.0f && absFloat(mpu6050.Yaw) <= 30.0f) { 
                    Trigger_Feedback(); Car_Mode = TASK_FINISHED; 
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            break;

        case TASK_FINISHED:
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            if (Feedback_Timer == 0) {
                // 只在第一次进入TASK_FINISHED时触发声光提示
               
            }
            break;
    }

    final_L_pwm = (int16_t)PID_Calc_Positional(&pid_speed_L, filtered_L);
    final_R_pwm = (int16_t)PID_Calc_Positional(&pid_speed_R, filtered_R);

    Set_Motor_Speed_Left(final_L_pwm);
    Set_Motor_Speed_Right(final_R_pwm);
}

void Vofa_Send_Debug(void)
{
    float data[6];
    data[0] = 0.0F; 
    data[1] = Sensor_Get_Error();        
    data[2] = pid_line34.output;
    data[3] = filtered_L;          
    data[4] = filtered_R;         
    data[5] = mpu6050.Yaw;    

    for(int i=0; i<6; i++) {
        uint8_t *p = (uint8_t *)&data[i];
        for(int j=0; j<4; j++) {
            DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[j]);
        }
    }
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    for(int i=0; i<4; i++) DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, tail[i]);
}

void Control_Reset(void)
{
    PID_Clear(&pid_line12);
    PID_Clear(&pid_line34);
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_yaw34);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
    Set_Motor_Speed_Left(0);
    Set_Motor_Speed_Right(0);
    Current_Step = 0;
    Reset_Encoder_Distance();
}