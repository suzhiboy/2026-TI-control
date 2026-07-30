/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "line_follow.h"
#include "imu601.h"
#include "oled.h"
#include "vofa.h"
#include "mt6701.h"
#include "vision_uart.h"
#include "board_config.h"
#include "balance_control.h"
#include "sys_state.h"
#include "encoder.h"
#include "sensor.h"
#include "key_menu.h"
#include "delay.h"
#include <stdio.h>

static volatile uint32_t control_ticks_10ms = 0;
static BalanceControl_t bc;
static volatile VisionBallData g_vision_ball = {0};

/* 循迹一圈参数 — 参考值 (赛道 ~6.14m)
 * 注意: T2 现已改用灰度传感器终点线检测, 不再依赖固定里程阈值,
 *       以下常量仅保留作为赛道长度参考. */
#define TRACK_LAP_DISTANCE_CM        (614.0f)
#define TRACK_LAP_STOP_DECEL_CM      (30.0f)    /* 最后 30cm 开始减速 (旧逻辑) */
static float g_lap_start_distance = 0.0f;
static bool  g_lap_completed = false;
static bool  g_lap_decelerating = false;
static uint32_t g_lap_time_seconds = 0;         /* T2 计时 (秒) */
static bool  g_timer_running = false;

/* IMU 偏航角辅助判据 (防止里程计打滑误判) */
#define TRACK_LAP_YAW_THRESHOLD      (300.0f)   /* 累计偏航 ≥300° 才判一圈完成 */
#define TRACK_LAP_YAW_DECEL_THRESHOLD (240.0f)  /* 累计偏航 ≥240° 才允许进入减速区 */
static float g_start_yaw = 0.0f;                /* 起始航向角 */
static float g_last_yaw = 0.0f;                 /* 上一帧航向角 (用于计算增量) */
static float g_yaw_accumulated = 0.0f;          /* 累计偏航绝对值 (度) */

/* ======================================================================== *
 *  T3 静态球控轨迹规划 (静止, 0 → +5cm → -5cm, 限时 5s)
 * ======================================================================== */
static uint32_t task3_start_tick = 0;   /* 起始 control_ticks_10ms 值      */
static uint8_t  task3_step = 0;         /* 阶段: 0=0→+5, 1=hold+5,         */
                                        /*       2=+5→-5, 3=hold-5        */
/* 丢球超时阈值 (50 ticks × 10ms = 500ms 未收到有效数据触发) */
#define VISION_LOST_TIMEOUT_TICKS  50U

/* ======================================================================== *
 *  T4/T5/T6 共用参数
 *
 *  停车策略: 编码器里程开窗 + 灰度传感器检测横线 (双重确认)
 *    开窗后如果 ≥5 路探头同时检测到黑线, 判为终点横线.
 * ======================================================================== */
#define TASK4_STOP_WINDOW_CM    (120.0f)  /* A→B 1.5m, 1.2m后开启冲线检测    */
#define LAP_STOP_WINDOW_CM      (550.0f)  /* 一圈6.14m, 5.5m后开启冲线检测   */
#define LINE_DETECT_THRESHOLD   (5U)      /* ≥5探头同时检测黑线 = 横线       */
#define TASK4_TARGET_SPEED      (25.0f)   /* T4 直道目标速度                 */
#define LAP_TARGET_SPEED        (25.0f)   /* T5/T6 一圈目标速度              */

/* T2 一圈停车: 编码器/偏航开窗 + 灰度终点线检测 (双重确认)         *
 * 开窗后如果 ≥5 路探头同时检测到黑线, 判为终点横线立即刹车.          *
 * 里程和偏航是 "或" 关系, 任一开窗即可启动灰度检测,                  *
 * 防止编码器校准偏差或 IMU 漂移导致跑多圈或停不下来.                */
#define T2_STOP_WINDOW_CM       (500.0f)  /* 500cm后开启终点线灰度检测       */
static float t2_start_dist  = 0.0f;       /* T2 启动时的编码器初始里程        */
static bool  t2_finish_line = false;      /* T2 终点横线已检测标志            */

float             run_time_s = 0.0f;          /* 秒表, 每 10ms += 0.01s       */
static bool       finish_line_detected = false;/* 已检测到终点横线             */

static MT6701_Data mt6701;

/* ======================================================================== *
 *  PD42S1 平滑斜坡辅助函数 & 归位演示
 *
 *  映射计算:
 *    PD42S1 500~2500 µs → 0~25600 counts (线性)
 *    1 µs = 12.8 counts, 1 rev = 25600 counts = 360°
 *    30° = 30/360 × 25600 = 2133.3 counts → 166 µs
 *
 *  平滑原理:
 *    脉宽每次增减 1 µs, 配合间隔延时 → 电机位置连续微调, 无阶跃冲击
 * ======================================================================== */

/**
 * @brief  平滑地将 PWM 脉宽从 from_pulse 过渡到 to_pulse
 * @param  from_pulse  起始脉宽 (µs)
 * @param  to_pulse    目标脉宽 (µs)
 * @param  duration_ms 过渡总时长 (ms)
 */
static void pd42s1_smooth_move(uint16_t from_pulse, uint16_t to_pulse,
                                uint32_t duration_ms)
{
    if (from_pulse == to_pulse) return;

    uint32_t steps;
    int32_t step_dir;
    if (to_pulse > from_pulse) {
        steps    = (uint32_t)(to_pulse - from_pulse);
        step_dir = 1;
    } else {
        steps    = (uint32_t)(from_pulse - to_pulse);
        step_dir = -1;
    }

    /* 每步延时 = 总时长 ÷ 步数, 至少 1ms */
    uint32_t delay_per_step = (steps > 0) ? (duration_ms / steps) : 1;
    if (delay_per_step < 1)  delay_per_step = 1;

    uint16_t pulse = from_pulse;
    for (uint32_t i = 0; i < steps; i++) {
        pulse += (int16_t)step_dir;
        DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, pulse,
                                        DL_TIMER_CC_1_INDEX);
        delay_ms(delay_per_step);
    }
    /* 确保最终值精确到位 */
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, to_pulse,
                                    DL_TIMER_CC_1_INDEX);
}

static void pd42s1_homing_demo(void)
{
    char str[32];

    /* 30° 对应脉宽偏移量 = 166 µs */
    const uint16_t OFFSET_30D = 166U;
    const uint16_t PULSE_CW  = BC_PWM_CENTER_US + OFFSET_30D;   /* 1666 µs */
    const uint16_t PULSE_CCW = BC_PWM_CENTER_US - OFFSET_30D;   /* 1334 µs */
    const uint32_t RAMP_MS   = 1200;   /* 每段斜坡时长 1.2 秒, 约 7 µs/步进 */

    OLED_Clear();
    OLED_ShowLineString(1, 1, "PD42S1 Homing");
    OLED_Refresh();
    delay_ms(500);

    /* ---- Step 1: 顺时针 30° (平滑斜坡) ---- */
    snprintf(str, sizeof(str), "CW +30  %uus", (unsigned)PULSE_CW);
    OLED_ShowLineString(2, 1, str);
    OLED_Refresh();
    pd42s1_smooth_move(BC_PWM_CENTER_US, PULSE_CW, RAMP_MS);
    delay_ms(800);

    /* ---- Step 2: 归位 - 中心 (平滑斜坡) ---- */
    OLED_ShowLineString(2, 1, "Center  1500us");
    OLED_Refresh();
    pd42s1_smooth_move(PULSE_CW, BC_PWM_CENTER_US, RAMP_MS);
    delay_ms(800);

    /* ---- Step 3: 逆时针 30° (平滑斜坡) ---- */
    snprintf(str, sizeof(str), "CCW -30  %uus", (unsigned)PULSE_CCW);
    OLED_ShowLineString(2, 1, str);
    OLED_Refresh();
    pd42s1_smooth_move(BC_PWM_CENTER_US, PULSE_CCW, RAMP_MS);
    delay_ms(800);

    /* ---- Step 4: 归位 - 中心 (平滑斜坡) ---- */
    OLED_ShowLineString(2, 1, "Center  1500us");
    OLED_Refresh();
    pd42s1_smooth_move(PULSE_CCW, BC_PWM_CENTER_US, RAMP_MS);
    delay_ms(500);

    OLED_ShowLineString(3, 1, "Demo Done!");
    OLED_Refresh();
    delay_ms(500);
}

/* ======================================================================== *
 *  T3 静态球控轨迹规划
 *
 *  最小加加速度 (Minimum Jerk) 轨迹:
 *    s(τ) = 10τ³ - 15τ⁴ + 6τ⁵,  τ = t/T ∈ [0,1]
 *    位置、速度、加速度全程连续, 无阶跃冲击.
 *
 *  时序:
 *    Phase 0:   0 ms  ~ 2000 ms   0 → +5  cm
 *    Phase 1: 2000 ms ~ 2500 ms   hold +5 cm
 *    Phase 2: 2500 ms ~ 4500 ms   +5 → -5 cm
 *    Phase 3: 4500 ms ~ 5000 ms+  hold -5 cm
 * ======================================================================== */

/**
 * @brief  最小加加速度插值
 * @param  t   当前时间 (s)
 * @param  T   阶段总时长 (s)
 * @param  x0  起始位置 (cm)
 * @param  x1  目标位置 (cm)
 * @return  平滑插值位置
 */
static float min_jerk(float t, float T, float x0, float x1)
{
    float tau;
    float tau2, tau3;
    float s;

    if (t >= T) return x1;
    if (t <= 0.0f) return x0;

    tau  = t / T;
    tau2 = tau * tau;
    tau3 = tau2 * tau;
    /* s = 10τ³ - 15τ⁴ + 6τ⁵ */
    s = 10.0f * tau3 - 15.0f * tau2 * tau2 + 6.0f * tau3 * tau2;
    return x0 + (x1 - x0) * s;
}

/**
 * @brief  初始化 T3 轨迹 (由 key_menu T3_Init 调用)
 */
void Task3_InitTrajectory(void)
{
    task3_start_tick = control_ticks_10ms;
    task3_step = 0;
    BalanceControl_SetReference(&bc, 0.0f);
    ControlState_Set(CONTROL_TASK3);
}

/**
 * @brief  每 10ms 更新 T3 轨迹参考值 (由 key_menu T3_Run 调用)
 */
void Task3_UpdateTrajectory(void)
{
    uint32_t elapsed_ticks = control_ticks_10ms - task3_start_tick;
    uint32_t t_ms = elapsed_ticks * 10U;   /* 每 tick = 10ms */

    switch (task3_step) {

        case 0:     /* Phase 0: 0 → +5 cm, 0~2000 ms */
            if (t_ms >= 2000U) {
                BalanceControl_SetReference(&bc, 5.0f);
                task3_step = 1;
            } else {
                float ref = min_jerk((float)t_ms / 1000.0f, 2.0f, 0.0f, 5.0f);
                BalanceControl_SetReference(&bc, ref);
            }
            break;

        case 1:     /* Phase 1: hold at +5 cm, 2000~2500 ms */
            BalanceControl_SetReference(&bc, 5.0f);
            if (t_ms >= 2500U) {
                task3_step = 2;
            }
            break;

        case 2:     /* Phase 2: +5 → -5 cm, 2500~4500 ms */
            if (t_ms >= 4500U) {
                BalanceControl_SetReference(&bc, -5.0f);
                task3_step = 3;
            } else {
                float t_local = ((float)t_ms - 2500.0f) / 1000.0f;
                float ref = min_jerk(t_local, 2.0f, 5.0f, -5.0f);
                BalanceControl_SetReference(&bc, ref);
            }
            break;

        case 3:     /* Phase 3: hold at -5 cm, 4500 ms+ */
            BalanceControl_SetReference(&bc, -5.0f);
            break;

        default:
            break;
    }
}

/* ======================================================================== *
 *  T2 一圈停车初始化 (由 key_menu T2_Init 调用)
 * ======================================================================== */

void Task2_Init(void)
{
    t2_start_dist  = g_Encoder.distance_cm;
    t2_finish_line = false;
    g_lap_completed     = false;
    g_lap_decelerating  = false;
    g_timer_running     = false;
    g_yaw_accumulated   = 0.0f;
    g_last_yaw          = current_attitude.yaw;
    g_start_yaw         = current_attitude.yaw;
    run_time_s          = 0.0f;
    ControlState_Set(CONTROL_TRACK_ONLY);
    LineTrack_Start(LineTrack_Get_BaseSpeed());
}

/* ======================================================================== *
 *  T4/T5/T6 共用辅助函数
 * ======================================================================== */

/**
 * @brief  检测是否压到终点横线 (≥5 路探头同时检测到黑线)
 * @return true = 压线
 */
static bool Check_Finish_Line(void)
{
    uint8_t data[SENSOR_COUNT];
    uint8_t count = 0;

    Sensor_Read_All(data);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (data[i] == 1U) count++;   /* LINE_DETECTED */
    }
    return (count >= LINE_DETECT_THRESHOLD);
}

/**
 * @brief  系统一键清零: 编码器 + 秒表 + 摆杆角度零点
 *         由 READY 态 K1 长按触发
 */
void Task_SystemReset(void)
{
    Encoder_Clear();
    run_time_s = 0.0f;
    BalanceControl_CalibrateCenter(&bc, BC_PWM_CENTER_US);
}

/**
 * @brief  初始化 T4: 小球居中, 跑直道 A→B 停车
 */
void Task4_Init(void)
{
    BalanceControl_SetReference(&bc, 0.0f);
    Encoder_Clear();
    run_time_s = 0.0f;
    finish_line_detected = false;
    LineTrack_Start(TASK4_TARGET_SPEED);
    ControlState_Set(CONTROL_TASK4);
}

/**
 * @brief  初始化 T5: 小球居中, 跑一圈回 A 点停车
 */
void Task5_Init(void)
{
    BalanceControl_SetReference(&bc, 0.0f);
    Encoder_Clear();
    run_time_s = 0.0f;
    finish_line_detected = false;
    LineTrack_Start(LAP_TARGET_SPEED);
    ControlState_Set(CONTROL_TASK5);
}

/**
 * @brief  初始化 T6: 小球在用户设定位置, 跑一圈回 A 点停车
 */
void Task6_Init(void)
{
    float target_cm = user_target_x_cm;
    if (target_cm < -12.0f) target_cm = -12.0f;
    if (target_cm >  12.0f) target_cm =  12.0f;
    BalanceControl_SetReference(&bc, target_cm);
    Encoder_Clear();
    run_time_s = 0.0f;
    finish_line_detected = false;
    LineTrack_Start(LAP_TARGET_SPEED);
    ControlState_Set(CONTROL_TASK6);
}

int main(void)
{
    char oled_str[50];
    uint32_t last_vofa_tick = 0;
    uint32_t last_oled_tick = 0;

    SYSCFG_DL_init();

    BalanceControl_Init(&bc);
    BalanceControl_SetReference(&bc, 0.0f);

    OLED_Init();
    OLED_Clear();
    OLED_ShowLineString(1, 1, "OLED OK");
    OLED_Refresh();
    delay_cycles(STARTUP_SPLASH_DELAY_CYCLES);

    LineTrack_Init();
    IMU601_init();
    MT6701_Init(&mt6701);

    NVIC_SetPriority(GPIO_ENCODER_INT_IRQN, 0);
    NVIC_SetPriority(IMU601_INST_INT_IRQN, 1);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2);
    NVIC_SetPriority(VOFA_INST_INT_IRQN, 3);
    NVIC_SetPriority(VISION_UART_INST_INT_IRQN, 3);
    __enable_irq();

    Vofa_Init();
    VisionUart_Init();

    LineTrack_Start(LineTrack_Get_BaseSpeed());
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, BC_PWM_CENTER_US,
                                    DL_TIMER_CC_1_INDEX);
    DL_Timer_startCounter(PD42S1_PWM_INST);

    /* ===== PD42S1 归位演示 (控制 ISR 尚未启动, 不会覆盖 PWM) ===== */
    pd42s1_homing_demo();

    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ========== 无按键模式: 配置 PB13 为 AD2 后直接启动 T2 一圈停车 ========== */
    DL_GPIO_initDigitalOutput(GPIO_SENSOR_AD2_IOMUX);
    DL_GPIO_enableOutput(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
    DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
    Task2_Init();

    while (1) {
        uint32_t now;

        IMU601_poll();
        Vofa_Poll();
        VisionUart_Poll(control_ticks_10ms);

        /* 高频获取视觉数据, 更新全局变量供中断读取 */
        g_vision_ball = VisionUart_GetLatest();
        if (g_vision_ball.valid) {
            BalanceControl_SetRawPosition(&bc,
                (float)g_vision_ball.x_mm / 10.0f);
        }

        /* ========== T2 一圈停车: 主循环只做偏航跟踪供 OLED 显示 ==========
         *  停车判定已移至 ISR CONTROL_TRACK_ONLY 处理:
         *    编码器开窗 OR 偏航开窗 → 灰度终点线检测 → LineTrack_Brake → IDLE
         * ================================================================== */
        if (g_control_state == CONTROL_TRACK_ONLY) {

            /* 首次进入时记录起始里程/航向 */
            if (g_lap_start_distance < 0.0f) {
                g_lap_start_distance = g_Encoder.distance_cm;
                g_lap_completed = false;
                g_lap_decelerating = false;
                g_lap_time_seconds = 0;
                g_timer_running = true;
                g_start_yaw = current_attitude.yaw;
                g_last_yaw  = current_attitude.yaw;
                g_yaw_accumulated = 0.0f;
            }

            /* IMU 偏航角累计 (仅用于 OLED 显示, 不再参与停车判定) */
            if (!g_lap_completed) {
                float yaw = current_attitude.yaw;
                float delta = yaw - g_last_yaw;
                if (delta > 180.0f)  delta -= 360.0f;
                if (delta < -180.0f) delta += 360.0f;
                if (delta < 0.0f)    delta = -delta;
                g_yaw_accumulated += delta;
                g_last_yaw = yaw;
            }

            /* 同步 ISR 终点线状态到主循环变量 (OLED 显示用) */
            if (t2_finish_line) {
                g_lap_completed = true;
                g_timer_running = false;
            }
        } else {
            /* 离开 CONTROL_TRACK_ONLY → 复位 */
            g_lap_start_distance = -1.0f;
            g_lap_completed = false;
            g_lap_decelerating = false;
            g_timer_running = false;
            g_yaw_accumulated = 0.0f;
        }

        now = control_ticks_10ms;

        if ((uint32_t)(now - last_vofa_tick) >= 2U) {
            last_vofa_tick = now;
            Vofa_SendTelemetry();
        }

        if ((uint32_t)(now - last_oled_tick) >= 10U) {
            last_oled_tick = now;

            /* ---- T2 一圈停车专用 OLED 显示 ---- */
            {
                float traveled = (g_lap_start_distance >= 0.0f)
                    ? (g_Encoder.distance_cm - g_lap_start_distance) : 0.0f;

                /* Line 1: 任务名 */
                sprintf(oled_str, "T2 一圈停车       ");
                OLED_ShowLineString(1, 1, oled_str);

                /* Line 2: 里程 + 偏航 */
                sprintf(oled_str, "D:%.0fcm Y:%.0fdeg",
                    (double)traveled, (double)g_yaw_accumulated);
                OLED_ShowLineString(2, 1, oled_str);

                /* Line 3: 运行时间 */
                sprintf(oled_str, "T:%.1fs            ", (double)run_time_s);
                OLED_ShowLineString(3, 1, oled_str);

                /* Line 4: 运行状态 */
                if (t2_finish_line) {
                    sprintf(oled_str, "> LINE! 刹车中... ");
                } else if (g_control_state == CONTROL_IDLE) {
                    sprintf(oled_str, "> 已停车           ");
                } else {
                    sprintf(oled_str, "> 巡线中...        ");
                }
                OLED_ShowLineString(4, 1, oled_str);
                OLED_Refresh();
            }
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO: {
            static uint32_t lost_ticks = 0;
            bool need_balance = false;

            /* ---- 秒表累加 (非 IDLE 态) ---- */
            if (g_control_state != CONTROL_IDLE) {
                run_time_s += 0.01f;
            }

            /* ========== 控制算法调度 ========== */
            switch (g_control_state) {

                case CONTROL_IDLE:
                    LineTrack_Stop();
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case CONTROL_TRACK_ONLY:
                    if (!t2_finish_line) {
                        LineTrack_Loop_10ms();
                        float traveled = g_Encoder.distance_cm - t2_start_dist;

                        /* 编码器里程开窗 OR 偏航累计开窗 → 启动灰度终点线检测 */
                        if ((traveled >= T2_STOP_WINDOW_CM ||
                             g_yaw_accumulated >= TRACK_LAP_YAW_THRESHOLD) &&
                            Check_Finish_Line()) {
                            t2_finish_line = true;
                            LineTrack_Brake();
                        }
                    } else {
                        /* 刹车完成 → IDLE */
                        if (!LineTrack_IsRunning()) {
                            ControlState_Set(CONTROL_IDLE);
                        }
                    }
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case CONTROL_TASK3:
                case CONTROL_STATIC_BALL:
                    LineTrack_Stop();
                    need_balance = true;
                    break;

                case CONTROL_TASK4:
                    BalanceControl_SetReference(&bc, 0.0f);
                    if (!finish_line_detected) {
                        LineTrack_Loop_10ms();
                        if (g_Encoder.distance_cm >= TASK4_STOP_WINDOW_CM &&
                            Check_Finish_Line()) {
                            finish_line_detected = true;
                            LineTrack_Stop();
                            ControlState_Set(CONTROL_IDLE);
                        }
                    }
                    need_balance = false;  /* 停稳后由 IDLE 处理平衡复位 */
                    break;

                case CONTROL_TASK5:
                    BalanceControl_SetReference(&bc, 0.0f);
                    if (!finish_line_detected) {
                        LineTrack_Loop_10ms();
                        if (g_Encoder.distance_cm >= LAP_STOP_WINDOW_CM &&
                            Check_Finish_Line()) {
                            finish_line_detected = true;
                            LineTrack_Stop();
                            ControlState_Set(CONTROL_IDLE);
                        }
                    }
                    need_balance = false;
                    break;

                case CONTROL_TASK6:
                    BalanceControl_SetReference(&bc, user_target_x_cm);
                    if (!finish_line_detected) {
                        LineTrack_Loop_10ms();
                        if (g_Encoder.distance_cm >= LAP_STOP_WINDOW_CM &&
                            Check_Finish_Line()) {
                            finish_line_detected = true;
                            LineTrack_Stop();
                            ControlState_Set(CONTROL_IDLE);
                        }
                    }
                    need_balance = false;
                    break;

                case CONTROL_DYNAMIC_BALL:
                    LineTrack_Loop_10ms();
                    need_balance = true;
                    break;
            }

            if (need_balance) {
                /* 安全降级: 丢球超时则清除积分并回平 */
                if (g_vision_ball.valid) {
                    lost_ticks = 0;
                    BalanceControl_Run(&bc);
                } else {
                    if (lost_ticks < VISION_LOST_TIMEOUT_TICKS) {
                        lost_ticks++;
                        BalanceControl_Run(&bc);
                    } else {
                        BalanceControl_Reset(&bc);
                        bc.pwm_pulse = 1500U;
                    }
                }
            }

            DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, bc.pwm_pulse,
                                            DL_TIMER_CC_1_INDEX);
            control_ticks_10ms++;
            break;
        }
        default:
            break;
    }
}
