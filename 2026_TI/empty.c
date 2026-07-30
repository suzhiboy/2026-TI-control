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
#include "key_menu.h"
#include "delay.h"
#include <stdio.h>

static volatile uint32_t control_ticks_10ms = 0;
static BalanceControl_t bc;
static volatile VisionBallData g_vision_ball = {0};

/* 循迹一圈参数 (赛道 ~6.14m, 留 2% 余量确保过线后停) */
#define TRACK_LAP_DISTANCE_CM        (614.0f)
#define TRACK_LAP_STOP_MARGIN_CM     (12.0f)    /* 超量后多走 12cm 防止提前停 */
static float g_lap_start_distance = 0.0f;
static bool  g_lap_completed = false;

/* 丢球超时阈值 (50 ticks × 10ms = 500ms 未收到有效数据触发) */
#define VISION_LOST_TIMEOUT_TICKS  50U

static MT6701_Data mt6701;
static MT6701_Status mt6701_status = MT6701_ERR_TIMEOUT;

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
    KeyMenu_Init();

    LineTrack_Start(LineTrack_Get_BaseSpeed());
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, BC_PWM_CENTER_US,
                                    DL_TIMER_CC_1_INDEX);
    DL_Timer_startCounter(PD42S1_PWM_INST);

    /* ===== PD42S1 归位演示 (控制 ISR 尚未启动, 不会覆盖 PWM) ===== */
    pd42s1_homing_demo();

    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ========== 初始 IDLE, 等待菜单系统启动任务 ========== */
    ControlState_Set(CONTROL_IDLE);
    g_lap_start_distance = -1.0f;   /* 负值表示未记录起始里程 */
    g_lap_completed = false;

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

        /* ---- 运行当前任务 (按键菜单 RUNNING 态) ---- */
        if (KeyMenu_GetState() == SYS_RUNNING) {
            const TaskDef *task = KeyMenu_GetCurrentTask();
            if (task && task->run) {
                task->run();
            }
        }

        /* ========== 循迹一圈完成检测 ========== */
        if (g_control_state == CONTROL_DYNAMIC_BALL) {
            /* 首次进入 DYNAMIC_BALL 时记录起始里程 */
            if (g_lap_start_distance < 0.0f) {
                g_lap_start_distance = g_Encoder.distance_cm;
                g_lap_completed = false;
            }
            if (!g_lap_completed) {
                float traveled = g_Encoder.distance_cm - g_lap_start_distance;
                if (traveled >= (TRACK_LAP_DISTANCE_CM + TRACK_LAP_STOP_MARGIN_CM)) {
                    g_lap_completed = true;
                    ControlState_Set(CONTROL_IDLE);
                }
            }
        } else {
            /* 离开 DYNAMIC_BALL → 重置, 下次进入重新计数 */
            g_lap_start_distance = -1.0f;
            g_lap_completed = false;
        }

        now = control_ticks_10ms;
        if ((uint32_t)(now - last_vofa_tick) >= 2U) {
            last_vofa_tick = now;
            Vofa_SendTelemetry();
        }

        if ((uint32_t)(now - last_oled_tick) >= 10U) {
            last_oled_tick = now;

            /* ---- 菜单信息 (Line 1~3) ---- */
            KeyMenu_OLED();

            /* ---- Line 4: 小球 / 传感器 ---- */
            mt6701_status = MT6701_Update(&mt6701);
            if (g_vision_ball.valid) {
                sprintf(oled_str, "Ball:%dmm C%u", g_vision_ball.x_mm,
                    (unsigned)g_vision_ball.conf_percent);
            } else {
                sprintf(oled_str, "ball:LOST");
            }
            OLED_ShowLineString(4, 1, oled_str);
            OLED_Refresh();
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO: {
            static uint32_t lost_ticks = 0;
            bool need_balance = false;

            /* ---- 按键扫描 (始终运行) ---- */
            KeyMenu_Scan();

            /* ========== 控制算法调度 ========== */
            switch (g_control_state) {

                case CONTROL_IDLE:
                    LineTrack_Stop();
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case CONTROL_TRACK_ONLY:
                    LineTrack_Loop_10ms();
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case CONTROL_STATIC_BALL:
                    LineTrack_Stop();
                    need_balance = true;
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
