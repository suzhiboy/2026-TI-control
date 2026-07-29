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

/* PD42S1 步进电机 UART 写回调 (发送 F5H 指令使能 PWM 位置模式) */
static void pd42s1_uart_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        DL_UART_Main_transmitData(PD42S1_UART_INST, data[i]);
    }
}
static MT6701_Data mt6701;
static MT6701_Status mt6701_status = MT6701_ERR_TIMEOUT;

int main(void)
{
    char oled_str[50];
    uint32_t last_vofa_tick = 0;
    uint32_t last_oled_tick = 0;

    SYSCFG_DL_init();

    BalanceControl_Init(&bc);
    BalanceControl_PD42S1_Init(pd42s1_uart_write);
    BalanceControl_SetReference(&bc, 0.0f);

    /* 配置 PD42S1 PWM 输出: PA17 作为 TIMA1_CC0 */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM39,
                                         IOMUX_PINCM39_PF_TIMA1_CCP0);
    DL_Timer_setCCPDirection(PD42S1_PWM_INST, DL_TIMER_CC0_OUTPUT);
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, 1500U,
                                    DL_TIMER_CC_0_INDEX);

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
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, 1500U,
                                    DL_TIMER_CC_0_INDEX);
    DL_Timer_startCounter(PD42S1_PWM_INST);
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ========== 启动: 进入循迹 + 调球模式, 记录起始里程 ========== */
    SysState_Set(STATE_DYNAMIC_BALL);
    g_lap_start_distance = g_Encoder.distance_cm;
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

        /* ========== 循迹一圈完成检测 ========== */
        if (g_sys_state == STATE_DYNAMIC_BALL && !g_lap_completed) {
            float traveled = g_Encoder.distance_cm - g_lap_start_distance;
            if (traveled >= (TRACK_LAP_DISTANCE_CM + TRACK_LAP_STOP_MARGIN_CM)) {
                g_lap_completed = true;
                SysState_Set(STATE_IDLE);
            }
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

            /* ---- Line 4: 小球 / 传感器保留 ---- */
            sprintf(oled_str, "Yaw: %.2f", current_attitude.yaw);
            OLED_ShowLineString(1, 1, oled_str);
            sprintf(oled_str, "Pitch: %.2f", current_attitude.pitch);
            OLED_ShowLineString(2, 1, oled_str);
            sprintf(oled_str, "Roll: %.2f", current_attitude.roll);
            OLED_ShowLineString(3, 1, oled_str);
            mt6701_status = MT6701_Update(&mt6701);
            if (mt6701_status == MT6701_OK) {
                sprintf(oled_str, "MT:%.2f A%02X", mt6701.angle_deg,
                    MT6701_GetActiveAddress());
            } else {
                sprintf(oled_str, "MT:E%u A%02X", (unsigned)mt6701_status,
                    MT6701_GetActiveAddress());
            }
            if (g_vision_ball.valid) {
                sprintf(oled_str, "Ball:%dmm C%u", g_vision_ball.x_mm,
                    (unsigned)g_vision_ball.conf_percent);
            } else {
                sprintf(oled_str, "Ball:LOST");
            } else if (mt6701_status == MT6701_OK) {
                sprintf(oled_str, "MT:%.2f A%02X", mt6701.angle_deg,
                    MT6701_GetActiveAddress());
            } else {
                sprintf(oled_str, "MT:E%u A%02X", (unsigned)mt6701_status,
                    MT6701_GetActiveAddress());
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

            /* ========== 状态机调度 ========== */
            switch (g_sys_state) {

                case STATE_IDLE:
                    LineTrack_Stop();
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case STATE_TRACK_ONLY:
                    LineTrack_Loop_10ms();
                    BalanceControl_Reset(&bc);
                    bc.pwm_pulse = 1500U;
                    break;

                case STATE_STATIC_BALL:
                    LineTrack_Stop();
                    need_balance = true;
                    break;

                case STATE_DYNAMIC_BALL:
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
                                            DL_TIMER_CC_0_INDEX);
            control_ticks_10ms++;
            break;
        }
        default:
            break;
    }
}
