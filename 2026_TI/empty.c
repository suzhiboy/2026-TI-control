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
#include "key_menu.h"
#include "t3_task.h"
#include <stdio.h>

#ifndef APP_AUTO_START_T3
#define APP_AUTO_START_T3  1
#endif

static volatile uint32_t control_ticks_10ms = 0;
static BalanceControl_t bc;
static volatile VisionBallData g_vision_ball = {0};
static bool g_pd42s1_pwm_running = false;
static MT6701_Data mt6701;
static MT6701_Status mt6701_status = MT6701_ERR_TIMEOUT;

static uint16_t slew_u16_toward(uint16_t current, uint16_t target, uint16_t step)
{
    if (current < target) {
        uint16_t next = (uint16_t)(current + step);
        return (next < target) ? next : target;
    }

    if (current > target) {
        uint16_t next = (current > step) ? (uint16_t)(current - step) : 0U;
        return (next > target) ? next : target;
    }

    return current;
}

static void PD42S1_WritePulse(uint16_t pulse)
{
    DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, pulse, DL_TIMER_CC_1_INDEX);
    if (!g_pd42s1_pwm_running) {
        DL_Timer_startCounter(PD42S1_PWM_INST);
        g_pd42s1_pwm_running = true;
    }
}

static void PD42S1_LockCenter(void)
{
    bc.pwm_pulse = (uint16_t)(bc.pwm_neutral + 0.5f);
    PD42S1_WritePulse(bc.pwm_pulse);
}

static void PD42S1_SoftLockCenter(void)
{
    uint16_t previous_pulse = bc.pwm_pulse;
    uint16_t neutral_pulse = (uint16_t)(bc.pwm_neutral + 0.5f);

    BalanceControl_Reset(&bc);
    bc.pwm_pulse = previous_pulse;
    bc.pwm_pulse = slew_u16_toward(bc.pwm_pulse, neutral_pulse,
                                   BC_PWM_SLEW_LIMIT_US);
    PD42S1_WritePulse(bc.pwm_pulse);
}

int main(void)
{
    char oled_str[50];
    uint32_t last_vofa_tick = 0;
    uint32_t last_oled_tick = 0;
    uint32_t last_task_run_tick = 0;

    SYSCFG_DL_init();

    BalanceControl_Init(&bc);
    BalanceControl_SetReference(&bc, 0.0f);
    T3Task_AttachController(&bc);

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

    ControlState_Set(CONTROL_IDLE);
    PD42S1_LockCenter();
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

#if APP_AUTO_START_T3
    KeyMenu_StartTask(TASK_T3);
#endif

    while (1) {
        uint32_t now;

        IMU601_poll();
        Vofa_Poll();
        VisionUart_Poll(control_ticks_10ms);

        g_vision_ball = VisionUart_GetLatest();
        T3Task_UpdateVision(g_vision_ball.valid, g_vision_ball.x_mm);
        if (g_vision_ball.valid) {
            BalanceControl_SetRawPosition(&bc,
                (float)g_vision_ball.x_mm / 10.0f);
        }

        now = control_ticks_10ms;
        if ((KeyMenu_GetState() == SYS_RUNNING) && (now != last_task_run_tick)) {
            last_task_run_tick = now;
            const TaskDef *task = KeyMenu_GetCurrentTask();
            if (task && task->run) {
                task->run();
            }
        }

        if ((uint32_t)(now - last_vofa_tick) >= 2U) {
            last_vofa_tick = now;
            Vofa_SendTelemetry();
        }

        if ((uint32_t)(now - last_oled_tick) >= 10U) {
            last_oled_tick = now;

            KeyMenu_OLED();
            if (T3Task_IsActive()) {
                int pwm_delta = (int)bc.pwm_pulse - (int)BC_PWM_CENTER_US;
                if (g_vision_ball.valid) {
                    sprintf(oled_str, "T%+d X%+d P%+d",
                        (int)T3Task_GetTargetMM(),
                        (int)g_vision_ball.x_mm,
                        pwm_delta);
                } else if (g_vision_ball.timed_out) {
                    sprintf(oled_str, "T%+d TO P%+d",
                        (int)T3Task_GetTargetMM(),
                        pwm_delta);
                } else {
                    sprintf(oled_str, "T%+d L P%+d",
                        (int)T3Task_GetTargetMM(),
                        pwm_delta);
                }
            } else if (g_vision_ball.valid) {
                sprintf(oled_str, "Ball:%dmm C%u", g_vision_ball.x_mm,
                    (unsigned)g_vision_ball.conf_percent);
            } else if (g_vision_ball.timed_out) {
                sprintf(oled_str, "ball:TIMEOUT");
            } else {
                mt6701_status = MT6701_Update(&mt6701);
                sprintf(oled_str, "LOST S%u",
                    (unsigned)g_vision_ball.seq);
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
            bool need_balance = false;

            KeyMenu_Scan();

            switch (g_control_state) {
                case CONTROL_IDLE:
                    LineTrack_Stop();
                    PD42S1_SoftLockCenter();
                    break;

                case CONTROL_TRACK_ONLY:
                    LineTrack_Loop_10ms();
                    PD42S1_SoftLockCenter();
                    break;

                case CONTROL_STATIC_BALL:
                    LineTrack_Stop();
                    need_balance = true;
                    break;

                case CONTROL_DYNAMIC_BALL:
                    LineTrack_Loop_10ms();
                    need_balance = true;
                    break;

                default:
                    LineTrack_Stop();
                    PD42S1_SoftLockCenter();
                    break;
            }

            if (need_balance) {
                if (g_vision_ball.valid) {
                    BalanceControl_Run(&bc);
                    if (!g_pd42s1_pwm_running) {
                        DL_Timer_startCounter(PD42S1_PWM_INST);
                        g_pd42s1_pwm_running = true;
                    }
                } else {
                    PD42S1_SoftLockCenter();
                }
            }

            if (g_pd42s1_pwm_running) {
                DL_Timer_setCaptureCompareValue(PD42S1_PWM_INST, bc.pwm_pulse,
                                                DL_TIMER_CC_1_INDEX);
            }
            control_ticks_10ms++;
            break;
        }
        default:
            break;
    }
}
