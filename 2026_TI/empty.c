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

#ifndef APP_AUTO_START_TASK
#define APP_AUTO_START_TASK  TASK_T2
#endif

#define APP_OLED_PERIOD_TICKS      (25U)
#define T4_CAR_ACCEL_FF_GAIN       (1.0f)
#define T4_CAR_ACCEL_FF_LIMIT_MS2  (0.85f)
#define T4_CAR_ACCEL_HOLD_MS2      (2.5f)
#define T4_CAR_ACCEL_START_HOLD_TICKS (150U)
#define T4_CAR_ACCEL_STOP_HOLD_TICKS  (210U)
#define T4_CAR_ACCEL_EDGE_DEADBAND_MS2 (0.05f)
#define T5_CAR_ACCEL_FF_GAIN       (2.0f)
#define T5_CAR_ACCEL_FF_SIGN       (1.0f)
#define T5_CAR_ACCEL_FF_LIMIT_MS2  (1.20f)
#define T5_CAR_ACCEL_HOLD_MS2      (1.10f)
#define T5_CAR_ACCEL_START_HOLD_TICKS (180U)
#define T5_CAR_ACCEL_STOP_HOLD_TICKS  (240U)
#define T5_CAR_ACCEL_EDGE_DEADBAND_MS2 (0.04f)
#define T5_CURVE_ACCEL_FF_GAIN     (0.06f)
#define T5_CURVE_ACCEL_FF_LIMIT_MS2 (0.45f)

static volatile uint32_t control_ticks_10ms = 0;
static BalanceControl_t bc;
static volatile VisionBallData g_vision_ball = {0};
static bool g_pd42s1_pwm_running = false;
static uint16_t last_vision_seq = 0U;
static uint32_t last_vision_tick = 0U;
static bool has_vision_seq = false;
static bool t4_ff_session_active = false;
static uint16_t t4_start_hold_ticks = 0U;
static uint16_t t4_stop_hold_ticks = 0U;
static float t4_prev_line_accel_ms2 = 0.0f;
static bool t5_ff_session_active = false;
static uint16_t t5_start_hold_ticks = 0U;
static uint16_t t5_stop_hold_ticks = 0U;
static float t5_prev_line_accel_ms2 = 0.0f;
static MT6701_Data mt6701;

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

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void T4_ResetCarAccelFeedforward(void)
{
    t4_ff_session_active = false;
    t4_start_hold_ticks = 0U;
    t4_stop_hold_ticks = 0U;
    t4_prev_line_accel_ms2 = 0.0f;
}

static float T4_BuildCarAccelFeedforward(void)
{
    float line_accel_ms2 = LineTrack_GetLongitudinalAccelMS2();
    float accel_ms2;

    if (KeyMenu_GetTaskID() != TASK_T4) {
        T4_ResetCarAccelFeedforward();
        return 0.0f;
    }

    if (!t4_ff_session_active) {
        t4_ff_session_active = true;
        t4_start_hold_ticks = T4_CAR_ACCEL_START_HOLD_TICKS;
        t4_stop_hold_ticks = 0U;
        t4_prev_line_accel_ms2 = 0.0f;
    }

    if ((line_accel_ms2 > T4_CAR_ACCEL_EDGE_DEADBAND_MS2) &&
        (t4_prev_line_accel_ms2 <= T4_CAR_ACCEL_EDGE_DEADBAND_MS2)) {
        t4_start_hold_ticks = T4_CAR_ACCEL_START_HOLD_TICKS;
    }
    if ((line_accel_ms2 < -T4_CAR_ACCEL_EDGE_DEADBAND_MS2) &&
        (t4_prev_line_accel_ms2 >= -T4_CAR_ACCEL_EDGE_DEADBAND_MS2)) {
        t4_stop_hold_ticks = T4_CAR_ACCEL_STOP_HOLD_TICKS;
    }

    t4_prev_line_accel_ms2 = line_accel_ms2;

    accel_ms2 = line_accel_ms2 * T4_CAR_ACCEL_FF_GAIN;
    if (t4_start_hold_ticks > 0U) {
        accel_ms2 += T4_CAR_ACCEL_HOLD_MS2;
        t4_start_hold_ticks--;
    }
    if (t4_stop_hold_ticks > 0U) {
        accel_ms2 -= T4_CAR_ACCEL_HOLD_MS2;
        t4_stop_hold_ticks--;
    }

    return clamp_float(accel_ms2,
                       -T4_CAR_ACCEL_FF_LIMIT_MS2,
                       T4_CAR_ACCEL_FF_LIMIT_MS2);
}

static void T5_ResetCarAccelFeedforward(void)
{
    t5_ff_session_active = false;
    t5_start_hold_ticks = 0U;
    t5_stop_hold_ticks = 0U;
    t5_prev_line_accel_ms2 = 0.0f;
}

static float T5_BuildCarAccelFeedforward(void)
{
    float line_accel_ms2 = LineTrack_GetLongitudinalAccelMS2();
    float curve_accel_ms2 = LineTrack_Get_TurnOut() * T5_CURVE_ACCEL_FF_GAIN;
    float accel_ms2;

    if (KeyMenu_GetTaskID() != TASK_T5) {
        T5_ResetCarAccelFeedforward();
        return 0.0f;
    }

    if (!t5_ff_session_active) {
        t5_ff_session_active = true;
        t5_start_hold_ticks = T5_CAR_ACCEL_START_HOLD_TICKS;
        t5_stop_hold_ticks = 0U;
        t5_prev_line_accel_ms2 = 0.0f;
    }

    if ((line_accel_ms2 > T5_CAR_ACCEL_EDGE_DEADBAND_MS2) &&
        (t5_prev_line_accel_ms2 <= T5_CAR_ACCEL_EDGE_DEADBAND_MS2)) {
        t5_start_hold_ticks = T5_CAR_ACCEL_START_HOLD_TICKS;
    }
    if ((line_accel_ms2 < -T5_CAR_ACCEL_EDGE_DEADBAND_MS2) &&
        (t5_prev_line_accel_ms2 >= -T5_CAR_ACCEL_EDGE_DEADBAND_MS2)) {
        t5_stop_hold_ticks = T5_CAR_ACCEL_STOP_HOLD_TICKS;
    }

    t5_prev_line_accel_ms2 = line_accel_ms2;

    curve_accel_ms2 = clamp_float(curve_accel_ms2,
                                  -T5_CURVE_ACCEL_FF_LIMIT_MS2,
                                  T5_CURVE_ACCEL_FF_LIMIT_MS2);
    accel_ms2 = T5_CAR_ACCEL_FF_SIGN *
        ((line_accel_ms2 * T5_CAR_ACCEL_FF_GAIN) + curve_accel_ms2);
    if (t5_start_hold_ticks > 0U) {
        accel_ms2 += T5_CAR_ACCEL_FF_SIGN * T5_CAR_ACCEL_HOLD_MS2;
        t5_start_hold_ticks--;
    }
    if (t5_stop_hold_ticks > 0U) {
        accel_ms2 -= T5_CAR_ACCEL_FF_SIGN * T5_CAR_ACCEL_HOLD_MS2;
        t5_stop_hold_ticks--;
    }

    return clamp_float(accel_ms2,
                       -T5_CAR_ACCEL_FF_LIMIT_MS2,
                       T5_CAR_ACCEL_FF_LIMIT_MS2);
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

#if APP_AUTO_START_TASK
    KeyMenu_StartTask((TaskID)APP_AUTO_START_TASK);
#endif

    while (1) {
        uint32_t now;

        IMU601_poll();
        Vofa_Poll();
        VisionUart_Poll(control_ticks_10ms);

        g_vision_ball = VisionUart_GetLatest();
        T3Task_UpdateVision(g_vision_ball.valid, g_vision_ball.timed_out,
                            g_vision_ball.seq, g_vision_ball.x_mm);
        if (g_vision_ball.valid) {
            if ((!has_vision_seq) || (g_vision_ball.seq != last_vision_seq)) {
                uint32_t sample_tick = control_ticks_10ms;
                uint32_t sample_ticks = has_vision_seq ?
                    (uint32_t)(sample_tick - last_vision_tick) : 1U;

                if (sample_ticks == 0U) {
                    sample_ticks = 1U;
                }
                BalanceControl_SetRawPositionTimed(
                    &bc,
                    (float)g_vision_ball.x_mm / 10.0f,
                    (float)sample_ticks * BC_DT_S);
                last_vision_seq = g_vision_ball.seq;
                last_vision_tick = sample_tick;
                has_vision_seq = true;
            }
        } else {
            has_vision_seq = false;
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
            if (!T3Task_IsActive()) {
                Vofa_SendTelemetry();
            }
        }

        if ((uint32_t)(now - last_oled_tick) >= APP_OLED_PERIOD_TICKS) {
            last_oled_tick = now;

            KeyMenu_OLED();
            OLED_RequestRefresh();
        }
        OLED_RefreshStep();
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO: {
            bool need_balance = false;

            KeyMenu_Scan();
            T3Task_Tick10ms();

            switch (g_control_state) {
                case CONTROL_IDLE:
                    T4_ResetCarAccelFeedforward();
                    T5_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, 0.0f);
                    LineTrack_Stop();
                    PD42S1_SoftLockCenter();
                    break;

                case CONTROL_TRACK_ONLY:
                    T4_ResetCarAccelFeedforward();
                    T5_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, 0.0f);
                    LineTrack_Loop_10ms();
                    PD42S1_SoftLockCenter();
                    break;

                case CONTROL_STATIC_BALL:
                    T4_ResetCarAccelFeedforward();
                    T5_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, 0.0f);
                    LineTrack_Stop();
                    need_balance = true;
                    break;

                case CONTROL_DYNAMIC_BALL:
                    LineTrack_Loop_10ms();
                    need_balance = true;
                    break;

                default:
                    T4_ResetCarAccelFeedforward();
                    T5_ResetCarAccelFeedforward();
                    LineTrack_Stop();
                    PD42S1_SoftLockCenter();
                    break;
            }

            if (need_balance) {
                if (KeyMenu_GetTaskID() == TASK_T4) {
                    T5_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, T4_BuildCarAccelFeedforward());
                } else if (KeyMenu_GetTaskID() == TASK_T5) {
                    T4_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, T5_BuildCarAccelFeedforward());
                } else {
                    T4_ResetCarAccelFeedforward();
                    T5_ResetCarAccelFeedforward();
                    BalanceControl_SetCarAccel(&bc, 0.0f);
                }
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
