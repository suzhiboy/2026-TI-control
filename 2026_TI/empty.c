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
#include <stdio.h>

static volatile uint32_t control_ticks_10ms = 0;
static MT6701_Data mt6701;
static MT6701_Status mt6701_status = MT6701_ERR_TIMEOUT;

int main(void)
{
    char oled_str[50];
    uint32_t last_vofa_tick = 0;
    uint32_t last_oled_tick = 0;
    VisionBallData vision_ball;

    SYSCFG_DL_init();

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
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        uint32_t now;

        IMU601_poll();
        Vofa_Poll();
        VisionUart_Poll(control_ticks_10ms);

        now = control_ticks_10ms;
        if ((uint32_t)(now - last_vofa_tick) >= 2U) {
            last_vofa_tick = now;
            Vofa_SendTelemetry();
        }

        if ((uint32_t)(now - last_oled_tick) >= 10U) {
            last_oled_tick = now;
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
            vision_ball = VisionUart_GetLatest();
            if (vision_ball.valid) {
                sprintf(oled_str, "Ball:%dmm C%u", vision_ball.x_mm,
                    (unsigned)vision_ball.conf_percent);
            } else if (vision_ball.lost) {
                sprintf(oled_str, "Ball:LOST");
            }
            OLED_ShowLineString(4, 1, oled_str);
            OLED_Refresh();
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            LineTrack_Loop_10ms();
            control_ticks_10ms++;
            break;
        default:
            break;
    }
}
