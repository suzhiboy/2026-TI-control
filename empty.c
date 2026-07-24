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
#include <stdio.h>

int main(void)
{
    char oled_str[50];

    SYSCFG_DL_init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowLineString(1, 1, "OLED OK");
    OLED_Refresh();
    delay_cycles(1600000);

    LineTrack_Init();
    IMU601_init();

    NVIC_SetPriority(GPIOB_INT_IRQn, 0);
    NVIC_SetPriority(IMU601_INST_INT_IRQN, 1);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2);
    __enable_irq();

    LineTrack_Start(12.0f);
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        IMU601_poll();

        sprintf(oled_str, "Yaw: %.2f", current_attitude.yaw);
        OLED_ShowLineString(1, 1, oled_str);
        sprintf(oled_str, "Pitch: %.2f", current_attitude.pitch);
        OLED_ShowLineString(2, 1, oled_str);
        sprintf(oled_str, "Roll: %.2f", current_attitude.roll);
        OLED_ShowLineString(3, 1, oled_str);
        OLED_ShowLineString(4, 1, "");
        OLED_Refresh();
        delay_cycles(3200000);
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            LineTrack_Loop_10ms();
            break;
        default:
            break;
    }
}
