#include "vofa.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "line_follow.h"
#include "motor.h"
#include "uart.h"
#include "vofa_protocol.h"

#define VOFA_RX_BUF_SIZE 96
#define VOFA_BUILD_TAG "MTEST_IMM_V2"

static volatile char vofa_rx_buf[VOFA_RX_BUF_SIZE];
static volatile uint8_t vofa_rx_len = 0;
static volatile bool vofa_line_ready = false;
static char vofa_line[VOFA_RX_BUF_SIZE];

static void vofa_send(const char *text)
{
    UART_send_string(VOFA_INST, text);
}

static const char *parse_result_text(VofaParseResult result)
{
    switch (result) {
    case VOFA_PARSE_EMPTY:
        return "EMPTY";
    case VOFA_PARSE_UNKNOWN:
        return "UNKNOWN";
    case VOFA_PARSE_BAD_VALUE:
        return "BAD_VALUE";
    case VOFA_PARSE_OK:
    default:
        return "OK";
    }
}

static void send_params(void)
{
    char buf[192];
    PidTuningParams params;

    LineTrack_GetParams(&params);
    snprintf(buf, sizeof(buf),
             "#PID LINE %.4f %.4f %.4f SL %.4f %.4f %.4f SR %.4f %.4f %.4f BASE %.4f\r\n",
             params.line.kp, params.line.ki, params.line.kd,
             params.speed_left.kp, params.speed_left.ki, params.speed_left.kd,
             params.speed_right.kp, params.speed_right.ki, params.speed_right.kd,
             params.base_speed);
    vofa_send(buf);
}

static void set_pid_gain(PidGainSet *gains, VofaPidTerm term, float value)
{
    switch (term) {
    case VOFA_PID_KP:
        gains->kp = value;
        break;
    case VOFA_PID_KI:
        gains->ki = value;
        break;
    case VOFA_PID_KD:
    default:
        gains->kd = value;
        break;
    }
}

static bool apply_set_pid(const VofaCommand *cmd)
{
    PidTuningParams params;

    LineTrack_GetParams(&params);

    switch (cmd->pid_group) {
    case VOFA_PID_LINE:
        set_pid_gain(&params.line, cmd->pid_term, cmd->value);
        break;
    case VOFA_PID_SPEED_LEFT:
        set_pid_gain(&params.speed_left, cmd->pid_term, cmd->value);
        break;
    case VOFA_PID_SPEED_RIGHT:
    default:
        set_pid_gain(&params.speed_right, cmd->pid_term, cmd->value);
        break;
    }

    if (!PidParams_AreValid(&params)) {
        return false;
    }

    LineTrack_SetParams(&params);
    return true;
}

static bool apply_base_speed(float value)
{
    PidTuningParams params;

    LineTrack_GetParams(&params);
    params.base_speed = value;

    if (!PidParams_AreValid(&params)) {
        return false;
    }

    LineTrack_SetParams(&params);
    return true;
}

static void send_motor_status(void)
{
    char buf[192];
    Motor_DebugStatus motor;

    Motor_ReadDebugStatus(&motor);
    snprintf(buf, sizeof(buf),
        "#MSTAT LPWM %.0f RPWM %.0f LCC %u RCC %u LRAW %d RRAW %d LPUL %ld RPUL %ld AIN %u%u BIN %u%u PPIN %u%u ADO %08lX AOE %08lX BDO %08lX BOE %08lX %s\r\n",
        LineTrack_Get_LeftPwm(),
        LineTrack_Get_RightPwm(),
        (unsigned)motor.cc_left,
        (unsigned)motor.cc_right,
        (int)g_Encoder.speed_left,
        (int)g_Encoder.speed_right,
        (long)g_Encoder.pulses_left,
        (long)g_Encoder.pulses_right,
        (unsigned)motor.ain1,
        (unsigned)motor.ain2,
        (unsigned)motor.bin1,
        (unsigned)motor.bin2,
        (unsigned)motor.pwm_left_pin,
        (unsigned)motor.pwm_right_pin,
        (unsigned long)motor.gpioa_dout,
        (unsigned long)motor.gpioa_doe,
        (unsigned long)motor.gpiob_dout,
        (unsigned long)motor.gpiob_doe,
        VOFA_BUILD_TAG);
    vofa_send(buf);
}

static void handle_command(const char *line)
{
    VofaCommand cmd;
    PidTuningParams params;
    VofaParseResult result = VofaProtocol_ParseLine(line, &cmd);

    if (result != VOFA_PARSE_OK) {
        char buf[48];
        snprintf(buf, sizeof(buf), "#ERR %s\r\n", parse_result_text(result));
        vofa_send(buf);
        return;
    }

    switch (cmd.type) {
    case VOFA_CMD_SET_PID:
        vofa_send(apply_set_pid(&cmd) ? "#ACK SET_PID\r\n" : "#ERR BAD_RANGE\r\n");
        break;
    case VOFA_CMD_SET_BASE_SPEED:
        vofa_send(apply_base_speed(cmd.value) ? "#ACK BASE\r\n" : "#ERR BAD_RANGE\r\n");
        break;
    case VOFA_CMD_SAVE:
        vofa_send("#ACK SAVE RAM_ONLY\r\n");
        break;
    case VOFA_CMD_LOAD:
        PidParams_SetDefaults(&params);
        LineTrack_SetParams(&params);
        LineTrack_ClearPidState();
        vofa_send("#ACK LOAD DEFAULT\r\n");
        break;
    case VOFA_CMD_RESET:
        PidParams_SetDefaults(&params);
        LineTrack_SetParams(&params);
        LineTrack_ClearPidState();
        vofa_send("#ACK RESET\r\n");
        break;
    case VOFA_CMD_GET:
        send_params();
        break;
    case VOFA_CMD_CLEAR_PID:
        LineTrack_ClearPidState();
        vofa_send("#ACK PIDCLR\r\n");
        break;
    case VOFA_CMD_MOTOR_TEST:
        if (cmd.motor_test_side == VOFA_MOTOR_TEST_LEFT) {
            char buf[48];
            LineTrack_SetMotorTest((int16_t)cmd.value, 0);
            snprintf(buf, sizeof(buf), "#ACK MTEST LEFT %.0f %s\r\n", cmd.value, VOFA_BUILD_TAG);
            vofa_send(buf);
        } else if (cmd.motor_test_side == VOFA_MOTOR_TEST_RIGHT) {
            char buf[48];
            LineTrack_SetMotorTest(0, (int16_t)cmd.value);
            snprintf(buf, sizeof(buf), "#ACK MTEST RIGHT %.0f %s\r\n", cmd.value, VOFA_BUILD_TAG);
            vofa_send(buf);
        } else if (cmd.motor_test_side == VOFA_MOTOR_TEST_EXIT) {
            LineTrack_ExitMotorTest();
            vofa_send("#ACK MTEST EXIT " VOFA_BUILD_TAG "\r\n");
        } else {
            LineTrack_SetMotorTest(0, 0);
            vofa_send("#ACK MTEST OFF " VOFA_BUILD_TAG "\r\n");
        }
        break;
    case VOFA_CMD_MOTOR_STATUS:
        send_motor_status();
        break;
    case VOFA_CMD_NONE:
    default:
        vofa_send("#ERR UNKNOWN\r\n");
        break;
    }
}

void Vofa_Init(void)
{
    vofa_rx_len = 0;
    vofa_line_ready = false;
    NVIC_EnableIRQ(VOFA_INST_INT_IRQN);
    vofa_send("#VOFA READY 115200 " VOFA_BUILD_TAG "\r\n");
}

void Vofa_Poll(void)
{
    if (!vofa_line_ready) {
        return;
    }

    NVIC_DisableIRQ(VOFA_INST_INT_IRQN);
    strncpy(vofa_line, (const char *)vofa_rx_buf, sizeof(vofa_line) - 1U);
    vofa_line[sizeof(vofa_line) - 1U] = '\0';
    vofa_rx_len = 0;
    vofa_rx_buf[0] = '\0';
    vofa_line_ready = false;
    NVIC_EnableIRQ(VOFA_INST_INT_IRQN);

    handle_command(vofa_line);
}

void Vofa_SendTelemetry(void)
{
    char buf[160];

    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
             LineTrack_Get_Error(),
             LineTrack_Get_TurnOut(),
             LineTrack_Get_BaseSpeed(),
             LineTrack_Get_LeftTarget(),
             LineTrack_Get_RightTarget(),
             LineTrack_Get_FilteredLeft(),
             LineTrack_Get_FilteredRight(),
             LineTrack_Get_LeftPwm(),
             LineTrack_Get_RightPwm());
    vofa_send(buf);
}

void VOFA_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(VOFA_INST)) {
    case DL_UART_IIDX_RX:
    {
        uint8_t byte = DL_UART_receiveData(VOFA_INST);

        if (vofa_line_ready) {
            break;
        }
        if ((byte == '\r') || (byte == '\n')) {
            if (vofa_rx_len > 0U) {
                vofa_rx_buf[vofa_rx_len] = '\0';
                vofa_line_ready = true;
            }
            break;
        }
        if (vofa_rx_len < (VOFA_RX_BUF_SIZE - 1U)) {
            vofa_rx_buf[vofa_rx_len++] = (char)byte;
        } else {
            vofa_rx_len = 0;
        }
        break;
    }
    default:
        break;
    }
}
