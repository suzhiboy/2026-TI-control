#include "line_follow.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include "board_config.h"

PID_TypeDef pid_line;
PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;

static bool line_track_running = false;
static float line_base_speed = 12.0f;
static float line_error = 0.0f;
static float line_turn_out = 0.0f;
static float filtered_L = 0.0f;
static float filtered_R = 0.0f;
static float line_left_pwm = 0.0f;
static float line_right_pwm = 0.0f;
static bool motor_test_mode = false;
static int16_t motor_test_left_pwm = 0;
static int16_t motor_test_right_pwm = 0;

static float clamp_target_speed(float speed)
{
    if (speed < 0.0f) {
        return 0.0f;
    }
    return speed;
}

static void apply_gains(PID_TypeDef *pid, const PidGainSet *gains)
{
    pid->Kp = gains->kp;
    pid->Ki = gains->ki;
    pid->Kd = gains->kd;
}

void LineTrack_Init(void)
{
    PidTuningParams defaults;

    Motor_Init();
    Encoder_Init();

    PidParams_SetDefaults(&defaults);
    PID_Init(&pid_line, defaults.line.kp, defaults.line.ki, defaults.line.kd, 10.0f, -10.0f, 8.0f);
    PID_Init(&pid_speed_L, defaults.speed_left.kp, defaults.speed_left.ki, defaults.speed_left.kd,
        LINE_SPEED_PID_OUTPUT_MAX, 0.0f, LINE_SPEED_PID_INTEGRAL_MAX);
    PID_Init(&pid_speed_R, defaults.speed_right.kp, defaults.speed_right.ki, defaults.speed_right.kd,
        LINE_SPEED_PID_OUTPUT_MAX, 0.0f, LINE_SPEED_PID_INTEGRAL_MAX);
    line_base_speed = defaults.base_speed;

    LineTrack_Reset();
}

void LineTrack_Start(float base_speed)
{
    if (base_speed > 0.0f) {
        line_base_speed = base_speed;
    }
    LineTrack_Reset();
    line_track_running = true;
}

void LineTrack_Stop(void)
{
    line_track_running = false;
    pid_speed_L.target = 0.0f;
    pid_speed_R.target = 0.0f;
    Set_Motor_Speed_Left(0);
    Set_Motor_Speed_Right(0);
}

void LineTrack_Reset(void)
{
    PID_Clear(&pid_line);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
    Encoder_Clear();
    filtered_L = 0.0f;
    filtered_R = 0.0f;
    line_error = 0.0f;
    line_turn_out = 0.0f;
    line_left_pwm = 0.0f;
    line_right_pwm = 0.0f;
}

void LineTrack_SetParams(const PidTuningParams *params)
{
    if ((params == NULL) || !PidParams_AreValid(params)) {
        return;
    }

    apply_gains(&pid_line, &params->line);
    apply_gains(&pid_speed_L, &params->speed_left);
    apply_gains(&pid_speed_R, &params->speed_right);
    line_base_speed = params->base_speed;
}

void LineTrack_GetParams(PidTuningParams *params)
{
    if (params == NULL) {
        return;
    }

    params->line.kp = pid_line.Kp;
    params->line.ki = pid_line.Ki;
    params->line.kd = pid_line.Kd;
    params->speed_left.kp = pid_speed_L.Kp;
    params->speed_left.ki = pid_speed_L.Ki;
    params->speed_left.kd = pid_speed_L.Kd;
    params->speed_right.kp = pid_speed_R.Kp;
    params->speed_right.ki = pid_speed_R.Ki;
    params->speed_right.kd = pid_speed_R.Kd;
    params->base_speed = line_base_speed;
}

void LineTrack_ClearPidState(void)
{
    PID_Clear(&pid_line);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
}

void LineTrack_SetMotorTest(int16_t left_pwm, int16_t right_pwm)
{
    motor_test_left_pwm = left_pwm;
    motor_test_right_pwm = right_pwm;
    motor_test_mode = true;
    Encoder_Clear();
    filtered_L = 0.0f;
    filtered_R = 0.0f;
    line_left_pwm = (float)motor_test_left_pwm;
    line_right_pwm = (float)motor_test_right_pwm;
    Set_Motor_Speed_Left(motor_test_left_pwm);
    Set_Motor_Speed_Right(motor_test_right_pwm);
}

void LineTrack_ExitMotorTest(void)
{
    motor_test_mode = false;
    LineTrack_Reset();
}

void LineTrack_Loop_10ms(void)
{
    Encoder_UpdateData_10ms();

    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    if (motor_test_mode) {
        line_left_pwm = (float)motor_test_left_pwm;
        line_right_pwm = (float)motor_test_right_pwm;
        Set_Motor_Speed_Left(motor_test_left_pwm);
        Set_Motor_Speed_Right(motor_test_right_pwm);
        return;
    }

    if (line_track_running) {
        line_error = Sensor_Get_Error();
        line_turn_out = PID_Calc_Positional(&pid_line, line_error);

        pid_speed_L.target = clamp_target_speed(line_base_speed + line_turn_out);
        pid_speed_R.target = clamp_target_speed(line_base_speed - line_turn_out);
    } else {
        line_turn_out = 0.0f;
        pid_speed_L.target = 0.0f;
        pid_speed_R.target = 0.0f;
    }

    int16_t final_L_pwm = (int16_t)PID_Calc_Positional(&pid_speed_L, filtered_L);
    int16_t final_R_pwm = (int16_t)PID_Calc_Positional(&pid_speed_R, filtered_R);
    line_left_pwm = (float)final_L_pwm;
    line_right_pwm = (float)final_R_pwm;

    Set_Motor_Speed_Left(final_L_pwm);
    Set_Motor_Speed_Right(final_R_pwm);
}

bool LineTrack_IsRunning(void)
{
    return line_track_running;
}

float LineTrack_Get_Error(void)
{
    return line_error;
}

float LineTrack_Get_TurnOut(void)
{
    return line_turn_out;
}

float LineTrack_Get_BaseSpeed(void)
{
    return line_base_speed;
}

float LineTrack_Get_FilteredLeft(void)
{
    return filtered_L;
}

float LineTrack_Get_FilteredRight(void)
{
    return filtered_R;
}

float LineTrack_Get_LeftTarget(void)
{
    return pid_speed_L.target;
}

float LineTrack_Get_RightTarget(void)
{
    return pid_speed_R.target;
}

float LineTrack_Get_LeftPwm(void)
{
    return line_left_pwm;
}

float LineTrack_Get_RightPwm(void)
{
    return line_right_pwm;
}
