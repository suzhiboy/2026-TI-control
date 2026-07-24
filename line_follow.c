#include "line_follow.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"

#define DEFAULT_BASE_SPEED 12.0f

PID_TypeDef pid_line;
PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;

static bool line_track_running = false;
static float line_base_speed = DEFAULT_BASE_SPEED;
static float line_error = 0.0f;
static float filtered_L = 0.0f;
static float filtered_R = 0.0f;

static float clamp_target_speed(float speed)
{
    if (speed < 0.0f) {
        return 0.0f;
    }
    return speed;
}

void LineTrack_Init(void)
{
    Motor_Init();
    Encoder_Init();

    PID_Init(&pid_line, 0.7f, 0.05f, 0.5f, 10.0f, -10.0f, 8.0f);
    PID_Init(&pid_speed_L, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    PID_Init(&pid_speed_R, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);

    LineTrack_Reset();
}

void LineTrack_Start(float base_speed)
{
    line_base_speed = (base_speed > 0.0f) ? base_speed : DEFAULT_BASE_SPEED;
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
}

void LineTrack_Loop_10ms(void)
{
    float turn_out = 0.0f;

    Encoder_UpdateData_10ms();

    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    if (line_track_running) {
        line_error = Sensor_Get_Error();
        turn_out = PID_Calc_Positional(&pid_line, line_error);

        pid_speed_L.target = clamp_target_speed(line_base_speed + turn_out);
        pid_speed_R.target = clamp_target_speed(line_base_speed - turn_out);
    } else {
        pid_speed_L.target = 0.0f;
        pid_speed_R.target = 0.0f;
    }

    int16_t final_L_pwm = (int16_t)PID_Calc_Positional(&pid_speed_L, filtered_L);
    int16_t final_R_pwm = (int16_t)PID_Calc_Positional(&pid_speed_R, filtered_R);

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

float LineTrack_Get_FilteredLeft(void)
{
    return filtered_L;
}

float LineTrack_Get_FilteredRight(void)
{
    return filtered_R;
}
