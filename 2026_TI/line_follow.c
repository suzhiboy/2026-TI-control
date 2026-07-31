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
static float line_speed_setpoint = 0.0f;    /* 当前实际使用速度 (带斜坡) */
static float line_error = 0.0f;
static float line_turn_out = 0.0f;
static float filtered_L = 0.0f;
static float filtered_R = 0.0f;
static float line_left_pwm = 0.0f;
static float line_right_pwm = 0.0f;
static float line_longitudinal_accel_ms2 = 0.0f;
static bool line_braking = false;
static uint8_t line_brake_timer = 0;
static bool motor_test_mode = false;
static int16_t motor_test_left_pwm = 0;
static int16_t motor_test_right_pwm = 0;
static float line_turn_direction_scale = 1.0f;

#define LINE_OPEN_LOOP_PWM_MODE      0
#define LINE_OPEN_LOOP_BASE_PWM      900.0f
#define LINE_OPEN_LOOP_TURN_GAIN     45.0f
#define LINE_OPEN_LOOP_PWM_MIN       250.0f
#define LINE_TURN_DIRECTION_SIGN    (1.0f)
#define LINE_TURN_OUTPUT_LIMIT       10.0f
#define LINE_TURN_INTEGRAL_LIMIT      8.0f
#define LINE_DEFAULT_ACCEL_STEP_CM_S (0.8f)
#define LINE_DEFAULT_DECEL_STEP_CM_S (0.35f)
#define LINE_DEFAULT_BRAKE_STEP_CM_S (6.0f)

static float line_accel_step_cm_s = LINE_DEFAULT_ACCEL_STEP_CM_S;
static float line_decel_step_cm_s = LINE_DEFAULT_DECEL_STEP_CM_S;
static float line_brake_step_cm_s = LINE_DEFAULT_BRAKE_STEP_CM_S;

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
    PID_Init(&pid_line, defaults.line.kp, defaults.line.ki, defaults.line.kd,
        LINE_TURN_OUTPUT_LIMIT, -LINE_TURN_OUTPUT_LIMIT, LINE_TURN_INTEGRAL_LIMIT);
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
    line_speed_setpoint = 0.0f;             /* 从 0 开始斜坡加速 */
    line_braking = false;
    line_brake_timer = 0;
    line_track_running = true;
}

void LineTrack_Stop(void)
{
    line_track_running = false;
    pid_speed_L.target = 0.0f;
    pid_speed_R.target = 0.0f;
    line_speed_setpoint = 0.0f;
    line_longitudinal_accel_ms2 = 0.0f;
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
    line_longitudinal_accel_ms2 = 0.0f;
}

void LineTrack_SetBaseSpeed(float speed)
{
    if (speed >= 0.0f && speed <= 200.0f) {
        line_base_speed = speed;
    }
}

static bool valid_motion_step(float step)
{
    return (step > 0.0f) && (step <= 20.0f);
}

static void update_longitudinal_accel(float previous_speed_setpoint)
{
    line_longitudinal_accel_ms2 = line_speed_setpoint - previous_speed_setpoint;
}

void LineTrack_SetMotionProfile(float accel_step_cm_s,
                                float decel_step_cm_s,
                                float brake_step_cm_s)
{
    if (valid_motion_step(accel_step_cm_s)) {
        line_accel_step_cm_s = accel_step_cm_s;
    }
    if (valid_motion_step(decel_step_cm_s)) {
        line_decel_step_cm_s = decel_step_cm_s;
    }
    if (valid_motion_step(brake_step_cm_s)) {
        line_brake_step_cm_s = brake_step_cm_s;
    }
}

void LineTrack_ResetMotionProfile(void)
{
    line_accel_step_cm_s = LINE_DEFAULT_ACCEL_STEP_CM_S;
    line_decel_step_cm_s = LINE_DEFAULT_DECEL_STEP_CM_S;
    line_brake_step_cm_s = LINE_DEFAULT_BRAKE_STEP_CM_S;
}

void LineTrack_SetTurnDirectionSign(float sign)
{
    line_turn_direction_scale = (sign >= 0.0f) ? 1.0f : -1.0f;
}

void LineTrack_Brake(void)
{
    if (line_track_running) {
        line_braking = true;
        line_brake_timer = 0;
    }
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
    float previous_speed_setpoint = line_speed_setpoint;

    Encoder_UpdateData_10ms();

    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    if (motor_test_mode) {
        line_longitudinal_accel_ms2 = 0.0f;
        line_left_pwm = (float)motor_test_left_pwm;
        line_right_pwm = (float)motor_test_right_pwm;
        Set_Motor_Speed_Left(motor_test_left_pwm);
        Set_Motor_Speed_Right(motor_test_right_pwm);
        return;
    }

#if LINE_OPEN_LOOP_PWM_MODE
    if (line_track_running) {
        float left_pwm;
        float right_pwm;

        line_error = Sensor_Get_Error();
        line_turn_out = LINE_TURN_DIRECTION_SIGN * PID_Calc_Positional(&pid_line, line_error);
        line_turn_out *= line_turn_direction_scale;

        if (line_braking) {
            line_brake_timer++;
            if (line_brake_timer <= 5) {
                Set_Motor_Speed_Left(0);
                Set_Motor_Speed_Right(0);
                return;
            }
            line_braking = false;
            line_track_running = false;
            line_speed_setpoint = 0.0f;
            Set_Motor_Speed_Left(0);
            Set_Motor_Speed_Right(0);
            return;
        }

        left_pwm = LINE_OPEN_LOOP_BASE_PWM + line_turn_out * LINE_OPEN_LOOP_TURN_GAIN;
        right_pwm = LINE_OPEN_LOOP_BASE_PWM - line_turn_out * LINE_OPEN_LOOP_TURN_GAIN;

        if (left_pwm < LINE_OPEN_LOOP_PWM_MIN) {
            left_pwm = LINE_OPEN_LOOP_PWM_MIN;
        }
        if (right_pwm < LINE_OPEN_LOOP_PWM_MIN) {
            right_pwm = LINE_OPEN_LOOP_PWM_MIN;
        }
        if (left_pwm > (float)MOTOR_PWM_MAX) {
            left_pwm = (float)MOTOR_PWM_MAX;
        }
        if (right_pwm > (float)MOTOR_PWM_MAX) {
            right_pwm = (float)MOTOR_PWM_MAX;
        }

        line_left_pwm = left_pwm;
        line_right_pwm = right_pwm;
        pid_speed_L.target = 0.0f;
        pid_speed_R.target = 0.0f;
        Set_Motor_Speed_Left((int16_t)left_pwm);
        Set_Motor_Speed_Right((int16_t)right_pwm);
        return;
    }
#endif

    if (line_track_running) {
        /* ---- 速度斜坡: 逐渐跟随目标速度 ---- */
        if (!line_braking) {
            if (line_speed_setpoint < line_base_speed) {
                line_speed_setpoint += line_accel_step_cm_s;
                if (line_speed_setpoint > line_base_speed) {
                    line_speed_setpoint = line_base_speed;
                }
            } else if (line_speed_setpoint > line_base_speed) {
                line_speed_setpoint -= line_decel_step_cm_s;
                if (line_speed_setpoint < line_base_speed) {
                    line_speed_setpoint = line_base_speed;
                }
            }
        }

        /* ---- 刹车流程 ---- */
        if (line_braking) {
            line_brake_timer++;
            if (line_speed_setpoint > line_brake_step_cm_s) {
                line_speed_setpoint -= line_brake_step_cm_s;
            } else {
                line_speed_setpoint = 0.0f;
            }

            if ((line_speed_setpoint <= 0.0f) && (line_brake_timer >= 3U)) {
                update_longitudinal_accel(previous_speed_setpoint);
                Set_Motor_Speed_Left(0);
                Set_Motor_Speed_Right(0);
                pid_speed_L.target = 0.0f;
                pid_speed_R.target = 0.0f;
                line_left_pwm = 0.0f;
                line_right_pwm = 0.0f;
                line_braking = false;
                line_track_running = false;
                return;
            }
        }

        line_error = Sensor_Get_Error();
        line_turn_out = LINE_TURN_DIRECTION_SIGN * PID_Calc_Positional(&pid_line, line_error);
        line_turn_out *= line_turn_direction_scale;

        pid_speed_L.target = clamp_target_speed(line_speed_setpoint + line_turn_out);
        pid_speed_R.target = clamp_target_speed(line_speed_setpoint - line_turn_out);
    } else {
        line_turn_out = 0.0f;
        pid_speed_L.target = 0.0f;
        pid_speed_R.target = 0.0f;
    }

    update_longitudinal_accel(previous_speed_setpoint);

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

float LineTrack_GetLongitudinalAccelMS2(void)
{
    return line_longitudinal_accel_ms2;
}
