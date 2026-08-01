/*
 * key_menu.c - six-key task launcher
 *
 * Scope of this file:
 * - scan K1..K6 every 10 ms,
 * - map K1..K6 directly to TASK_T1..TASK_T6,
 * - start/stop the registered task entry points.
 *
 * Task internals stay behind the TaskDef init/run/stop callbacks.
 */

#include "key_menu.h"
#include "encoder.h"
#include "line_follow.h"
#include "oled.h"
#include "board_config.h"
#include "sensor.h"
#include "sys_state.h"
#include "t3_task.h"
#include <stdio.h>

static void PB13_SetGPIO(void);
static void PB13_SetSensorMuxOutput(void);
static void Start_SelectedTask(void);
static void Key_FlushAll(void);
static bool Key_ConsumeShort(uint8_t idx);
static bool Key_ConsumeLong(uint8_t idx);
static void Key_ClearOne(uint8_t idx);
static void Target_Clamp(void);

#define T2_LAP_DISTANCE_CM          (614.0f)
#define T2_START_LINE_IGNORE_CM     (100.0f)
#define T2_SEARCH_START_CM          (560.0f)
#define T2_FAILSAFE_OVERRUN_CM      (5.0f)
#define T2_STOP_COMPENSATION_CM     (5.0f)
#define T2_MIN_RUN_TICKS_10MS       (100U)
#define T2_LINE_ACTIVE_MIN          (3U)
#define T2_LINE_ACTIVE_MAX          (4U)
#define T2_LINE_CONFIRM_TICKS       (3U)
#define T4_AB_DISTANCE_CM           (150.0f)
#define T4_BASE_SPEED_CM_S          (22.0f)
#define T4_DECEL_START_CM           (125.0f)
#define T4_APPROACH_SPEED_CM_S      (12.0f)
#define T4_ACCEL_STEP_CM_S          (0.35f)
#define T4_DECEL_STEP_CM_S          (0.35f)
#define T4_BRAKE_STEP_CM_S          (0.35f)
#define T4_BRAKE_HOLD_TICKS_10MS    (80U)
#define T4_BRAKE_MAX_HOLD_TICKS_10MS (220U)
#define T4_BRAKE_SETTLE_ERROR_MM    (5)
#define T4_BRAKE_SETTLE_TICKS_10MS  (20U)
#define T4_VISION_LOST_BRAKE_TICKS  (10U)
#define T4_MAX_TIME_TICKS_10MS      (800U)
#define T4_TURN_DIRECTION_SIGN      (1.0f)
#define T4_BALL_NEGATIVE_DELTA_LIMIT_US (320U)
#define T4_BALL_POSITIVE_DELTA_LIMIT_US (320U)
#define T4_BALL_MINIMUM_DRIVE_US         (0U)
#define T4_BALL_POSITION_KP              (0.05f)
#define T4_BALL_POSITION_KI              (0.0035f)
#define T4_BALL_POSITION_KD              (0.02f)
#define T4_BALL_VELOCITY_KP              (0.005f)
#define T4_BALL_VELOCITY_KI              (0.0f)
#define T4_BALL_VELOCITY_KD              (0.0f)
#define T5_LAP_DISTANCE_CM          T2_LAP_DISTANCE_CM
#define T5_START_LINE_IGNORE_CM     T2_START_LINE_IGNORE_CM
#define T5_SEARCH_START_CM          T2_SEARCH_START_CM
#define T5_FAILSAFE_OVERRUN_CM      T2_FAILSAFE_OVERRUN_CM
#define T5_STOP_COMPENSATION_CM     (13.0f)
#define T5_MIN_RUN_TICKS_10MS       T2_MIN_RUN_TICKS_10MS
#define T5_LINE_CONFIRM_TICKS       T2_LINE_CONFIRM_TICKS
#define T5_BASE_SPEED_CM_S          (23.0f)
#define T5_SAFE_SPEED_CM_S          (16.0f)
#define T5_FINAL_APPROACH_SPEED_CM_S (8.0f)
#define T5_ACCEL_STEP_CM_S          (0.20f)
#define T5_DECEL_STEP_CM_S          (0.20f)
#define T5_BRAKE_STEP_CM_S          (0.35f)
#define T5_BALL_ERROR_SLOWDOWN_MM   (6)
#define T5_BALL_ERROR_RECOVERY_MM   (4)
#define T5_BALL_RECOVERY_TICKS_10MS (20U)
#define T5_VISION_LOST_BRAKE_TICKS  (10U)
#define T5_BALL_NEGATIVE_DELTA_LIMIT_US (320U)
#define T5_BALL_POSITIVE_DELTA_LIMIT_US (320U)
#define T5_BALL_MINIMUM_DRIVE_US         (0U)
#define T5_BALL_POSITION_KP              (0.05f)
#define T5_BALL_POSITION_KI              (0.0035f)
#define T5_BALL_POSITION_KD              (0.02f)
#define T5_BALL_VELOCITY_KP              (0.005f)
#define T5_BALL_VELOCITY_KI              (0.0f)
#define T5_BALL_VELOCITY_KD              (0.0f)
#define T5_MAX_TIME_TICKS_10MS      (3000U)
#define T5_TURN_DIRECTION_SIGN      T4_TURN_DIRECTION_SIGN
#define T6_LAP_DISTANCE_CM          T5_LAP_DISTANCE_CM
#define T6_START_LINE_IGNORE_CM     T5_START_LINE_IGNORE_CM
#define T6_SEARCH_START_CM          T5_SEARCH_START_CM
#define T6_FAILSAFE_OVERRUN_CM      T5_FAILSAFE_OVERRUN_CM
#define T6_STOP_COMPENSATION_CM     T2_STOP_COMPENSATION_CM
#define T6_MIN_RUN_TICKS_10MS       T5_MIN_RUN_TICKS_10MS
#define T6_LINE_CONFIRM_TICKS       T5_LINE_CONFIRM_TICKS
#define T6_BASE_SPEED_CM_S          (20.0f)
#define T6_MAX_TIME_TICKS_10MS      T5_MAX_TIME_TICKS_10MS
#define T6_TURN_DIRECTION_SIGN      T5_TURN_DIRECTION_SIGN
#define TARGET_MIN_MM               (-120)
#define TARGET_MAX_MM               ( 120)

typedef enum {
    T4_STATE_IDLE = 0,
    T4_STATE_RUN_TO_B,
    T4_STATE_APPROACH_B,
    T4_STATE_BRAKE,
    T4_STATE_DONE,
} T4State;

typedef enum {
    T2_STATE_IDLE = 0,
    T2_STATE_IGNORE_START_LINE,
    T2_STATE_LAP,
    T2_STATE_FIND_A_LINE,
    T2_STATE_ADVANCE_TO_MARK,
    T2_STATE_BRAKE,
} T2State;

typedef enum {
    T5_STATE_IDLE = 0,
    T5_STATE_IGNORE_START_LINE,
    T5_STATE_LAP,
    T5_STATE_FIND_A_LINE,
    T5_STATE_ADVANCE_TO_MARK,
    T5_STATE_BRAKE,
    T5_STATE_DONE,
} T5State;

typedef enum {
    T6_STATE_IDLE = 0,
    T6_STATE_IGNORE_START_LINE,
    T6_STATE_LAP,
    T6_STATE_FIND_A_LINE,
    T6_STATE_ADVANCE_TO_MARK,
    T6_STATE_BRAKE,
    T6_STATE_DONE,
} T6State;

typedef enum {
    T6_SETUP_SIGN = 0,
    T6_SETUP_CM,
    T6_SETUP_TENTH,
} T6SetupMode;

static MenuState menu;
static uint32_t t2_elapsed_ticks_10ms = 0U;
static uint32_t t2_last_logic_tick = 0U;
static uint32_t t2_finish_ticks_10ms = 0U;
static bool t2_brake_requested = false;
static T2State t2_state = T2_STATE_IDLE;
static float t2_line_seen_distance_cm = 0.0f;
static uint8_t t2_line_confirm_ticks = 0U;
static uint32_t t4_elapsed_ticks_10ms = 0U;
static uint32_t t4_finish_ticks_10ms = 0U;
static uint32_t t4_last_logic_tick = 0U;
static uint32_t t4_brake_requested_tick = 0U;
static float t4_restore_base_speed = 0.0f;
static bool t4_brake_requested = false;
static uint8_t t4_vision_lost_ticks = 0U;
static uint8_t t4_brake_settle_ticks = 0U;
static T4State t4_state = T4_STATE_IDLE;
static uint32_t t5_elapsed_ticks_10ms = 0U;
static uint32_t t5_finish_ticks_10ms = 0U;
static uint32_t t5_last_logic_tick = 0U;
static float t5_restore_base_speed = 0.0f;
static bool t5_brake_requested = false;
static T5State t5_state = T5_STATE_IDLE;
static float t5_line_seen_distance_cm = 0.0f;
static uint8_t t5_line_confirm_ticks = 0U;
static uint8_t t5_ball_recovery_ticks = 0U;
static uint8_t t5_vision_lost_ticks = 0U;
static bool t5_safe_speed_active = false;
static uint32_t t6_elapsed_ticks_10ms = 0U;
static uint32_t t6_finish_ticks_10ms = 0U;
static uint32_t t6_last_logic_tick = 0U;
static float t6_restore_base_speed = 0.0f;
static bool t6_brake_requested = false;
static T6State t6_state = T6_STATE_IDLE;
static float t6_line_seen_distance_cm = 0.0f;
static uint8_t t6_line_confirm_ticks = 0U;
static bool t6_setup_active = false;
static T6SetupMode t6_setup_mode = T6_SETUP_SIGN;
static int8_t t6_setup_sign = 1;
static uint8_t t6_setup_cm = 0U;
static uint8_t t6_setup_tenth = 0U;
static bool t6_setup_wait_release = false;
static bool t6_skip_next_stop_short = false;

static const T3TuneProfile t5_ball_profile = {
    .negative_delta_limit_us = T5_BALL_NEGATIVE_DELTA_LIMIT_US,
    .positive_delta_limit_us = T5_BALL_POSITIVE_DELTA_LIMIT_US,
    .minimum_drive_us = T5_BALL_MINIMUM_DRIVE_US,
    .position_kp = T5_BALL_POSITION_KP,
    .position_ki = T5_BALL_POSITION_KI,
    .position_kd = T5_BALL_POSITION_KD,
    .velocity_kp = T5_BALL_VELOCITY_KP,
    .velocity_ki = T5_BALL_VELOCITY_KI,
    .velocity_kd = T5_BALL_VELOCITY_KD,
};

static uint8_t T2_CountActiveSensors(void)
{
    uint8_t data[SENSOR_COUNT];
    uint8_t active_count = 0U;

    Sensor_Read_All(data);
    for (uint8_t i = 0U; i < SENSOR_COUNT; i++) {
        if (data[i] != 0U) {
            active_count++;
        }
    }

    return active_count;
}

static bool T2_StopLineDetected(void)
{
    uint8_t active_count = T2_CountActiveSensors();

    return (active_count >= T2_LINE_ACTIVE_MIN) &&
           (active_count <= T2_LINE_ACTIVE_MAX);
}

static bool T5_StopLineDetected(void)
{
    return T2_StopLineDetected();
}

static bool T6_StopLineDetected(void)
{
    return T2_StopLineDetected();
}

static void T2_RequestBrake(void)
{
    if (!t2_brake_requested) {
        t2_brake_requested = true;
        t2_state = T2_STATE_BRAKE;
        LineTrack_Brake();
    }
}

#define T1_TUNE_TARGET_MM                      (0)
#define T1_TUNE_NEGATIVE_DELTA_LIMIT_US        (150U)
#define T1_TUNE_POSITIVE_DELTA_LIMIT_US        (150U)
#define T1_TUNE_MINIMUM_DRIVE_US               (0U)
#define T1_TUNE_POSITION_KP                    (0.05f)
#define T1_TUNE_POSITION_KI                    (0.0f)
#define T1_TUNE_POSITION_KD                    (0.02f)
#define T1_TUNE_VELOCITY_KP                    (0.005f)
#define T1_TUNE_VELOCITY_KI                    (0.0f)
#define T1_TUNE_VELOCITY_KD                    (0.0f)

static const T3TuneProfile t1_tune_profile = {
    .negative_delta_limit_us = T1_TUNE_NEGATIVE_DELTA_LIMIT_US,
    .positive_delta_limit_us = T1_TUNE_POSITIVE_DELTA_LIMIT_US,
    .minimum_drive_us = T1_TUNE_MINIMUM_DRIVE_US,
    .position_kp = T1_TUNE_POSITION_KP,
    .position_ki = T1_TUNE_POSITION_KI,
    .position_kd = T1_TUNE_POSITION_KD,
    .velocity_kp = T1_TUNE_VELOCITY_KP,
    .velocity_ki = T1_TUNE_VELOCITY_KI,
    .velocity_kd = T1_TUNE_VELOCITY_KD,
};

static const T3TuneProfile t4_ball_profile = {
    .negative_delta_limit_us = T4_BALL_NEGATIVE_DELTA_LIMIT_US,
    .positive_delta_limit_us = T4_BALL_POSITIVE_DELTA_LIMIT_US,
    .minimum_drive_us = T4_BALL_MINIMUM_DRIVE_US,
    .position_kp = T4_BALL_POSITION_KP,
    .position_ki = T4_BALL_POSITION_KI,
    .position_kd = T4_BALL_POSITION_KD,
    .velocity_kp = T4_BALL_VELOCITY_KP,
    .velocity_ki = T4_BALL_VELOCITY_KI,
    .velocity_kd = T4_BALL_VELOCITY_KD,
};

static void T1_Init(void)
{
    T3Task_StartTuneHold(T1_TUNE_TARGET_MM, &t1_tune_profile);
}

static void T1_Run(void)
{
    T3Task_Run();
}

static void T1_Stop(void)
{
    T3Task_Stop();
}

static void T2_Init(void)
{
    T3Task_Stop();
    t2_elapsed_ticks_10ms = 0U;
    t2_last_logic_tick = 0U;
    t2_finish_ticks_10ms = 0U;
    t2_brake_requested = false;
    t2_state = T2_STATE_IGNORE_START_LINE;
    t2_line_seen_distance_cm = 0.0f;
    t2_line_confirm_ticks = 0U;
    ControlState_Set(CONTROL_TRACK_ONLY);
    LineTrack_Start(LineTrack_Get_BaseSpeed());
}

static void T2_Run(void)
{
    if (t2_finish_ticks_10ms != 0U) {
        return;
    }

    if (t2_last_logic_tick == t2_elapsed_ticks_10ms) {
        return;
    }
    t2_last_logic_tick = t2_elapsed_ticks_10ms;

    switch (t2_state) {
        case T2_STATE_IGNORE_START_LINE:
            if ((t2_elapsed_ticks_10ms >= T2_MIN_RUN_TICKS_10MS) &&
                (g_Encoder.distance_cm >= T2_START_LINE_IGNORE_CM)) {
                t2_state = T2_STATE_LAP;
            }
            break;

        case T2_STATE_LAP:
            if (g_Encoder.distance_cm >= T2_SEARCH_START_CM) {
                t2_state = T2_STATE_FIND_A_LINE;
                t2_line_confirm_ticks = 0U;
            }
            break;

        case T2_STATE_FIND_A_LINE:
            if (T2_StopLineDetected()) {
                if (t2_line_confirm_ticks < T2_LINE_CONFIRM_TICKS) {
                    t2_line_confirm_ticks++;
                }
            } else {
                t2_line_confirm_ticks = 0U;
            }

            if (t2_line_confirm_ticks >= T2_LINE_CONFIRM_TICKS) {
                t2_line_seen_distance_cm = g_Encoder.distance_cm;
                t2_state = T2_STATE_ADVANCE_TO_MARK;
            } else if (g_Encoder.distance_cm >=
                       (T2_LAP_DISTANCE_CM + T2_FAILSAFE_OVERRUN_CM)) {
                T2_RequestBrake();
            }
            break;

        case T2_STATE_ADVANCE_TO_MARK:
            if ((g_Encoder.distance_cm - t2_line_seen_distance_cm) >=
                T2_STOP_COMPENSATION_CM) {
                T2_RequestBrake();
            }
            break;

        case T2_STATE_BRAKE:
        case T2_STATE_IDLE:
        default:
            break;
    }

    if (t2_brake_requested && !LineTrack_IsRunning()) {
        t2_finish_ticks_10ms = t2_elapsed_ticks_10ms;
        ControlState_Set(CONTROL_IDLE);
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        t2_state = T2_STATE_IDLE;
    }
}

static void T2_Stop(void)
{
    ControlState_Set(CONTROL_IDLE);
}

static void T3_Init(void)  { T3Task_Start(); }
static void T3_Run(void)   { T3Task_Run(); }
static void T3_Stop(void)  { T3Task_Stop(); }

static void T4_RequestBrake(void)
{
    if (!t4_brake_requested) {
        t4_brake_requested = true;
        t4_brake_requested_tick = t4_elapsed_ticks_10ms;
        t4_state = T4_STATE_BRAKE;
        LineTrack_Brake();
    }
}

static void T4_Init(void)
{
    T3Task_Stop();
    t4_elapsed_ticks_10ms = 0U;
    t4_finish_ticks_10ms = 0U;
    t4_last_logic_tick = 0U;
    t4_brake_requested_tick = 0U;
    t4_brake_requested = false;
    t4_vision_lost_ticks = 0U;
    t4_brake_settle_ticks = 0U;
    t4_state = T4_STATE_RUN_TO_B;
    t4_restore_base_speed = LineTrack_Get_BaseSpeed();

    T3Task_StartProfileHold(0, &t4_ball_profile);
    LineTrack_SetTurnDirectionSign(T4_TURN_DIRECTION_SIGN);
    LineTrack_SetMotionProfile(T4_ACCEL_STEP_CM_S,
        T4_DECEL_STEP_CM_S, T4_BRAKE_STEP_CM_S);
    LineTrack_Start(T4_BASE_SPEED_CM_S);
    ControlState_Set(CONTROL_DYNAMIC_BALL);
}

static void T4_Run(void)
{
    if (t4_finish_ticks_10ms != 0U) {
        return;
    }

    if (t4_last_logic_tick == t4_elapsed_ticks_10ms) {
        return;
    }
    t4_last_logic_tick = t4_elapsed_ticks_10ms;

    if (T3Task_HasValidVision()) {
        t4_vision_lost_ticks = 0U;
    } else if (t4_vision_lost_ticks < T4_VISION_LOST_BRAKE_TICKS) {
        t4_vision_lost_ticks++;
    }

    if (t4_vision_lost_ticks >= T4_VISION_LOST_BRAKE_TICKS) {
        T4_RequestBrake();
    }

    if (t4_state == T4_STATE_RUN_TO_B) {
        if (g_Encoder.distance_cm >= T4_AB_DISTANCE_CM) {
            T4_RequestBrake();
        } else if (g_Encoder.distance_cm >= T4_DECEL_START_CM) {
            LineTrack_SetBaseSpeed(T4_APPROACH_SPEED_CM_S);
            t4_state = T4_STATE_APPROACH_B;
        }
    } else if ((t4_state == T4_STATE_APPROACH_B) &&
               (g_Encoder.distance_cm >= T4_AB_DISTANCE_CM)) {
        T4_RequestBrake();
    }

    if (t4_brake_requested && !LineTrack_IsRunning()) {
        uint32_t brake_elapsed =
            t4_elapsed_ticks_10ms - t4_brake_requested_tick;
        int32_t ball_error_mm = (int32_t)T3Task_GetBallXMM();
        bool brake_min_hold_done =
            brake_elapsed >= T4_BRAKE_HOLD_TICKS_10MS;
        bool brake_max_hold_done =
            brake_elapsed >= T4_BRAKE_MAX_HOLD_TICKS_10MS;
        bool ball_settled;

        if (ball_error_mm < 0) {
            ball_error_mm = -ball_error_mm;
        }
        ball_settled = T3Task_HasValidVision() &&
            (ball_error_mm <= T4_BRAKE_SETTLE_ERROR_MM);
        if (ball_settled) {
            if (t4_brake_settle_ticks < T4_BRAKE_SETTLE_TICKS_10MS) {
                t4_brake_settle_ticks++;
            }
        } else {
            t4_brake_settle_ticks = 0U;
        }

        if (brake_min_hold_done &&
            ((t4_brake_settle_ticks >= T4_BRAKE_SETTLE_TICKS_10MS) ||
             brake_max_hold_done)) {
        t4_finish_ticks_10ms = t4_elapsed_ticks_10ms;
        t4_state = T4_STATE_DONE;
        t4_vision_lost_ticks = 0U;
        t4_brake_settle_ticks = 0U;
        LineTrack_SetBaseSpeed(t4_restore_base_speed);
        LineTrack_ResetMotionProfile();
        T3Task_Stop();
        LineTrack_SetTurnDirectionSign(T4_TURN_DIRECTION_SIGN);
        ControlState_Set(CONTROL_IDLE);
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        }
    }
}

static void T4_Stop(void)
{
    LineTrack_Stop();
    LineTrack_SetBaseSpeed(t4_restore_base_speed);
    LineTrack_ResetMotionProfile();
    T3Task_Stop();
    LineTrack_SetTurnDirectionSign(T4_TURN_DIRECTION_SIGN);
    t4_state = T4_STATE_IDLE;
    t4_brake_requested_tick = 0U;
    t4_brake_requested = false;
    t4_vision_lost_ticks = 0U;
    t4_brake_settle_ticks = 0U;
    ControlState_Set(CONTROL_IDLE);
}

static void T5_RequestBrake(void)
{
    if (!t5_brake_requested) {
        t5_brake_requested = true;
        t5_state = T5_STATE_BRAKE;
        LineTrack_Brake();
    }
}

static void T5_UpdateBallSafety(void)
{
    int32_t ball_error_mm;

    if (!T3Task_HasValidVision()) {
        t5_safe_speed_active = true;
        t5_ball_recovery_ticks = 0U;
        if (t5_vision_lost_ticks < T5_VISION_LOST_BRAKE_TICKS) {
            t5_vision_lost_ticks++;
        }
        if (t5_vision_lost_ticks >= T5_VISION_LOST_BRAKE_TICKS) {
            T5_RequestBrake();
        }
        return;
    }

    t5_vision_lost_ticks = 0U;
    ball_error_mm = (int32_t)T3Task_GetBallXMM();
    if (ball_error_mm < 0) {
        ball_error_mm = -ball_error_mm;
    }

    if (ball_error_mm >= T5_BALL_ERROR_SLOWDOWN_MM) {
        t5_safe_speed_active = true;
        t5_ball_recovery_ticks = 0U;
    } else if (t5_safe_speed_active &&
               (ball_error_mm <= T5_BALL_ERROR_RECOVERY_MM)) {
        if (t5_ball_recovery_ticks < T5_BALL_RECOVERY_TICKS_10MS) {
            t5_ball_recovery_ticks++;
        }
        if (t5_ball_recovery_ticks >= T5_BALL_RECOVERY_TICKS_10MS) {
            t5_safe_speed_active = false;
            t5_ball_recovery_ticks = 0U;
        }
    } else if (t5_safe_speed_active) {
        t5_ball_recovery_ticks = 0U;
    }
}

static void T5_ApplyTargetSpeed(void)
{
    if ((t5_state == T5_STATE_IDLE) ||
        (t5_state == T5_STATE_BRAKE) ||
        (t5_state == T5_STATE_DONE)) {
        return;
    }

    if (t5_state == T5_STATE_ADVANCE_TO_MARK) {
        LineTrack_SetBaseSpeed(T5_FINAL_APPROACH_SPEED_CM_S);
    } else if (t5_safe_speed_active) {
        LineTrack_SetBaseSpeed(T5_SAFE_SPEED_CM_S);
    } else {
        LineTrack_SetBaseSpeed(T5_BASE_SPEED_CM_S);
    }
}

static void T5_Init(void)
{
    T3Task_Stop();
    t5_elapsed_ticks_10ms = 0U;
    t5_finish_ticks_10ms = 0U;
    t5_last_logic_tick = 0U;
    t5_restore_base_speed = LineTrack_Get_BaseSpeed();
    t5_brake_requested = false;
    t5_state = T5_STATE_IGNORE_START_LINE;
    t5_line_seen_distance_cm = 0.0f;
    t5_line_confirm_ticks = 0U;
    t5_ball_recovery_ticks = 0U;
    t5_vision_lost_ticks = 0U;
    t5_safe_speed_active = false;

    T3Task_StartProfileHold(0, &t5_ball_profile);
    LineTrack_SetTurnDirectionSign(T5_TURN_DIRECTION_SIGN);
    LineTrack_SetMotionProfile(T5_ACCEL_STEP_CM_S,
                               T5_DECEL_STEP_CM_S,
                               T5_BRAKE_STEP_CM_S);
    LineTrack_Start(T5_BASE_SPEED_CM_S);
    ControlState_Set(CONTROL_DYNAMIC_BALL);
}

static void T5_Run(void)
{
    if (t5_finish_ticks_10ms != 0U) {
        return;
    }

    if (t5_last_logic_tick == t5_elapsed_ticks_10ms) {
        return;
    }
    t5_last_logic_tick = t5_elapsed_ticks_10ms;
    T5_UpdateBallSafety();

    switch (t5_state) {
        case T5_STATE_IGNORE_START_LINE:
            if ((t5_elapsed_ticks_10ms >= T5_MIN_RUN_TICKS_10MS) &&
                (g_Encoder.distance_cm >= T5_START_LINE_IGNORE_CM)) {
                t5_state = T5_STATE_LAP;
            }
            break;

        case T5_STATE_LAP:
            if (g_Encoder.distance_cm >= T5_SEARCH_START_CM) {
                t5_state = T5_STATE_FIND_A_LINE;
                t5_line_confirm_ticks = 0U;
            }
            break;

        case T5_STATE_FIND_A_LINE:
            if (T5_StopLineDetected()) {
                if (t5_line_confirm_ticks < T5_LINE_CONFIRM_TICKS) {
                    t5_line_confirm_ticks++;
                }
            } else {
                t5_line_confirm_ticks = 0U;
            }

            if (t5_line_confirm_ticks >= T5_LINE_CONFIRM_TICKS) {
                t5_line_seen_distance_cm = g_Encoder.distance_cm;
                t5_state = T5_STATE_ADVANCE_TO_MARK;
            } else if (g_Encoder.distance_cm >=
                       (T5_LAP_DISTANCE_CM + T5_FAILSAFE_OVERRUN_CM)) {
                T5_RequestBrake();
            }
            break;

        case T5_STATE_ADVANCE_TO_MARK:
            if ((g_Encoder.distance_cm - t5_line_seen_distance_cm) >=
                T5_STOP_COMPENSATION_CM) {
                T5_RequestBrake();
            }
            break;

        case T5_STATE_BRAKE:
        case T5_STATE_DONE:
        case T5_STATE_IDLE:
        default:
            break;
    }

    T5_ApplyTargetSpeed();

    if (t5_brake_requested && !LineTrack_IsRunning()) {
        t5_finish_ticks_10ms = t5_elapsed_ticks_10ms;
        t5_state = T5_STATE_DONE;
        LineTrack_SetBaseSpeed(t5_restore_base_speed);
        LineTrack_ResetMotionProfile();
        T3Task_Stop();
        LineTrack_SetTurnDirectionSign(T5_TURN_DIRECTION_SIGN);
        ControlState_Set(CONTROL_IDLE);
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
    }
}

static void T5_Stop(void)
{
    LineTrack_Stop();
    LineTrack_SetBaseSpeed(t5_restore_base_speed);
    LineTrack_ResetMotionProfile();
    T3Task_Stop();
    LineTrack_SetTurnDirectionSign(T5_TURN_DIRECTION_SIGN);
    t5_state = T5_STATE_IDLE;
    t5_brake_requested = false;
    t5_ball_recovery_ticks = 0U;
    t5_vision_lost_ticks = 0U;
    t5_safe_speed_active = false;
    ControlState_Set(CONTROL_IDLE);
}

static void T6_RequestBrake(void)
{
    if (!t6_brake_requested) {
        t6_brake_requested = true;
        t6_state = T6_STATE_BRAKE;
        LineTrack_Brake();
    }
}

static void T6_Init(void)
{
    T3Task_Stop();
    t6_elapsed_ticks_10ms = 0U;
    t6_finish_ticks_10ms = 0U;
    t6_last_logic_tick = 0U;
    t6_restore_base_speed = LineTrack_Get_BaseSpeed();
    t6_brake_requested = false;
    t6_state = T6_STATE_IGNORE_START_LINE;
    t6_line_seen_distance_cm = 0.0f;
    t6_line_confirm_ticks = 0U;

    T3Task_StartTargetHold(menu.target_mm);
    LineTrack_SetTurnDirectionSign(T6_TURN_DIRECTION_SIGN);
    LineTrack_Start(T6_BASE_SPEED_CM_S);
    ControlState_Set(CONTROL_DYNAMIC_BALL);
}

static void T6_Run(void)
{
    if (t6_finish_ticks_10ms != 0U) {
        return;
    }

    if (t6_last_logic_tick == t6_elapsed_ticks_10ms) {
        return;
    }
    t6_last_logic_tick = t6_elapsed_ticks_10ms;

    switch (t6_state) {
        case T6_STATE_IGNORE_START_LINE:
            if ((t6_elapsed_ticks_10ms >= T6_MIN_RUN_TICKS_10MS) &&
                (g_Encoder.distance_cm >= T6_START_LINE_IGNORE_CM)) {
                t6_state = T6_STATE_LAP;
            }
            break;

        case T6_STATE_LAP:
            if (g_Encoder.distance_cm >= T6_SEARCH_START_CM) {
                t6_state = T6_STATE_FIND_A_LINE;
                t6_line_confirm_ticks = 0U;
            }
            break;

        case T6_STATE_FIND_A_LINE:
            if (T6_StopLineDetected()) {
                if (t6_line_confirm_ticks < T6_LINE_CONFIRM_TICKS) {
                    t6_line_confirm_ticks++;
                }
            } else {
                t6_line_confirm_ticks = 0U;
            }

            if (t6_line_confirm_ticks >= T6_LINE_CONFIRM_TICKS) {
                t6_line_seen_distance_cm = g_Encoder.distance_cm;
                t6_state = T6_STATE_ADVANCE_TO_MARK;
            } else if (g_Encoder.distance_cm >=
                       (T6_LAP_DISTANCE_CM + T6_FAILSAFE_OVERRUN_CM)) {
                T6_RequestBrake();
            }
            break;

        case T6_STATE_ADVANCE_TO_MARK:
            if ((g_Encoder.distance_cm - t6_line_seen_distance_cm) >=
                T6_STOP_COMPENSATION_CM) {
                T6_RequestBrake();
            }
            break;

        case T6_STATE_BRAKE:
        case T6_STATE_DONE:
        case T6_STATE_IDLE:
        default:
            break;
    }

    if (t6_brake_requested && !LineTrack_IsRunning()) {
        t6_finish_ticks_10ms = t6_elapsed_ticks_10ms;
        t6_state = T6_STATE_DONE;
        LineTrack_SetBaseSpeed(t6_restore_base_speed);
        T3Task_Stop();
        LineTrack_SetTurnDirectionSign(T6_TURN_DIRECTION_SIGN);
        ControlState_Set(CONTROL_IDLE);
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
    }
}

static void T6_Stop(void)
{
    LineTrack_Stop();
    LineTrack_SetBaseSpeed(t6_restore_base_speed);
    T3Task_Stop();
    LineTrack_SetTurnDirectionSign(T6_TURN_DIRECTION_SIGN);
    t6_state = T6_STATE_IDLE;
    t6_brake_requested = false;
    ControlState_Set(CONTROL_IDLE);
}

static void T6_SetupApplyTarget(void)
{
    int16_t magnitude_mm =
        (int16_t)((uint16_t)t6_setup_cm * 10U + (uint16_t)t6_setup_tenth);

    if (magnitude_mm > TARGET_MAX_MM) {
        magnitude_mm = TARGET_MAX_MM;
        t6_setup_cm = (uint8_t)(TARGET_MAX_MM / 10);
        t6_setup_tenth = (uint8_t)(TARGET_MAX_MM % 10);
    }
    menu.target_mm = (int16_t)(t6_setup_sign * magnitude_mm);
}

static void T6_SetupStart(void)
{
    int16_t target_mm = menu.target_mm;
    uint16_t magnitude_mm;

    Target_Clamp();
    target_mm = menu.target_mm;
    t6_setup_sign = (target_mm < 0) ? -1 : 1;
    magnitude_mm = (uint16_t)((target_mm < 0) ? -target_mm : target_mm);
    if (magnitude_mm > TARGET_MAX_MM) {
        magnitude_mm = TARGET_MAX_MM;
    }
    t6_setup_cm = (uint8_t)(magnitude_mm / 10U);
    t6_setup_tenth = (uint8_t)(magnitude_mm % 10U);
    t6_setup_mode = T6_SETUP_SIGN;
    t6_setup_active = true;
    t6_setup_wait_release = false;
    menu.task_id = TASK_T6;
    Key_FlushAll();
    T6_SetupApplyTarget();
}

static void T6_SetupHandleKeys(void)
{
    if (t6_setup_wait_release) {
        if (!menu.keys[KEY_IDX_K6].pressed) {
            Key_ClearOne(KEY_IDX_K6);
            t6_setup_wait_release = false;
        }
        return;
    }

    if (Key_ConsumeShort(KEY_IDX_K6)) {
        switch (t6_setup_mode) {
            case T6_SETUP_SIGN:
                t6_setup_sign = (t6_setup_sign > 0) ? -1 : 1;
                break;

            case T6_SETUP_CM:
                t6_setup_cm++;
                if (t6_setup_cm > (uint8_t)(TARGET_MAX_MM / 10)) {
                    t6_setup_cm = 0U;
                }
                if (t6_setup_cm >= (uint8_t)(TARGET_MAX_MM / 10)) {
                    t6_setup_tenth = 0U;
                }
                break;

            case T6_SETUP_TENTH:
                if (t6_setup_cm >= (uint8_t)(TARGET_MAX_MM / 10)) {
                    t6_setup_tenth = 0U;
                } else {
                    t6_setup_tenth = (uint8_t)((t6_setup_tenth + 1U) % 10U);
                }
                break;

            default:
                t6_setup_mode = T6_SETUP_SIGN;
                break;
        }
        T6_SetupApplyTarget();
    }

    if (Key_ConsumeLong(KEY_IDX_K6)) {
        switch (t6_setup_mode) {
            case T6_SETUP_SIGN:
                t6_setup_mode = T6_SETUP_CM;
                t6_setup_wait_release = true;
                break;

            case T6_SETUP_CM:
                t6_setup_mode = T6_SETUP_TENTH;
                t6_setup_wait_release = true;
                break;

            case T6_SETUP_TENTH:
                T6_SetupApplyTarget();
                t6_setup_active = false;
                t6_skip_next_stop_short = true;
                KeyMenu_StartTask(TASK_T6);
                break;

            default:
                t6_setup_mode = T6_SETUP_SIGN;
                break;
        }
    }
}

static void T1_OLED(void)
{
    char buf[22];
    int16_t ball_x_mm = T3Task_GetBallXMM();

    OLED_ShowLineString(1, 1, "T1 Tune Center  ");
    if (T3Task_HasValidVision()) {
        snprintf(buf, sizeof(buf), "T%+4d X%+4dmm", 0, (int)ball_x_mm);
    } else {
        snprintf(buf, sizeof(buf), "T%+4d X----mm", 0);
    }
    OLED_ShowLineString(2, 1, buf);
    OLED_ShowLineString(3, 1, "Wheel stopped   ");
    OLED_ShowLineString(4, 1, "Tune PID only   ");
}

static void T2_OLED(void)
{
    char buf[22];
    uint32_t shown_ticks = (t2_finish_ticks_10ms != 0U) ?
        t2_finish_ticks_10ms : t2_elapsed_ticks_10ms;

    OLED_ShowLineString(1, 1, "T2 Lap Stop     ");
    snprintf(buf, sizeof(buf), "t:%2lu.%02lus d:%3d",
             (unsigned long)(shown_ticks / 100U),
             (unsigned long)(shown_ticks % 100U),
             (int)g_Encoder.distance_cm);
    OLED_ShowLineString(2, 1, buf);
    OLED_ShowLineString(3, 1,
        (t2_finish_ticks_10ms != 0U) ? "[STOPPED]       " : "[RUNNING]       ");
    OLED_ShowLineString(4, 1, "                ");
}

static void T3_OLED(void)  { T3Task_OLED(); }

static void T4_OLED(void)
{
    char buf[22];
    uint32_t shown_ticks = (t4_finish_ticks_10ms != 0U) ?
        t4_finish_ticks_10ms : t4_elapsed_ticks_10ms;
    int16_t ball_x_mm = T3Task_GetBallXMM();

    OLED_ShowLineString(1, 1, "T4 AB Center    ");
    snprintf(buf, sizeof(buf), "t:%2lu.%02lus d:%3d",
             (unsigned long)(shown_ticks / 100U),
             (unsigned long)(shown_ticks % 100U),
             (int)g_Encoder.distance_cm);
    OLED_ShowLineString(2, 1, buf);
    if (T3Task_HasValidVision()) {
        snprintf(buf, sizeof(buf), "x:%+4dmm       ", (int)ball_x_mm);
    } else {
        snprintf(buf, sizeof(buf), "x:----mm       ");
    }
    OLED_ShowLineString(3, 1, buf);
    OLED_ShowLineString(4, 1,
        (shown_ticks <= T4_MAX_TIME_TICKS_10MS) ? "time OK         " : "time >8s       ");
}

static void T5_OLED(void)
{
    char buf[22];
    uint32_t shown_ticks = (t5_finish_ticks_10ms != 0U) ?
        t5_finish_ticks_10ms : t5_elapsed_ticks_10ms;
    int16_t ball_x_mm = T3Task_GetBallXMM();

    OLED_ShowLineString(1, 1, "T5 Lap Center   ");
    snprintf(buf, sizeof(buf), "t:%2lu.%02lus d:%3d",
             (unsigned long)(shown_ticks / 100U),
             (unsigned long)(shown_ticks % 100U),
             (int)g_Encoder.distance_cm);
    OLED_ShowLineString(2, 1, buf);
    if (T3Task_HasValidVision()) {
        snprintf(buf, sizeof(buf), "x:%+4dmm       ", (int)ball_x_mm);
    } else {
        snprintf(buf, sizeof(buf), "x:----mm       ");
    }
    OLED_ShowLineString(3, 1, buf);
    OLED_ShowLineString(4, 1,
        (shown_ticks <= T5_MAX_TIME_TICKS_10MS) ? "time OK         " : "time >30s      ");
}

static void T6_OLED(void)
{
    char buf[22];
    uint32_t shown_ticks = (t6_finish_ticks_10ms != 0U) ?
        t6_finish_ticks_10ms : t6_elapsed_ticks_10ms;
    int16_t ball_x_mm = T3Task_GetBallXMM();

    OLED_ShowLineString(1, 1, "T6 Lap Target   ");
    snprintf(buf, sizeof(buf), "t:%2lu.%02lus d:%3d",
             (unsigned long)(shown_ticks / 100U),
             (unsigned long)(shown_ticks % 100U),
             (int)g_Encoder.distance_cm);
    OLED_ShowLineString(2, 1, buf);
    if (T3Task_HasValidVision()) {
        snprintf(buf, sizeof(buf), "T%+4d X%+4dmm", menu.target_mm,
                 (int)ball_x_mm);
    } else {
        snprintf(buf, sizeof(buf), "T%+4d X----mm", menu.target_mm);
    }
    OLED_ShowLineString(3, 1, buf);
    OLED_ShowLineString(4, 1,
        (shown_ticks <= T6_MAX_TIME_TICKS_10MS) ? "time OK         " : "time >30s      ");
}

static void T6_SetupOLED(void)
{
    char buf[22];
    const char *mode = "SIGN";

    switch (t6_setup_mode) {
        case T6_SETUP_CM:
            mode = "CM";
            break;
        case T6_SETUP_TENTH:
            mode = "0.1";
            break;
        case T6_SETUP_SIGN:
        default:
            mode = "SIGN";
            break;
    }

    snprintf(buf, sizeof(buf), "T6 SET %-8s", mode);
    OLED_ShowLineString(1, 1, buf);
    snprintf(buf, sizeof(buf), "target:%+3d.%1dcm",
             (int)(menu.target_mm / 10),
             (int)((menu.target_mm < 0 ? -menu.target_mm : menu.target_mm) % 10));
    OLED_ShowLineString(2, 1, buf);
    OLED_ShowLineString(3, 1, "short edit      ");
    OLED_ShowLineString(4, 1,
        (t6_setup_mode == T6_SETUP_TENTH) ? "long run        " : "long next       ");
}

static const TaskDef task_table[] = {
    [TASK_T1] = { "T1 Ball Tune",   TASK_T1, T1_Init, T1_Run, T1_Stop, T1_OLED, false },
    [TASK_T2] = { "T2 Lap Stop",    TASK_T2, T2_Init, T2_Run, T2_Stop, T2_OLED, true  },
    [TASK_T3] = { "T3 Ball Static", TASK_T3, T3_Init, T3_Run, T3_Stop, T3_OLED, false },
    [TASK_T4] = { "T4 AB Center",   TASK_T4, T4_Init, T4_Run, T4_Stop, T4_OLED, true  },
    [TASK_T5] = { "T5 Lap Center",  TASK_T5, T5_Init, T5_Run, T5_Stop, T5_OLED, true  },
    [TASK_T6] = { "T6 Lap Target",  TASK_T6, T6_Init, T6_Run, T6_Stop, T6_OLED, true  },
};

static const TaskID key_task_map[KEY_COUNT] = {
    [KEY_IDX_K1] = TASK_T1,
    [KEY_IDX_K2] = TASK_T2,
    [KEY_IDX_K3] = TASK_T3,
    [KEY_IDX_K4] = TASK_T4,
    [KEY_IDX_K5] = TASK_T5,
    [KEY_IDX_K6] = TASK_T6,
};

static const char *state_names[] = {
    [SYS_MENU]    = "[MENU]",
    [SYS_READY]   = "[READY]",
    [SYS_RUNNING] = "[RUNNING]",
    [SYS_STOPPED] = "[STOPPED]",
    [SYS_FAULT]   = "[FAULT]",
};

static void Key_SetGPIOInput(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(
        iomux,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void PB13_SetGPIO(void)
{
    Key_SetGPIOInput(KEY_K2_IOMUX);
}

static void PB13_SetSensorMuxOutput(void)
{
    DL_GPIO_initDigitalOutput(GPIO_SENSOR_AD2_IOMUX);
    DL_GPIO_enableOutput(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
    DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
}

#define KEY_PRESSED(port, pin)  (DL_GPIO_readPins(port, pin) == 0U)

static bool Key_IsPressed(uint8_t idx)
{
    switch (idx) {
        case KEY_IDX_K1: return KEY_PRESSED(KEY_K1_PORT, KEY_K1_PIN);
        case KEY_IDX_K2: return KEY_PRESSED(KEY_K2_PORT, KEY_K2_PIN);
        case KEY_IDX_K3: return KEY_PRESSED(KEY_K3_PORT, KEY_K3_PIN);
        case KEY_IDX_K4: return KEY_PRESSED(KEY_K4_PORT, KEY_K4_PIN);
        case KEY_IDX_K5: return KEY_PRESSED(KEY_K5_PORT, KEY_K5_PIN);
        case KEY_IDX_K6: return KEY_PRESSED(KEY_K6_PORT, KEY_K6_PIN);
        default:         return false;
    }
}

void KeyMenu_Init(void)
{
    Key_SetGPIOInput(KEY_K1_IOMUX);
    PB13_SetGPIO();
    Key_SetGPIOInput(KEY_K3_IOMUX);
    Key_SetGPIOInput(KEY_K4_IOMUX);
    Key_SetGPIOInput(KEY_K5_IOMUX);
    Key_SetGPIOInput(KEY_K6_IOMUX);

    menu.state          = SYS_MENU;
    menu.task_id        = TASK_ID_DEFAULT;
    menu.target_mm      = 0;
    menu.boot_ticks     = 0;
    menu.startup_window = true;

    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        menu.keys[i].pressed     = false;
        menu.keys[i].short_flag  = false;
        menu.keys[i].long_flag   = false;
        menu.keys[i].press_ticks = 0U;
    }
}

void KeyMenu_StartTask(TaskID task_id)
{
    if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)) {
        return;
    }

    if (menu.state == SYS_RUNNING) {
        const TaskDef *old_task = KeyMenu_GetCurrentTask();
        if (old_task && old_task->stop) {
            old_task->stop();
        }
        PB13_SetGPIO();
    }

    menu.task_id = task_id;
    Start_SelectedTask();
}

static void Key_ScanOne(KeyState *k, bool is_pressed)
{
    if (is_pressed) {
        if (k->press_ticks < 0xFFFFU) {
            k->press_ticks++;
        }
        if (k->press_ticks == KEY_LONG_TICKS) {
            k->long_flag = true;
            k->press_ticks = 0U;
        }
    } else {
        if (k->press_ticks > 0U && k->press_ticks < KEY_LONG_TICKS) {
            k->short_flag = true;
        }
        k->press_ticks = 0U;
    }
    k->pressed = is_pressed;
}

static bool Key_ConsumeShort(uint8_t idx)
{
    if (idx >= KEY_COUNT) {
        return false;
    }
    if (menu.keys[idx].short_flag) {
        menu.keys[idx].short_flag = false;
        return true;
    }
    return false;
}

static bool Key_ConsumeLong(uint8_t idx)
{
    if (idx >= KEY_COUNT) {
        return false;
    }
    if (menu.keys[idx].long_flag) {
        menu.keys[idx].long_flag = false;
        return true;
    }
    return false;
}

static void Key_FlushAll(void)
{
    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        menu.keys[i].short_flag = false;
        menu.keys[i].long_flag = false;
        menu.keys[i].press_ticks = 0U;
    }
}

static void Key_ClearOne(uint8_t idx)
{
    if (idx < KEY_COUNT) {
        menu.keys[idx].pressed = false;
        menu.keys[idx].short_flag = false;
        menu.keys[idx].long_flag = false;
        menu.keys[idx].press_ticks = 0U;
    }
}

static bool Key_StartMappedTask(uint8_t idx)
{
    if (idx >= KEY_COUNT) {
        return false;
    }

    if (Key_ConsumeShort(idx)) {
        if (key_task_map[idx] == TASK_T6) {
            T6_SetupStart();
            return true;
        }
        KeyMenu_StartTask(key_task_map[idx]);
        return true;
    }

    return false;
}

static bool Key_StartAnyMappedTask(void)
{
    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        if (Key_StartMappedTask(i)) {
            return true;
        }
    }
    return false;
}

static bool Key_ConsumeCurrentTaskShort(void)
{
    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        if (key_task_map[i] == menu.task_id) {
            return Key_ConsumeShort(i);
        }
    }
    return false;
}

static bool Key_ShouldScanWhileRunning(uint8_t idx)
{
    if (idx >= KEY_COUNT) {
        return false;
    }
    return (key_task_map[idx] == menu.task_id);
}

static void Key_StopCurrentTask(void)
{
    const TaskDef *task = KeyMenu_GetCurrentTask();

    if (task && task->stop) {
        task->stop();
    }
    PB13_SetGPIO();
    menu.state = SYS_STOPPED;
    Key_FlushAll();
}

static void Target_Clamp(void)
{
    if (menu.target_mm < TARGET_MIN_MM) {
        menu.target_mm = TARGET_MIN_MM;
    }
    if (menu.target_mm > TARGET_MAX_MM) {
        menu.target_mm = TARGET_MAX_MM;
    }
}

static void Start_SelectedTask(void)
{
    const TaskDef *task = KeyMenu_GetCurrentTask();

    if (task == NULL) {
        return;
    }

    Target_Clamp();
    if (task->needs_sensor) {
        PB13_SetSensorMuxOutput();
    } else {
        PB13_SetGPIO();
    }
    if (task->init) {
        task->init();
    }
    menu.state = SYS_RUNNING;
    Key_FlushAll();
}

static void FSM_Menu(void)
{
    (void)Key_StartAnyMappedTask();
}

static void FSM_Ready(void)
{
    (void)Key_StartAnyMappedTask();
}

static void FSM_Running(void)
{
    if (Key_ConsumeCurrentTaskShort()) {
        if ((menu.task_id == TASK_T6) && t6_skip_next_stop_short) {
            t6_skip_next_stop_short = false;
            return;
        }
        Key_StopCurrentTask();
    }
}

static void FSM_Stopped(void)
{
    (void)Key_StartAnyMappedTask();
}

static void FSM_Fault(void)
{
    if (Key_ConsumeLong(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
    }
}

static void KeyMenu_ShowWaiting(void)
{
    OLED_ShowLineString(1, 1, "waiting         ");
    OLED_ShowLineString(2, 1, "                ");
    OLED_ShowLineString(3, 1, "                ");
    OLED_ShowLineString(4, 1, "                ");
}

static bool KeyMenu_ShouldRenderStoppedTaskOLED(const TaskDef *task)
{
    return (menu.state == SYS_STOPPED) &&
           (task != NULL) &&
           (task->id == TASK_T2) &&
           (t2_finish_ticks_10ms != 0U);
}

void KeyMenu_Scan(void)
{
    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        if (menu.state == SYS_RUNNING) {
            if (!Key_ShouldScanWhileRunning(i)) {
                Key_ClearOne(i);
                continue;
            }
        }
        Key_ScanOne(&menu.keys[i], Key_IsPressed(i));
    }

    if (menu.boot_ticks < 0xFFFFU) {
        menu.boot_ticks++;
    }
    if (menu.boot_ticks >= KEY_STARTUP_TICKS) {
        menu.startup_window = false;
    }

    if (t6_setup_active) {
        T6_SetupHandleKeys();
        return;
    }

    if ((menu.state == SYS_RUNNING) &&
        (menu.task_id == TASK_T2) &&
        (t2_finish_ticks_10ms == 0U) &&
        (t2_elapsed_ticks_10ms < 0xFFFFFFFFU)) {
        t2_elapsed_ticks_10ms++;
    }
    if ((menu.state == SYS_RUNNING) &&
        (menu.task_id == TASK_T4) &&
        (t4_finish_ticks_10ms == 0U) &&
        (t4_elapsed_ticks_10ms < 0xFFFFFFFFU)) {
        t4_elapsed_ticks_10ms++;
    }
    if ((menu.state == SYS_RUNNING) &&
        (menu.task_id == TASK_T5) &&
        (t5_finish_ticks_10ms == 0U) &&
        (t5_elapsed_ticks_10ms < 0xFFFFFFFFU)) {
        t5_elapsed_ticks_10ms++;
    }
    if ((menu.state == SYS_RUNNING) &&
        (menu.task_id == TASK_T6) &&
        (t6_finish_ticks_10ms == 0U) &&
        (t6_elapsed_ticks_10ms < 0xFFFFFFFFU)) {
        t6_elapsed_ticks_10ms++;
    }

    switch (menu.state) {
        case SYS_MENU:    FSM_Menu();    break;
        case SYS_READY:   FSM_Ready();   break;
        case SYS_RUNNING: FSM_Running(); break;
        case SYS_STOPPED: FSM_Stopped(); break;
        case SYS_FAULT:   FSM_Fault();   break;
        default:          menu.state = SYS_MENU; break;
    }
}

void KeyMenu_OLED(void)
{
    char buf[22];
    const TaskDef *task = KeyMenu_GetCurrentTask();

    if (t6_setup_active) {
        T6_SetupOLED();
        return;
    }

    if ((menu.state != SYS_RUNNING) && !KeyMenu_ShouldRenderStoppedTaskOLED(task)) {
        KeyMenu_ShowWaiting();
        return;
    }

    if ((task != NULL) && (task->oled != NULL)) {
        task->oled();
        return;
    }

    if (task != NULL) {
        snprintf(buf, sizeof(buf), "%-16s", task->name);
    } else {
        snprintf(buf, sizeof(buf), "?               ");
    }
    OLED_ShowLineString(1, 1, buf);

    snprintf(buf, sizeof(buf), "target: %4d mm", menu.target_mm);
    OLED_ShowLineString(2, 1, buf);

    OLED_ShowLineString(3, 1, state_names[menu.state]);
    OLED_ShowLineString(4, 1, "");
}

SysState KeyMenu_GetState(void)
{
    return menu.state;
}

TaskID KeyMenu_GetTaskID(void)
{
    return menu.task_id;
}

int16_t KeyMenu_GetTargetMM(void)
{
    return menu.target_mm;
}

void KeyMenu_SetFault(void)
{
    if (menu.state == SYS_RUNNING) {
        const TaskDef *task = KeyMenu_GetCurrentTask();
        if (task && task->stop) {
            task->stop();
        }
        PB13_SetGPIO();
    }
    menu.state = SYS_FAULT;
    Key_FlushAll();
}

const TaskDef *KeyMenu_GetCurrentTask(void)
{
    if (menu.task_id >= TASK_ID_MIN && menu.task_id <= TASK_ID_MAX) {
        return &task_table[menu.task_id];
    }
    return NULL;
}
