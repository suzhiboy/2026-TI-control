from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def macro_int(source, name):
    match = re.search(r"#define\s+" + re.escape(name) + r"\s+\((\d+)U\)", source)
    require(match is not None, f"{name} must be defined as an unsigned integer macro")
    return int(match.group(1))


def main():
    empty_c = text("empty.c")
    t3_task_c = text("t3_task.c")
    t3_task_h = text("t3_task.h")
    balance_control_h = text("balance_control.h")
    balance_control = text("balance_control.c")
    board_config = text("board_config.h")
    key_menu_h = text("key_menu.h")
    key_menu_c = text("key_menu.c")
    line_follow_h = text("line_follow.h")
    line_follow_c = text("line_follow.c")
    encoder_h = text("encoder.h")
    syscfg = text("empty.syscfg")

    require('#include "vision_uart.h"' in empty_c, "empty.c must include vision_uart.h")
    require("VisionUart_Init();" in empty_c, "empty.c must initialize VisionUart")
    require("VisionUart_Poll(control_ticks_10ms);" in empty_c, "empty.c must poll VisionUart")
    require("VISION_UART_INST_INT_IRQN" in empty_c, "empty.c must configure vision UART IRQ")
    require("APP_AUTO_START_TASK" in empty_c,
            "empty.c must expose a generic optional auto-start task switch")
    require("KeyMenu_StartTask((TaskID)APP_AUTO_START_TASK);" in empty_c,
            "empty.c must keep optional auto-start routed through the key menu")
    require("PD42S1_LockCenter();" in empty_c.split("while (1)")[0],
            "empty.c must lock PD42S1 at center during startup before vision is valid")
    require("PD42S1_SoftLockCenter();" in empty_c,
            "empty.c must slew back to center instead of instant reset")
    timer_irq = empty_c.split("void TIMER_0_INST_IRQHandler", 1)[1]
    require("PD42S1_LockCenter();" not in timer_irq,
            "timer control path must not hard-jump PD42S1 back to center")
    require("DL_Timer_stopCounter(PD42S1_PWM_INST);" not in empty_c,
            "empty.c must keep PD42S1 PWM enabled for center lock when vision is invalid")
    require("PD42S1_SoftLockCenter();" in empty_c and
            "BalanceControl_Run(&bc);" in empty_c,
            "empty.c must soft center-lock without vision and run balance only with valid vision")
    require("last_task_run_tick" in empty_c, "empty.c must rate-limit task run to control ticks")
    vofa_window = empty_c.split(
        "if ((uint32_t)(now - last_vofa_tick) >= 2U)", 1)[1].split(
        "if ((uint32_t)(now - last_oled_tick)", 1)[0]
    require("if (!T3Task_IsActive())" in vofa_window and
            "Vofa_SendTelemetry();" in vofa_window,
            "T3 control window must disable blocking VOFA telemetry")
    main_loop = empty_c.split("while (1)", 1)[1].split(
        "void TIMER_0_INST_IRQHandler", 1)[0]
    require("OLED_RequestRefresh();" in main_loop and
            "OLED_RefreshStep();" in main_loop,
            "runtime OLED updates must be incremental")
    require("OLED_Refresh();" not in main_loop,
            "T3 runtime must not use blocking full-screen OLED refresh")
    require("#define APP_OLED_PERIOD_TICKS      (25U)" in empty_c and
            "now - last_oled_tick) >= APP_OLED_PERIOD_TICKS" in empty_c,
            "runtime OLED refresh requests must be slow enough to read T3 X/P telemetry")
    require("last_vision_seq" in empty_c and
            "BalanceControl_SetRawPositionTimed" in empty_c,
            "empty.c must submit only new timestamped vision frames")
    require("T3Task_UpdateVision(g_vision_ball.valid, g_vision_ball.timed_out," in empty_c and
            "g_vision_ball.seq" in empty_c,
            "empty.c must pass vision status and sequence into T3")
    require("T3_REF_RAMP_MM_PER_TICK" not in t3_task_h,
            "T3 must not delay reversal with a multi-second reference ramp")
    require("#define T3_ARRIVAL_CONFIRM_TICKS   (2U)" in t3_task_h,
            "T3 arrival confirmation must avoid visible edge delay")
    require("#define T3_FINAL_STABLE_TICKS      (10U)" in t3_task_h,
            "T3 keeps the current final stability tuning parameter available")
    require("#define T3_VISION_FRESH_TICKS      (20U)" in t3_task_h,
            "T3 must reject a final position held only by a stale vision frame")
    require("#define T3_TARGET_TOLERANCE_MM     (10)" in t3_task_h and
            "#define T3_FREEZE_TOLERANCE_MM     (5)" in t3_task_h,
            "T3 must separate the problem tolerance from the narrower freeze tolerance")
    require("#define T3_STILL_FRAME_FREEZE_COUNT (4U)" in t3_task_h,
            "T3 final hold must freeze only after the configured identical fresh frame count")
    require("#define T3_FREEZE_RETURN_TICKS     (7U)" in t3_task_h and
            "#define T3_FREEZE_LEVEL_RETURN_TICKS (12U)" in t3_task_h and
            "#define T3_FREEZE_HOLD_BIAS_PERCENT (75U)" in t3_task_h and
            "#define T3_FREEZE_HOLD_MIN_DELTA_US (45U)" in t3_task_h,
            "T3 final freeze must use a biased settle pulse before leveling to horizontal")
    require("#define T3_TRAVEL_NEGATIVE_DELTA_LIMIT_US (350U)" in t3_task_h,
            "T3 travel must preserve the current negative PWM authority")
    require("#define T3_TRAVEL_POSITIVE_DELTA_LIMIT_US (250U)" in t3_task_h,
            "T3 travel must keep positive PWM authority explicit")
    require("#define T3_TRAVEL_MIN_DRIVE_US             (170U)" in t3_task_h,
            "T3 travel minimum drive must overcome the mechanism dead zone")
    require("#define T3_RETURN_NEGATIVE_DELTA_LIMIT_US  (125U)" in t3_task_h and
            "#define T3_RETURN_POSITIVE_DELTA_LIMIT_US  (125U)" in t3_task_h and
            "#define T3_RETURN_MIN_DRIVE_US             (50U)" in t3_task_h,
            "T3 negative return must use a slower profile than the positive travel phase")
    require("#define T3_NEGATIVE_CAPTURE_ENTRY_MM       (40)" in t3_task_h and
            "#define T3_CAPTURE_NEGATIVE_DELTA_LIMIT_US (80U)" in t3_task_h and
            "#define T3_CAPTURE_POSITIVE_DELTA_LIMIT_US (150U)" in t3_task_h and
            "#define T3_CAPTURE_MIN_DRIVE_US            (30U)" in t3_task_h,
            "T3 capture must keep enough positive PWM authority to cross 0mm")
    require("#define T3_CAPTURE_POS_KP                  (0.055f)" in t3_task_h and
            "#define T3_CAPTURE_POS_KD                  (0.06f)" in t3_task_h and
            "#define T3_CAPTURE_VEL_KP                  (0.032f)" in t3_task_h,
            "T3 capture profile must use stronger damping and softer position gain")
    require("#define T3_FINAL_NEGATIVE_DELTA_LIMIT_US   (65U)" in t3_task_h and
            "#define T3_FINAL_POSITIVE_DELTA_LIMIT_US   (65U)" in t3_task_h and
            "#define T3_FINAL_MIN_DRIVE_US              (0U)" in t3_task_h,
            "T3 final window must clamp target-near output below the observed 103us")
    require("#define T3_FINAL_POS_KP                    (0.045f)" in t3_task_h and
            "#define T3_FINAL_POS_KD                    (0.045f)" in t3_task_h and
            "#define T3_FINAL_VEL_KP                    (0.024f)" in t3_task_h,
            "T3 final window must use a soft high-damping hold profile")
    require("T3_TRAVEL_NEGATIVE_DELTA_LIMIT_US" in t3_task_c and
            "T3_TRAVEL_POSITIVE_DELTA_LIMIT_US" in t3_task_c and
            "T3_TRAVEL_MIN_DRIVE_US" in t3_task_c,
            "T3 travel profile must use T3-specific output authority")
    require("apply_return_profile" in t3_task_c and
            "T3_RETURN_POSITIVE_DELTA_LIMIT_US" in t3_task_c,
            "T3 must reduce the whole +5 cm to -5 cm return speed")
    require("apply_capture_profile" in t3_task_c and
            "capture_profile_active" in t3_task_c and
            "t3.x_mm <= T3_NEGATIVE_CAPTURE_ENTRY_MM" in t3_task_c,
            "T3 must latch the capture profile before the negative target")
    require("apply_final_profile" in t3_task_c and
            "final_profile_active" in t3_task_c and
            "T3_FINAL_POSITIVE_DELTA_LIMIT_US" in t3_task_c,
            "T3 must apply a separate low-output final window profile")
    require("T3_PHASE_TO_NEGATIVE" in t3_task_h and
            "T3_PHASE_FINAL_RETURN" in t3_task_h and
            "T3_PHASE_HOLD_NEGATIVE" in t3_task_h,
            "T3 must use continuous travel, center-return, and final hold phases")
    require("T3_PHASE_BRAKE_NEGATIVE" not in t3_task_h and
            "T3_PHASE_CENTER_LIFT_NEGATIVE" not in t3_task_h and
            "T3_PHASE_CENTER_RETURN_NEGATIVE" not in t3_task_h and
            "T3_PHASE_SETTLE_NEGATIVE" not in t3_task_h and
            "apply_pwm_override_delta" not in t3_task_c,
            "T3 must not reintroduce velocity-dependent open-loop brake or lift phases")
    require("has_reached_positive_tolerance" in t3_task_c and
            "is_inside_negative_tolerance" in t3_task_c and
            "is_inside_freeze_tolerance" in t3_task_c and
            "vision_sample_pending" in t3_task_c and
            "seq != t3.vision_seq" in t3_task_c and
            "T3_ARRIVAL_CONFIRM_TICKS" in t3_task_c,
            "T3 must use distinct positive frames and fresh-sequence state transitions")
    require("final_freeze_active" in t3_task_c and
            "still_frame_count" in t3_task_c and
            "still_x_mm" in t3_task_c and
            "BalanceControl_SetPwmOverride(t3.controller, true" in t3_task_c,
            "T3 final hold must support same-coordinate PWM freezing")
    t3_override_enable = re.search(
        r"BalanceControl_SetPwmOverride\s*\([^;]*?\btrue\b",
        t3_task_c,
        re.DOTALL,
    )
    require(t3_override_enable is not None and
            "begin_freeze_return" in t3_task_c and
            "t3.phase = T3_PHASE_FINAL_RETURN" in t3_task_c and
            "update_freeze_return" in t3_task_c and
            "freeze_bias_pulse_us" in t3_task_c and
            "T3_FREEZE_LEVEL_RETURN_TICKS" in t3_task_c and
            "T3_FREEZE_HOLD_MIN_DELTA_US" in t3_task_c and
            "t3.phase = T3_PHASE_HOLD_NEGATIVE" in t3_task_c and
            "apply_reference_mm((float)T3_NEGATIVE_TARGET_MM)" in t3_task_c,
            "T3 final hold must keep the negative reference active and finish at horizontal PWM")
    require("if (t3.phase == T3_PHASE_HOLD_NEGATIVE) {\n        return;" in t3_task_c and
            "if (t3.phase == T3_PHASE_FINAL_RETURN) {\n        update_freeze_return();\n        return;" in t3_task_c and
            "(t3.phase != T3_PHASE_FINAL_RETURN)" in t3_task_c,
            "T3 return/hold must keep PWM override active after same-coordinate completion")
    require("#define BC_PWM_DIRECTION_SIGN       (1.0f)" in balance_control_h,
            "balance PWM direction must match the current mechanism")
    require(macro_int(balance_control_h, "BC_PWM_NEGATIVE_DELTA_LIMIT_US") == 130,
            "default negative PWM limit must stay conservative outside T3 travel")
    require(macro_int(balance_control_h, "BC_PWM_POSITIVE_DELTA_LIMIT_US") == 250,
            "positive PWM must provide stronger reverse braking authority")
    require("BC_PWM_DELTA_LIMIT_US" not in balance_control_h,
            "balance PWM must not retain a hidden symmetric limit")
    require(macro_int(balance_control_h, "BC_PWM_MIN_DRIVE_US") == 130,
            "minimum drive must remain above the observed 100us dead zone")
    require("BC_PWM_ACCEL_LIMIT_US" not in balance_control_h and
            "BC_BRAKE_VEL_THRESHOLD_CM_S" not in balance_control_h,
            "acceleration and braking must not have separate hidden limits")
    require("#define BC_PWM_SLEW_LIMIT_US         (15U)" in balance_control_h,
            "balance control must rate-limit PWM changes")
    require("#define BC_PWM_REVERSE_SLEW_LIMIT_US (80U)" in balance_control_h,
            "balance control must allow faster PWM reversal for braking")
    require("#define BC_FULL_DRIVE_ERROR_CM      (6.0f)" in balance_control_h,
            "balance control must scale drive with larger position error")
    require("#define BC_RAD_TO_PWM_SCALE_DEFAULT (10000.0f)" in balance_control_h,
            "balance PWM scale must be large enough to move the ball")
    require("#define BC_ANGLE_MAX_RAD            (0.09f)" in balance_control_h,
            "balance angle limit must move the ball through static friction")
    require("#define BC_ACCEL_MAX_MS2            (0.7f)" in balance_control_h,
            "balance acceleration limit must stay conservative during T3 tuning")
    require("#define BC_DEFAULT_POS_KP           (0.12f)" in balance_control_h,
            "position gain must be soft enough to settle near 5 cm")
    require("#define BC_DEFAULT_POS_KD           (0.018f)" in balance_control_h,
            "position damping must reduce overshoot near 5 cm")
    require("#define BC_DEFAULT_VEL_KP           (0.006f)" in balance_control_h,
            "velocity damping must stay moderate with large PWM authority")
    require("#define BC_POSITION_LOOKAHEAD_S      (0.20f)" in balance_control_h,
            "ball controller must predict position for early braking")
    require(balance_control_h.count("#define BC_ACCEL_MAX_MS2") == 1,
            "balance acceleration limit must be defined in only one tuning location")
    require(balance_control_h.count("#define BC_DEFAULT_POS_KP") == 1,
            "position gain must be defined in only one tuning location")
    require(balance_control_h.count("#define BC_DEFAULT_POS_KD") == 1,
            "position damping must be defined in only one tuning location")
    require(balance_control_h.count("#define BC_DEFAULT_VEL_KP") == 1,
            "velocity damping must be defined in only one tuning location")
    require("BC_PWM_MIN_DRIVE_US" in balance_control and
            "BC_PWM_SLEW_LIMIT_US" in balance_control,
            "balance_control.c must apply minimum drive and PWM slew limiting")
    require("BalanceControl_SetOutputProfile" in balance_control_h and
            "BalanceControl_SetOutputProfile" in balance_control,
            "balance control must expose a runtime output profile for T3 travel")
    require("BalanceControl_SetPwmOverride" in balance_control_h and
            "pwm_override_enabled" in balance_control_h and
            "if (bc->pwm_override_enabled)" in balance_control,
            "balance control must retain its generic fixed PWM override support")
    require("BalanceControl_SetOutputProfile" in t3_task_c and
            "apply_travel_profile" in t3_task_c,
            "T3 must apply one continuous closed-loop travel profile")
    require("if (g_vision_ball.valid) {\n                    BalanceControl_Run(&bc);" in empty_c and
            "} else {\n                    PD42S1_SoftLockCenter();" in empty_c,
            "empty.c must not keep running balance control when vision is lost")
    require("void        (*oled)(void);" in key_menu_h,
            "each task definition must own an OLED layout callback")
    require(all(f"T{idx}_OLED" in key_menu_c for idx in range(1, 7)),
            "all six task slots must register task-specific OLED entry points")
    require("task->oled != NULL" in key_menu_c and "task->oled();" in key_menu_c,
            "key menu OLED rendering must dispatch through the current task")
    require("KeyMenu_ShouldRenderStoppedTaskOLED" in key_menu_c and
            "menu.state == SYS_STOPPED" in key_menu_c and
            "task->id == TASK_T2" in key_menu_c and
            "t2_finish_ticks_10ms != 0U" in key_menu_c,
            "T2 completed result page must stay visible after the task enters SYS_STOPPED")
    require("T3Task_OLED" in t3_task_h and "T3Task_OLED" in t3_task_c and
            '"X%c%03d P%c%03ld"' in t3_task_c and
            '"T%c%03d S%04u"' in t3_task_c and
            "T3Task_GetElapsedTicks10ms()" not in empty_c,
            "T3 diagnostics must live in the T3 task and show fixed-width X/P telemetry")
    require('"X%c%03d P%c%03ld"' not in empty_c and '"T%c%03d S%04u"' not in empty_c,
            "empty.c must not contain task-specific OLED layouts")

    require("#define VISION_UART_INST" in board_config, "board_config must define vision UART instance fallback")
    require("UART0" in board_config, "vision UART fallback must use UART0")

    require("#define KEY_COUNT           (6U)" in key_menu_h,
            "key menu must scan six independent keys")
    for idx in range(1, 7):
        require(f"KEY_IDX_K{idx}" in key_menu_h,
                f"key menu must define K{idx} index")
        require(f"TASK_T{idx}" in key_menu_h,
                f"key menu must expose TASK_T{idx}")
        require(f"KEY_K{idx}_PORT" in board_config and f"KEY_K{idx}_PIN" in board_config,
                f"board_config must define K{idx} port and pin")
        require(f"KEY_K{idx}_PORT" in key_menu_c and f"KEY_K{idx}_PIN" in key_menu_c,
                f"key scan must use configurable K{idx} port and pin macros")
        require(f'GPIO5.associatedPins[{idx - 1}].$name            = "K{idx}";' in syscfg,
                f"syscfg must expose K{idx} in GPIO_KEY")
    require("key_task_map[KEY_COUNT]" in key_menu_c,
            "key menu must keep direct K1..K6 to task mapping in one table")
    require("[KEY_IDX_K1] = TASK_T1" in key_menu_c and
            "[KEY_IDX_K6] = TASK_T6" in key_menu_c,
            "direct key map must bind K1..K6 to T1..T6")
    require("Key_StartMappedTask" in key_menu_c,
            "key menu must start a mapped task directly from a key event")
    require("Key_ConsumeCurrentTaskShort" in key_menu_c and
            "key_task_map[i] == menu.task_id" in key_menu_c and
            "Key_StopCurrentTask" in key_menu_c,
            "running direct mode must stop when the active task key is pressed again")
    require("Key_ConsumeLong(KEY_IDX_K1)" not in key_menu_c and
            "Key_ConsumeShort(KEY_IDX_K3)" not in key_menu_c,
            "running direct mode must not keep the old K1-long or K3-short stop shortcut")
    require("Key_ShouldScanWhileRunning" in key_menu_c and
            "Key_ClearOne(i);" in key_menu_c,
            "running direct mode must scan only stop-capable keys and clear other key events")
    require("menu.task_id = (TaskID)((int)menu.task_id + 1)" not in key_menu_c and
            "menu.task_id = (TaskID)((int)menu.task_id - 1)" not in key_menu_c,
            "direct six-key mode must not use old task increment/decrement selection")
    expected_key_pins = {
        1: ("GPIOB", "DL_GPIO_PIN_14", "PB14"),
        2: ("GPIOB", "DL_GPIO_PIN_11", "PB11"),
        3: ("GPIOB", "DL_GPIO_PIN_10", "PB10"),
        4: ("GPIOB", "DL_GPIO_PIN_1", "PB1"),
        5: ("GPIOB", "DL_GPIO_PIN_0", "PB0"),
        6: ("GPIOA", "DL_GPIO_PIN_30", "PA30"),
    }
    for idx, (port, pin_macro, syscfg_pin) in expected_key_pins.items():
        require(f"#define KEY_K{idx}_PORT     {port}" in board_config and
                f"#define KEY_K{idx}_PIN      {pin_macro}" in board_config and
                f'GPIO5.associatedPins[{idx - 1}].pin.$assign      = "{syscfg_pin}";' in syscfg,
                f"K{idx} must use {syscfg_pin}")
    require("GPIO5.associatedPins.create(6);" in syscfg and
            'GPIO5.associatedPins[4].pin.$assign      = "PB0";' in syscfg and
            'GPIO5.associatedPins[5].pin.$assign      = "PA30";' in syscfg,
            "syscfg must expose K5 on PB0 and K6 on PA30")
    require("#define T2_LAP_DISTANCE_CM          (614.0f)" in key_menu_c,
            "T2 must use the 6.14 m lap length from the H problem statement")
    require("#define T2_SEARCH_START_CM          (560.0f)" in key_menu_c,
            "T2 must ignore the start line until the car is near A again")
    require("T2_StopLineDetected()" in key_menu_c and
            "T2_LINE_CONFIRM_TICKS" in key_menu_c,
            "T2 must confirm the perpendicular A stop line before braking")
    require("t2_finish_ticks_10ms = t2_elapsed_ticks_10ms" in key_menu_c,
            "T2 must freeze and display the final lap time when stopped")
    require("#define LINE_OPEN_LOOP_PWM_MODE      0" in line_follow_c,
            "Task 2 line following must use encoder speed PID, not diagnostic open-loop PWM")
    require("#define LINE_DEFAULT_ACCEL_STEP_CM_S (0.8f)" in line_follow_c and
            "#define LINE_DEFAULT_DECEL_STEP_CM_S (0.35f)" in line_follow_c and
            "#define LINE_DEFAULT_BRAKE_STEP_CM_S (6.0f)" in line_follow_c and
            "LineTrack_SetMotionProfile" in line_follow_h and
            "LineTrack_ResetMotionProfile" in line_follow_h and
            "line_speed_setpoint > line_base_speed" in line_follow_c and
            "line_speed_setpoint -= line_decel_step_cm_s" in line_follow_c,
            "line following must ramp target speed up and down with configurable motion steps")
    require("LineTrack_GetLongitudinalAccelMS2" in line_follow_h and
            "float LineTrack_GetLongitudinalAccelMS2(void)" in line_follow_c and
            "update_longitudinal_accel(previous_speed_setpoint);" in line_follow_c,
            "line following must expose setpoint acceleration for ball inertia feed-forward")
    require("#define T5_LAP_DISTANCE_CM          T2_LAP_DISTANCE_CM" in key_menu_c,
            "T5 must reuse the H problem lap length from T2")
    require("#define T5_SEARCH_START_CM          T2_SEARCH_START_CM" in key_menu_c,
            "T5 must ignore the start line until the car is near A again")
    require("#define T5_BASE_SPEED_CM_S          (20.0f)" in key_menu_c,
            "T5 must use the Task 4 center-hold driving speed as its starting point")
    require("#define T5_SLOWDOWN_START_CM        (614.0f)" in key_menu_c and
            "#define T5_FINAL_APPROACH_SPEED_CM_S (6.0f)" in key_menu_c,
            "T5 must keep full speed until odometry reaches 614 cm, then slow down")
    require("#define T5_ACCEL_STEP_CM_S          (0.25f)" in key_menu_c and
            "#define T5_DECEL_STEP_CM_S          (0.20f)" in key_menu_c and
            "#define T5_BRAKE_STEP_CM_S          (0.45f)" in key_menu_c,
            "T5 must use a slower start, approach, and brake motion profile")
    require("#define T5_STOP_COMPENSATION_CM     (13.0f)" in key_menu_c,
            "T5 must let the vehicle body pass A before braking")
    require("#define T5_MAX_TIME_TICKS_10MS      (3000U)" in key_menu_c,
            "T5 must display the 30 s requirement")
    require("T5_STATE_IGNORE_START_LINE" in key_menu_c and
            "T5_STATE_FIND_A_LINE" in key_menu_c and
            "T5_STATE_ADVANCE_TO_MARK" in key_menu_c and
            "T5_STATE_BRAKE" in key_menu_c,
            "T5 must have a full-lap state machine instead of an empty task")
    require("T5Task_StartCenterHold" not in key_menu_c,
            "T5 must use the existing T3 center-hold API directly, not a duplicate wrapper")
    require("static void T5_Init(void)\n{" in key_menu_c and
            "T3Task_StartCenterHold();" in key_menu_c.split("static void T5_Init", 1)[1].split("static void T5_Run", 1)[0] and
            "LineTrack_SetMotionProfile(T5_ACCEL_STEP_CM_S," in key_menu_c.split("static void T5_Init", 1)[1].split("static void T5_Run", 1)[0] and
            "LineTrack_Start(T5_BASE_SPEED_CM_S);" in key_menu_c.split("static void T5_Init", 1)[1].split("static void T5_Run", 1)[0] and
            "ControlState_Set(CONTROL_DYNAMIC_BALL);" in key_menu_c.split("static void T5_Init", 1)[1].split("static void T5_Run", 1)[0],
            "T5 init must combine Task 4 center hold with line tracking")
    require("static void T5_Run(void)\n{" in key_menu_c and
            "T5_UpdateApproachSpeed();" in key_menu_c.split("static void T5_Run", 1)[1].split("static void T5_Stop", 1)[0] and
            "T5_StopLineDetected()" in key_menu_c.split("static void T5_Run", 1)[1].split("static void T5_Stop", 1)[0] and
            "T5_RequestBrake();" in key_menu_c.split("static void T5_Run", 1)[1].split("static void T5_Stop", 1)[0] and
            "t5_finish_ticks_10ms = t5_elapsed_ticks_10ms" in key_menu_c,
            "T5 run must find A after one lap, brake, and freeze final time")
    require("static void T5_UpdateApproachSpeed(void)" in key_menu_c and
            "LineTrack_SetBaseSpeed(T5_FINAL_APPROACH_SPEED_CM_S);" in key_menu_c and
            "g_Encoder.distance_cm >= T5_SLOWDOWN_START_CM" in key_menu_c,
            "T5 must command a slow approach only after the 614 cm odometry mark")
    require("if ((menu.state == SYS_RUNNING) &&\n        (menu.task_id == TASK_T5)" in key_menu_c and
            "t5_elapsed_ticks_10ms++;" in key_menu_c,
            "KeyMenu_Scan must advance T5 time every 10 ms while running")
    require("#define T5_CAR_ACCEL_FF_GAIN       (2.0f)" in empty_c and
            "#define T5_CAR_ACCEL_FF_SIGN       (-1.0f)" in empty_c and
            "#define T5_CAR_ACCEL_FF_LIMIT_MS2  (1.20f)" in empty_c and
            "#define T5_CURVE_ACCEL_FF_GAIN     (0.06f)" in empty_c and
            "LineTrack_Get_TurnOut() * T5_CURVE_ACCEL_FF_GAIN" in empty_c and
            "static float T5_BuildCarAccelFeedforward(void)" in empty_c and
            "KeyMenu_GetTaskID() == TASK_T5" in empty_c and
            "BalanceControl_SetCarAccel(&bc, T5_BuildCarAccelFeedforward());" in empty_c,
            "T5 must use board-verified reversed inertia feed-forward versus the first T5 trial")
    require("T5Task_Stop();" not in key_menu_c and
            "static void T5_Stop(void)\n{" in key_menu_c and
            "T3Task_Stop();" in key_menu_c.split("static void T5_Stop", 1)[1].split("static void T6_Init", 1)[0] and
            "LineTrack_ResetMotionProfile();" in key_menu_c.split("static void T5_Stop", 1)[1].split("static void T6_Init", 1)[0] and
            "LineTrack_SetBaseSpeed(t5_restore_base_speed);" in key_menu_c.split("static void T5_Stop", 1)[1].split("static void T6_Init", 1)[0],
            "T5 stop must restore line speed and stop the center-hold controller")
    require("T5 Lap Center" in key_menu_c and
            "time OK" in key_menu_c.split("static void T5_OLED", 1)[1].split("static void T6_OLED", 1)[0] and
            "time >30s" in key_menu_c.split("static void T5_OLED", 1)[1].split("static void T6_OLED", 1)[0] and
            "T3Task_GetBallXMM()" in key_menu_c.split("static void T5_OLED", 1)[1].split("static void T6_OLED", 1)[0],
            "T5 OLED must show lap time and live ball center error")
    require("void T3Task_StartTargetHold(int16_t target_mm);" in t3_task_h and
            "void T3Task_StartTargetHold(int16_t target_mm)" in t3_task_c,
            "T3 must expose a reusable arbitrary target hold entry for Task 6")
    require("void T3Task_StartCenterHold(void)" in t3_task_c and
            "T3Task_StartTargetHold(0);" in t3_task_c,
            "T3 center hold must delegate to the arbitrary target hold path")
    require("#define T6_LAP_DISTANCE_CM          T5_LAP_DISTANCE_CM" in key_menu_c and
            "#define T6_SEARCH_START_CM          T5_SEARCH_START_CM" in key_menu_c and
            "#define T6_BASE_SPEED_CM_S          T5_BASE_SPEED_CM_S" in key_menu_c and
            "#define T6_MAX_TIME_TICKS_10MS      T5_MAX_TIME_TICKS_10MS" in key_menu_c,
            "T6 must reuse the full-lap timing constants from T5")
    require("T6_STATE_IGNORE_START_LINE" in key_menu_c and
            "T6_STATE_FIND_A_LINE" in key_menu_c and
            "T6_STATE_ADVANCE_TO_MARK" in key_menu_c and
            "T6_STATE_BRAKE" in key_menu_c,
            "T6 must have a full-lap state machine instead of an empty task")
    require("static void T6_Init(void)\n{" in key_menu_c and
            "T3Task_StartTargetHold(menu.target_mm);" in key_menu_c.split("static void T6_Init", 1)[1].split("static void T6_Run", 1)[0] and
            "LineTrack_Start(T6_BASE_SPEED_CM_S);" in key_menu_c.split("static void T6_Init", 1)[1].split("static void T6_Run", 1)[0] and
            "ControlState_Set(CONTROL_DYNAMIC_BALL);" in key_menu_c.split("static void T6_Init", 1)[1].split("static void T6_Run", 1)[0],
            "T6 init must combine arbitrary target hold with line tracking")
    require("static void T6_Run(void)\n{" in key_menu_c and
            "T6_StopLineDetected()" in key_menu_c.split("static void T6_Run", 1)[1].split("static void T6_Stop", 1)[0] and
            "T6_RequestBrake();" in key_menu_c.split("static void T6_Run", 1)[1].split("static void T6_Stop", 1)[0] and
            "t6_finish_ticks_10ms = t6_elapsed_ticks_10ms" in key_menu_c,
            "T6 run must find A after one lap, brake, and freeze final time")
    require("if ((menu.state == SYS_RUNNING) &&\n        (menu.task_id == TASK_T6)" in key_menu_c and
            "t6_elapsed_ticks_10ms++;" in key_menu_c,
            "KeyMenu_Scan must advance T6 time every 10 ms while running")
    require("static void T6_Stop(void)\n{" in key_menu_c and
            "T3Task_Stop();" in key_menu_c.split("static void T6_Stop", 1)[1].split("static void Task_OLED_BlankTail", 1)[0] and
            "LineTrack_SetBaseSpeed(t6_restore_base_speed);" in key_menu_c.split("static void T6_Stop", 1)[1].split("static void Task_OLED_BlankTail", 1)[0],
            "T6 stop must restore line speed and stop the arbitrary target controller")
    require("T6 Lap Target" in key_menu_c and
            '"T%+4d X%+4dmm"' in key_menu_c.split("static void T6_OLED", 1)[1].split("static const TaskDef", 1)[0] and
            "time OK" in key_menu_c.split("static void T6_OLED", 1)[1].split("static const TaskDef", 1)[0] and
            "time >30s" in key_menu_c.split("static void T6_OLED", 1)[1].split("static const TaskDef", 1)[0],
            "T6 OLED must show target, live ball coordinate, and 30 s status")
    require("#define LINE_TURN_DIRECTION_SIGN    (1.0f)" in line_follow_c and
            "line_turn_out = LINE_TURN_DIRECTION_SIGN * PID_Calc_Positional(&pid_line, line_error)" in line_follow_c,
            "Task 2 line steering correction must use the current car direction")
    require("ENCODER_RES       13.0f" in encoder_h and
            "GEAR_RATIO        20.0f" in encoder_h and
            "ENCODER_EDGE_MULT 2.0f" in encoder_h and
            "PULSE_TO_CM" in encoder_h,
            "encoder conversion must keep Wheeltec pulse-to-distance parameters explicit")

    require('$name                            = "VISION_UART"' in syscfg, "syscfg must add VISION_UART")
    require('UART3  = UART.addInstance();' in syscfg, "syscfg must add a third UART instance")
    require('UART3.peripheral.$assign               = "UART0";' in syscfg, "VISION_UART must use UART0")
    require('UART3.peripheral.txPin.$assign         = "PA0";' in syscfg, "VISION_UART TX must use PA0")
    require('UART3.peripheral.rxPin.$assign         = "PA1";' in syscfg, "VISION_UART RX must use PA1")


if __name__ == "__main__":
    main()
