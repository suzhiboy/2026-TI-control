from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    empty_c = (ROOT / "empty.c").read_text(encoding="utf-8")
    key_menu_c = (ROOT / "key_menu.c").read_text(encoding="utf-8")
    t3_task_h = (ROOT / "t3_task.h").read_text(encoding="utf-8")
    t3_task_c = (ROOT / "t3_task.c").read_text(encoding="utf-8")

    require("#define T5_BASE_SPEED_CM_S          (23.0f)" in key_menu_c and
            "#define T5_SAFE_SPEED_CM_S          (16.0f)" in key_menu_c and
            "#define T5_FINAL_APPROACH_SPEED_CM_S (8.0f)" in key_menu_c,
            "T5 must use the 23/16/8 cm/s nominal, protected, and final speeds")
    require("#define T5_ACCEL_STEP_CM_S          (0.20f)" in key_menu_c and
            "#define T5_DECEL_STEP_CM_S          (0.20f)" in key_menu_c and
            "#define T5_BRAKE_STEP_CM_S          (0.35f)" in key_menu_c,
            "T5 must use the approved smooth acceleration and braking steps")

    for expected in (
        "#define T5_BALL_NEGATIVE_DELTA_LIMIT_US (320U)",
        "#define T5_BALL_POSITIVE_DELTA_LIMIT_US (320U)",
        "#define T5_BALL_MINIMUM_DRIVE_US         (0U)",
        "#define T5_BALL_POSITION_KP              (0.05f)",
        "#define T5_BALL_POSITION_KI              (0.0035f)",
        "#define T5_BALL_POSITION_KD              (0.02f)",
        "#define T5_BALL_VELOCITY_KP              (0.005f)",
        "#define T5_BALL_VELOCITY_KI              (0.0f)",
        "#define T5_BALL_VELOCITY_KD              (0.0f)",
    ):
        require(expected in key_menu_c,
                f"T5 must own the tuned T1 ball-control value: {expected}")
    require("static const T3TuneProfile t5_ball_profile" in key_menu_c,
            "T5 must own an independent ball-control profile")

    t5_init = key_menu_c.split("static void T5_Init", 1)[1].split(
        "static void T5_Run", 1)[0]
    require("T3Task_StartProfileHold(0, &t5_ball_profile);" in t5_init,
            "T5 must start its independent tuned center-hold profile")
    require("T3Task_StartCenterHold();" not in t5_init and
            "T3Task_SetCenterOutputProfile" not in t5_init,
            "T5 must not inherit or partially override the shared T3 center profile")

    require("#define T5_BALL_ERROR_SLOWDOWN_MM   (6)" in key_menu_c and
            "#define T5_BALL_ERROR_RECOVERY_MM   (4)" in key_menu_c and
            "#define T5_BALL_RECOVERY_TICKS_10MS (20U)" in key_menu_c,
            "T5 must use hysteresis and a 200 ms recovery confirmation")
    require("T3Task_GetBallXMM()" in key_menu_c and
            "LineTrack_SetBaseSpeed(T5_SAFE_SPEED_CM_S);" in key_menu_c and
            "t5_ball_recovery_ticks" in key_menu_c,
            "T5 must reduce speed when ball error grows and recover only after stability")
    require("#define T5_VISION_LOST_BRAKE_TICKS  (10U)" in key_menu_c and
            "t5_vision_lost_ticks" in key_menu_c and
            "T5_RequestBrake();" in key_menu_c,
            "T5 must request controlled braking after 100 ms of vision loss")
    t5_safety = key_menu_c.split("static void T5_UpdateBallSafety", 1)[1].split(
        "static void T5_ApplyTargetSpeed", 1)[0]
    vision_lost_branch = t5_safety.split(
        "if (!T3Task_HasValidVision())", 1)[1].split(
            "t5_vision_lost_ticks = 0U;", 1)[0]
    require("t5_safe_speed_active = true;" in vision_lost_branch,
            "T5 must enter protected speed immediately when vision becomes invalid")
    require("t5_state == T5_STATE_ADVANCE_TO_MARK" in key_menu_c and
            "LineTrack_SetBaseSpeed(T5_FINAL_APPROACH_SPEED_CM_S);" in key_menu_c,
            "T5 must enter final approach speed only after A has been detected")

    require("#define T5_CAR_ACCEL_FF_SIGN       (-1.0f)" in empty_c,
            "T5 longitudinal feed-forward direction must keep the board-tested sign")
    require("#define T5_CAR_ACCEL_FF_GAIN       (1.45f)" in empty_c,
            "T5 longitudinal feed-forward gain must match the T4 dynamic baseline")
    require("#define T5_CURVE_ACCEL_FF_GAIN     (0.0f)" in empty_c,
            "T5 curve feed-forward must start disabled for reliable board migration")
    for obsolete in (
        "T5_CAR_ACCEL_HOLD_MS2",
        "T5_CAR_ACCEL_START_HOLD_TICKS",
        "T5_CAR_ACCEL_STOP_HOLD_TICKS",
        "T5_CAR_ACCEL_EDGE_DEADBAND_MS2",
        "t5_ff_session_active",
        "t5_start_hold_ticks",
        "t5_stop_hold_ticks",
        "t5_prev_line_accel_ms2",
    ):
        require(obsolete not in empty_c,
                f"T5 must remove obsolete fixed feed-forward hold state: {obsolete}")
    require("LineTrack_GetLongitudinalAccelMS2()" in empty_c and
            "LineTrack_Get_TurnOut() * T5_CURVE_ACCEL_FF_GAIN" in empty_c and
            "accel_ms2 = T5_CAR_ACCEL_FF_SIGN *" in empty_c,
            "T5 feed-forward must follow the speed ramp with optional zeroed curve input")

    require("void T3Task_StartProfileHold(int16_t target_mm," in t3_task_h and
            "void T3Task_StartProfileHold(int16_t target_mm," in t3_task_c,
            "T3 must expose the non-static profiled hold used by moving tasks")


if __name__ == "__main__":
    main()
