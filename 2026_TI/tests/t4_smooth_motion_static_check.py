from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    empty_c = text("empty.c")
    key_menu_c = text("key_menu.c")
    line_follow_c = text("line_follow.c")
    line_follow_h = text("line_follow.h")
    t3_task_c = text("t3_task.c")
    t3_task_h = text("t3_task.h")

    require("void LineTrack_SetMotionProfile(float accel_step_cm_s," in line_follow_h,
            "line_follow must expose configurable accel/decel/brake ramp steps")
    require("void LineTrack_ResetMotionProfile(void);" in line_follow_h,
            "line_follow must expose a default motion profile restore API")
    require("float LineTrack_GetLongitudinalAccelMS2(void);" in line_follow_h,
            "line_follow must expose current longitudinal acceleration for ball feed-forward")
    require("line_accel_step_cm_s" in line_follow_c and
            "line_brake_step_cm_s" in line_follow_c and
            "line_longitudinal_accel_ms2" in line_follow_c,
            "line_follow must track configurable ramp steps and current acceleration")
    require("line_speed_setpoint -= line_brake_step_cm_s" in line_follow_c,
            "line braking must ramp down through the configured brake step")

    require("#define T4_BASE_SPEED_CM_S          (22.0f)" in key_menu_c,
            "T4 must have enough nominal speed margin for the 8 second limit")
    require("#define T4_DECEL_START_CM           (125.0f)" in key_menu_c and
            "#define T4_APPROACH_SPEED_CM_S      (12.0f)" in key_menu_c,
            "T4 must enter an approach speed before B instead of waiting until 150 cm")
    require("#define T4_ACCEL_STEP_CM_S" in key_menu_c and
            "#define T4_BRAKE_STEP_CM_S" in key_menu_c and
            "#define T4_BRAKE_HOLD_TICKS_10MS" in key_menu_c,
            "T4 must define its own slow start and slow brake parameters")
    require("LineTrack_SetMotionProfile(T4_ACCEL_STEP_CM_S," in key_menu_c,
            "T4 must switch line tracking to its slow motion profile on start")
    require("LineTrack_ResetMotionProfile();" in key_menu_c,
            "T4 must restore the default line tracking motion profile on finish/stop")
    require("T4_BRAKE_LEAD_CM" not in key_menu_c,
            "T4 must not brake before crossing scoring point B")
    require("T4_STATE_APPROACH_B" in key_menu_c and
            "LineTrack_SetBaseSpeed(T4_APPROACH_SPEED_CM_S);" in key_menu_c and
            "g_Encoder.distance_cm >= T4_DECEL_START_CM" in key_menu_c and
            "g_Encoder.distance_cm >= T4_AB_DISTANCE_CM" in key_menu_c,
            "T4 must slow to approach speed before B, then request brake at B")
    require("t4_brake_requested_tick = t4_elapsed_ticks_10ms;" in key_menu_c,
            "T4 must remember when the brake phase starts")
    require("T4_BRAKE_HOLD_TICKS_10MS" in key_menu_c and
            "LineTrack_IsRunning()" in key_menu_c,
            "T4 must keep the brake phase alive for a sustained settle window")

    for macro in (
        "T4_BALL_NEGATIVE_DELTA_LIMIT_US",
        "T4_BALL_POSITIVE_DELTA_LIMIT_US",
        "T4_BALL_MINIMUM_DRIVE_US",
        "T4_BALL_POSITION_KP",
        "T4_BALL_POSITION_KI",
        "T4_BALL_POSITION_KD",
        "T4_BALL_VELOCITY_KP",
        "T4_BALL_VELOCITY_KI",
        "T4_BALL_VELOCITY_KD",
    ):
        require(f"#define {macro}" in key_menu_c,
                f"T4 must expose its local {macro} parameter")
    require("static const T3TuneProfile t4_ball_profile" in key_menu_c,
            "T4 must own an independent ball-control profile")
    require("#define T4_BALL_NEGATIVE_DELTA_LIMIT_US (320U)" in key_menu_c and
            "#define T4_BALL_POSITIVE_DELTA_LIMIT_US (320U)" in key_menu_c,
            "T4 dynamic hold must have enough PWM headroom for start/brake feed-forward")
    require("#define T4_BALL_POSITION_KI              (0.0035f)" in key_menu_c,
            "T4 dynamic hold must include a very small integrator for moving bias")
    require("#define T4_BRAKE_MAX_HOLD_TICKS_10MS (220U)" in key_menu_c and
            "#define T4_BRAKE_SETTLE_ERROR_MM    (5)" in key_menu_c and
            "#define T4_BRAKE_SETTLE_TICKS_10MS  (20U)" in key_menu_c and
            "t4_brake_settle_ticks" in key_menu_c,
            "T4 must keep ball control active after stopping until the ball settles")
    require("T3Task_StartProfileHold(0, &t4_ball_profile);" in key_menu_c,
            "T4 must start center hold with its local ball profile")
    require("void T3Task_StartProfileHold(int16_t target_mm," in t3_task_h and
            "void T3Task_StartProfileHold(int16_t target_mm," in t3_task_c,
            "T3 must expose a generic profiled hold for scoring tasks")

    require("#define T4_VISION_LOST_BRAKE_TICKS  (10U)" in key_menu_c and
            "t4_vision_lost_ticks++" in key_menu_c and
            "t4_vision_lost_ticks >= T4_VISION_LOST_BRAKE_TICKS" in key_menu_c,
            "T4 must request controlled braking after 100 ms of lost vision")

    require("#define T4_CAR_ACCEL_FF_GAIN       (1.45f)" in empty_c and
            "#define T4_CAR_ACCEL_FF_LIMIT_MS2  (1.20f)" in empty_c,
            "dynamic ball control must define the board-tested stronger T4 feed-forward gain and limit")
    require("if (KeyMenu_GetTaskID() == TASK_T4)" in empty_c,
            "longitudinal acceleration feed-forward must be scoped to T4")
    require("accel_ms2 = -line_accel_ms2 * T4_CAR_ACCEL_FF_GAIN;" in empty_c,
            "T4 feed-forward sign must invert the line-accel direction")
    require("T4_CAR_ACCEL_HOLD_MS2" not in empty_c and
            "T4_CAR_ACCEL_START_HOLD_TICKS" not in empty_c and
            "T4_CAR_ACCEL_STOP_HOLD_TICKS" not in empty_c,
            "T4 must not apply long fixed feed-forward hold pulses")
    require("BalanceControl_SetCarAccel(&bc, T4_BuildCarAccelFeedforward());" in empty_c,
            "T4 must feed line tracking acceleration into balance control before BalanceControl_Run")
    require("BalanceControl_SetCarAccel(&bc, 0.0f);" in empty_c,
            "non-T4 paths must clear car acceleration feed-forward")


if __name__ == "__main__":
    main()
