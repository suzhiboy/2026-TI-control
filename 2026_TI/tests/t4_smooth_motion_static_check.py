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

    require("#define T4_ACCEL_STEP_CM_S" in key_menu_c and
            "#define T4_BRAKE_STEP_CM_S" in key_menu_c and
            "#define T4_BRAKE_LEAD_CM" in key_menu_c and
            "#define T4_BRAKE_HOLD_TICKS_10MS" in key_menu_c,
            "T4 must define its own slow start, slow brake, and brake lead parameters")
    require("LineTrack_SetMotionProfile(T4_ACCEL_STEP_CM_S," in key_menu_c,
            "T4 must switch line tracking to its slow motion profile on start")
    require("LineTrack_ResetMotionProfile();" in key_menu_c,
            "T4 must restore the default line tracking motion profile on finish/stop")
    require("g_Encoder.distance_cm >= (T4_AB_DISTANCE_CM - T4_BRAKE_LEAD_CM)" in key_menu_c,
            "T4 must request braking before B to allow a slow deceleration")
    require("t4_brake_requested_tick = t4_elapsed_ticks_10ms;" in key_menu_c,
            "T4 must remember when the brake phase starts")
    require("T4_BRAKE_HOLD_TICKS_10MS" in key_menu_c and
            "LineTrack_IsRunning()" in key_menu_c,
            "T4 must keep the brake phase alive for a sustained settle window")

    require("#define T4_CAR_ACCEL_FF_GAIN" in empty_c and
            "#define T4_CAR_ACCEL_FF_LIMIT_MS2" in empty_c and
            "#define T4_CAR_ACCEL_START_HOLD_TICKS" in empty_c and
            "#define T4_CAR_ACCEL_STOP_HOLD_TICKS" in empty_c,
            "dynamic ball control must define T4 feed-forward gain and limit")
    require("if (KeyMenu_GetTaskID() == TASK_T4)" in empty_c,
            "longitudinal acceleration feed-forward must be scoped to T4")
    require("BalanceControl_SetCarAccel(&bc, T4_BuildCarAccelFeedforward());" in empty_c,
            "T4 must feed line tracking acceleration into balance control before BalanceControl_Run")
    require("BalanceControl_SetCarAccel(&bc, 0.0f);" in empty_c,
            "non-T4 paths must clear car acceleration feed-forward")
    require("t4_start_hold_ticks = T4_CAR_ACCEL_START_HOLD_TICKS;" in empty_c and
            "t4_stop_hold_ticks = T4_CAR_ACCEL_STOP_HOLD_TICKS;" in empty_c,
            "T4 must keep startup and braking direction angles active for sustained settle windows")


if __name__ == "__main__":
    main()
