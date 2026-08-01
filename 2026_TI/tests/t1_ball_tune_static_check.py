from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    key_menu_c = text("key_menu.c")
    t3_c = text("t3_task.c")
    t3_h = text("t3_task.h")

    require("T3_PHASE_TUNE_HOLD" in t3_h,
            "T3 must expose a pure tuning hold phase")
    require("typedef struct {" in t3_h and "} T3TuneProfile;" in t3_h,
            "T3 must expose the Task1 tuning profile type")
    require("void T3Task_StartTuneHold(int16_t target_mm, const T3TuneProfile *profile);" in t3_h,
            "T3 must expose a pure tuning hold entry")
    require("void T3Task_StartTuneHold(int16_t target_mm, const T3TuneProfile *profile)" in t3_c,
            "T3 pure tuning hold entry must be implemented")
    require("T3_PHASE_TUNE_HOLD: return \"TUN\";" in t3_c,
            "OLED phase label must distinguish pure tuning hold")

    tune_start = t3_c.split("void T3Task_StartTuneHold(int16_t target_mm, const T3TuneProfile *profile)", 1)[1].split("\n}", 1)[0]
    require("LineTrack_Stop();" in tune_start,
            "pure tuning mode must stop line tracking")
    require("ControlState_Set(CONTROL_STATIC_BALL);" in tune_start,
            "pure tuning mode must enter static ball control")
    require("BalanceControl_Reset(t3.controller);" in tune_start and
            "BalanceControl_Enable(t3.controller, true);" in tune_start,
            "pure tuning mode must reset and enable the ball controller")
    require("apply_center_profile();" not in tune_start,
            "Task1 tuning must not use the shared T3 center profile")
    require("BalanceControl_SetOutputProfile(t3.controller," in tune_start and
            "profile->negative_delta_limit_us" in tune_start and
            "profile->positive_delta_limit_us" in tune_start and
            "profile->minimum_drive_us" in tune_start,
            "Task1 tuning must apply its own output profile")
    require("BalanceControl_SetPositionPID(t3.controller," in tune_start and
            "profile->position_kp" in tune_start and
            "BalanceControl_SetVelocityPID(t3.controller," in tune_start and
            "profile->velocity_kp" in tune_start,
            "Task1 tuning must apply both local PID loops")
    require("BalanceControl_SetReference(t3.controller, t3.reference_mm / 10.0f);" in tune_start,
            "pure tuning mode must apply the requested hold target")

    run = t3_c.split("void T3Task_Run(void)", 1)[1].split("void T3Task_Stop", 1)[0]
    require("if (t3.phase == T3_PHASE_TUNE_HOLD) {\n        return;\n    }" in run,
            "pure tuning hold must bypass the Task 3 positive/negative state machine")

    t1_init = key_menu_c.split("static void T1_Init(void)", 1)[1].split("static void T1_Run", 1)[0]
    for macro in (
        "T1_TUNE_TARGET_MM",
        "T1_TUNE_NEGATIVE_DELTA_LIMIT_US",
        "T1_TUNE_POSITIVE_DELTA_LIMIT_US",
        "T1_TUNE_MINIMUM_DRIVE_US",
        "T1_TUNE_POSITION_KP",
        "T1_TUNE_POSITION_KI",
        "T1_TUNE_POSITION_KD",
        "T1_TUNE_VELOCITY_KP",
        "T1_TUNE_VELOCITY_KI",
        "T1_TUNE_VELOCITY_KD",
    ):
        require(f"#define {macro}" in key_menu_c,
                f"Task1 must expose {macro} beside its task code")
    require("static const T3TuneProfile t1_tune_profile" in key_menu_c,
            "Task1 must own a local tuning profile")
    require("T3Task_StartTuneHold(T1_TUNE_TARGET_MM, &t1_tune_profile);" in t1_init,
            "K1/T1 must start a center pure tuning hold")
    require("LineTrack_Start" not in t1_init,
            "K1/T1 must not start wheel line tracking")
    require("static void T1_Run(void)\n{\n    T3Task_Run();\n}" in key_menu_c,
            "K1/T1 must keep static ball control serviced")
    require("static void T1_Stop(void)\n{\n    T3Task_Stop();\n}" in key_menu_c,
            "K1/T1 stop must disable tuning hold")
    require('"T1 Ball Tune"' in key_menu_c,
            "K1/T1 must be labeled as the pure tuning mode")
    require("T1 Tune Center" in key_menu_c,
            "K1/T1 OLED must show the pure tuning page")


if __name__ == "__main__":
    main()
