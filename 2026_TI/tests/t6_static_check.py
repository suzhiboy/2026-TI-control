from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    key_menu_c = text("key_menu.c")
    t3_task_c = text("t3_task.c")
    t3_task_h = text("t3_task.h")

    require("void T3Task_StartTargetHold(int16_t target_mm);" in t3_task_h,
            "T3 task must declare arbitrary target hold API")
    require("void T3Task_StartTargetHold(int16_t target_mm)" in t3_task_c,
            "T3 task must implement arbitrary target hold API")
    require("void T3Task_StartCenterHold(void)" in t3_task_c and
            "T3Task_StartTargetHold(0);" in t3_task_c,
            "center hold must reuse arbitrary target hold")
    require("#define T6_LAP_DISTANCE_CM          T5_LAP_DISTANCE_CM" in key_menu_c,
            "T6 must reuse T5 lap geometry")
    require("T6_STATE_IGNORE_START_LINE" in key_menu_c and
            "T6_STATE_FIND_A_LINE" in key_menu_c and
            "T6_STATE_ADVANCE_TO_MARK" in key_menu_c,
            "T6 must have the full-lap A-line state machine")
    require("T3Task_StartTargetHold(menu.target_mm);" in key_menu_c,
            "T6 must hold the user selected target_mm")
    require("LineTrack_Start(T6_BASE_SPEED_CM_S);" in key_menu_c,
            "T6 must start line tracking while holding target")
    require("ControlState_Set(CONTROL_DYNAMIC_BALL);" in key_menu_c,
            "T6 must run line tracking and ball balance together")
    require("t6_elapsed_ticks_10ms++;" in key_menu_c,
            "T6 elapsed time must advance in KeyMenu_Scan")
    require("t6_finish_ticks_10ms = t6_elapsed_ticks_10ms" in key_menu_c,
            "T6 must freeze final lap time after braking")
    require('"T%+4d X%+4dmm"' in key_menu_c,
            "T6 OLED must show target and live ball coordinate")
    require("T6_SETUP_SIGN" in key_menu_c and
            "T6_SETUP_CM" in key_menu_c and
            "T6_SETUP_TENTH" in key_menu_c,
            "T6 must expose a three-stage one-key target setup state")
    require("static void T6_SetupStart(void)" in key_menu_c and
            "t6_setup_active = true;" in key_menu_c,
            "K6 must enter a target setup screen before starting T6")
    require("static void T6_SetupApplyTarget(void)" in key_menu_c and
            "menu.target_mm = (int16_t)(t6_setup_sign * magnitude_mm);" in key_menu_c,
            "T6 setup must calculate signed target_mm in 1 mm units")
    require("Key_ConsumeShort(KEY_IDX_K6)" in key_menu_c and
            "Key_ConsumeLong(KEY_IDX_K6)" in key_menu_c and
            "t6_setup_mode = T6_SETUP_CM;" in key_menu_c and
            "t6_setup_mode = T6_SETUP_TENTH;" in key_menu_c and
            "KeyMenu_StartTask(TASK_T6);" in key_menu_c,
            "single-key T6 setup must use K6 short to edit and K6 long to advance/start")
    require("if (key_task_map[idx] == TASK_T6)" in key_menu_c and
            "T6_SetupStart();" in key_menu_c,
            "K6 short press must open target setup instead of immediately launching T6")
    require("if (t6_setup_active) {\n        T6_SetupHandleKeys();\n        return;\n    }" in key_menu_c,
            "KeyMenu_Scan must route input to T6 setup before normal menu FSM")
    require("if (t6_setup_active) {\n        T6_SetupOLED();\n        return;\n    }" in key_menu_c,
            "KeyMenu_OLED must show the T6 setup page while selecting target")
    require("t6_setup_wait_release" in key_menu_c and
            "if (t6_setup_wait_release)" in key_menu_c and
            "t6_setup_wait_release = true;" in key_menu_c,
            "T6 setup must ignore the release edge after a long press")


if __name__ == "__main__":
    main()
