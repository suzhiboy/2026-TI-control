from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    key_menu_c = (ROOT / "key_menu.c").read_text(encoding="utf-8")

    require("KeyMenu_ShouldRenderStoppedTaskOLED" in key_menu_c,
            "key menu must have a helper for stopped task OLED retention")
    require("menu.state == SYS_STOPPED" in key_menu_c,
            "stopped task OLED retention must only apply after SYS_STOPPED")
    require("task->id == TASK_T2" in key_menu_c,
            "only T2 should keep its completed result page in SYS_STOPPED")
    require("t2_finish_ticks_10ms != 0U" in key_menu_c,
            "T2 result page must be retained only after a real finish time is recorded")
    require("if ((menu.state != SYS_RUNNING) && !KeyMenu_ShouldRenderStoppedTaskOLED(task))" in key_menu_c,
            "waiting page must not override a completed T2 result page")


if __name__ == "__main__":
    main()
