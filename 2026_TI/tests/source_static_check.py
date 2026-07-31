from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    empty_c = text("empty.c")
    t3_task_c = text("t3_task.c")
    t3_task_h = text("t3_task.h")
    balance_control_h = text("balance_control.h")
    balance_control = text("balance_control.c")
    board_config = text("board_config.h")
    syscfg = text("empty.syscfg")

    require('#include "vision_uart.h"' in empty_c, "empty.c must include vision_uart.h")
    require("VisionUart_Init();" in empty_c, "empty.c must initialize VisionUart")
    require("VisionUart_Poll(control_ticks_10ms);" in empty_c, "empty.c must poll VisionUart")
    require("VISION_UART_INST_INT_IRQN" in empty_c, "empty.c must configure vision UART IRQ")
    require("APP_AUTO_START_T3" in empty_c, "empty.c must expose the T3 auto-start switch")
    require("KeyMenu_StartTask(TASK_T3);" in empty_c, "empty.c must auto-start the menu in T3")
    require("KeyMenu_StartTask(TASK_T2);" not in empty_c, "empty.c must not auto-start T2")
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
    require("#define T3_REF_RAMP_MM_PER_TICK    (0.4f)" in t3_task_h,
            "T3 reference ramp must be quick enough while PWM remains limited")
    require("#define T3_ARRIVAL_CONFIRM_TICKS   (2U)" in t3_task_h,
            "T3 arrival confirmation must avoid visible edge delay")
    require("has_reached_target()" in t3_task_c and
            "t3.x_mm >=" in t3_task_c and
            "t3.x_mm <=" in t3_task_c,
            "T3 arrival must treat overshoot past the target as reached")
    require("#define BC_PWM_DIRECTION_SIGN       (-1.0f)" in balance_control_h,
            "balance PWM direction must match the current mechanism")
    require("#define BC_PWM_DELTA_LIMIT_US       (400U)" in balance_control_h,
            "balance PWM delta must have enough authority while staying limited")
    require("#define BC_PWM_MIN_DRIVE_US         (175U)" in balance_control_h,
            "balance control must include a minimum drive to overcome static friction")
    require("#define BC_PWM_SLEW_LIMIT_US        (10U)" in balance_control_h,
            "balance control must rate-limit PWM changes")
    require("#define BC_RAD_TO_PWM_SCALE_DEFAULT (10000.0f)" in balance_control_h,
            "balance PWM scale must be large enough to move the ball")
    require("#define BC_ANGLE_MAX_RAD            (0.04f)" in balance_control_h,
            "balance angle limit must move the ball without returning to aggressive tuning")
    require("#define BC_ACCEL_MAX_MS2            (0.7f)" in balance_control_h,
            "balance acceleration limit must stay conservative during T3 tuning")
    require("#define BC_DEFAULT_POS_KP           (0.12f)" in balance_control_h,
            "position gain must be soft enough to settle near 5 cm")
    require("#define BC_DEFAULT_POS_KD           (0.018f)" in balance_control_h,
            "position damping must reduce overshoot near 5 cm")
    require("#define BC_DEFAULT_VEL_KP           (0.006f)" in balance_control_h,
            "velocity damping must stay moderate with large PWM authority")
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
    require("if (g_vision_ball.valid) {\n                    BalanceControl_Run(&bc);" in empty_c and
            "} else {\n                    PD42S1_SoftLockCenter();" in empty_c,
            "empty.c must not keep running balance control when vision is lost")
    require('"T%+d X%+d P%+d"' in empty_c and
            '"T%+d TO P%+d"' in empty_c and
            '"T%+d L P%+d"' in empty_c,
            "empty.c must show T3 target, vision position, and PWM delta on OLED")

    require("#define VISION_UART_INST" in board_config, "board_config must define vision UART instance fallback")
    require("UART0" in board_config, "vision UART fallback must use UART0")

    require('$name                            = "VISION_UART"' in syscfg, "syscfg must add VISION_UART")
    require('UART3  = UART.addInstance();' in syscfg, "syscfg must add a third UART instance")
    require('UART3.peripheral.$assign               = "UART0";' in syscfg, "VISION_UART must use UART0")
    require('UART3.peripheral.txPin.$assign         = "PA0";' in syscfg, "VISION_UART TX must use PA0")
    require('UART3.peripheral.rxPin.$assign         = "PA1";' in syscfg, "VISION_UART RX must use PA1")


if __name__ == "__main__":
    main()
