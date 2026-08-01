from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    h = text("balance_control.h")
    c = text("balance_control.c")
    t3_h = text("t3_task.h")
    t3_c = text("t3_task.c")

    for macro in (
        "BC_DEFAULT_POS_KI",
        "BC_DEFAULT_VEL_KI",
        "BC_DEFAULT_VEL_KD",
        "BC_POS_INTEGRAL_LIMIT_CM_S",
        "BC_VEL_INTEGRAL_LIMIT_CM",
        "BC_POS_INTEGRAL_DEADBAND_CM",
        "BC_VEL_INTEGRAL_DEADBAND_CM_S",
    ):
        require(f"#define {macro}" in h, f"{macro} must be defined")

    for field in (
        "pos_ki",
        "pos_integral",
        "vel_ki",
        "vel_kd",
        "vel_integral",
        "vel_prev_error",
    ):
        require(field in h, f"BalanceControl_t must expose {field}")
        require(re.search(rf"bc->{field}\s*=", c), f"{field} must be initialized or updated")

    require(
        "void BalanceControl_SetPositionPID(BalanceControl_t *bc, float kp, float ki, float kd);" in h,
        "position PID setter must be declared",
    )
    require(
        "void BalanceControl_SetVelocityPID(BalanceControl_t *bc, float kp, float ki, float kd);" in h,
        "velocity PID setter must be declared",
    )
    require(
        "void BalanceControl_ClearPidState(BalanceControl_t *bc);" in h,
        "PID state clear API must be declared",
    )

    require(
        "BalanceControl_SetPositionPID(bc, kp, 0.0f, kd);" in c,
        "SetPositionPD must delegate to SetPositionPID with zero I",
    )
    require(
        "BalanceControl_SetVelocityPID(bc, kp, 0.0f, 0.0f);" in c,
        "SetVelocityP must delegate to SetVelocityPID with zero I/D",
    )
    require(
        "BalanceControl_ClearPidState(bc);" in c.split("void BalanceControl_Reset", 1)[1],
        "BalanceControl_Reset must clear PID integrators",
    )

    run = c.split("void BalanceControl_Run", 1)[1]
    require("pos_error" in run and "bc->pos_integral" in run and "bc->pos_ki * bc->pos_integral" in run,
            "outer loop must use position integral contribution")
    require("vel_error" in run and "bc->vel_integral" in run and "bc->vel_ki * bc->vel_integral" in run,
            "inner loop must use velocity integral contribution")
    require("vel_derivative" in run and "bc->vel_kd * vel_derivative" in run,
            "inner loop must use velocity derivative contribution")
    require("fclamp(bc->pos_integral" in run and "BC_POS_INTEGRAL_LIMIT_CM_S" in run,
            "position integral must be clamped")
    require("fclamp(bc->vel_integral" in run and "BC_VEL_INTEGRAL_LIMIT_CM" in run,
            "velocity integral must be clamped")

    for macro in (
        "T3_CAPTURE_POS_KI",
        "T3_CAPTURE_VEL_KI",
        "T3_CAPTURE_VEL_KD",
        "T3_FINAL_POS_KI",
        "T3_FINAL_VEL_KI",
        "T3_FINAL_VEL_KD",
        "T3_CENTER_POS_KI",
        "T3_CENTER_VEL_KI",
        "T3_CENTER_VEL_KD",
    ):
        require(f"#define {macro}" in t3_h, f"{macro} must be available for board tuning")

    require("BalanceControl_SetPositionPID" in t3_c,
            "T3 profiles must call the position PID setter")
    require("BalanceControl_SetVelocityPID" in t3_c,
            "T3 profiles must call the velocity PID setter")


if __name__ == "__main__":
    main()
