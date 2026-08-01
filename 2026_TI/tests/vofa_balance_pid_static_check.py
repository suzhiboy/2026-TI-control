from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    vofa_h = text("vofa.h")
    vofa_c = text("vofa.c")
    proto_h = text("vofa_protocol.h")
    proto_c = text("vofa_protocol.c")
    empty_c = text("empty.c")

    require("VOFA_PID_BALANCE_POS" in proto_h,
            "VOFA protocol must expose a balance position PID group")
    require("VOFA_PID_BALANCE_VEL" in proto_h,
            "VOFA protocol must expose a balance velocity PID group")
    require('"BPOS_"' in proto_c and "VOFA_PID_BALANCE_POS" in proto_c,
            "VOFA parser must accept BPOS_KP/KI/KD")
    require('"BVEL_"' in proto_c and "VOFA_PID_BALANCE_VEL" in proto_c,
            "VOFA parser must accept BVEL_KP/KI/KD")

    require('#include "balance_control.h"' in vofa_h,
            "vofa.h must know BalanceControl_t for attachment")
    require("void Vofa_AttachBalanceControl(BalanceControl_t *controller);" in vofa_h,
            "VOFA must declare balance controller attachment")
    require("Vofa_AttachBalanceControl(&bc);" in empty_c,
            "empty.c must attach the balance controller before VOFA tuning")

    require('#include "balance_control.h"' in vofa_c,
            "vofa.c must include balance_control.h")
    require("static BalanceControl_t *vofa_balance_control" in vofa_c,
            "vofa.c must keep the attached balance controller")
    require("BalanceControl_SetPositionPID" in vofa_c,
            "VOFA must apply BPOS gains through BalanceControl_SetPositionPID")
    require("BalanceControl_SetVelocityPID" in vofa_c,
            "VOFA must apply BVEL gains through BalanceControl_SetVelocityPID")
    require("BalanceControl_ClearPidState(vofa_balance_control)" in vofa_c,
            "PIDCLR must clear balance PID state too")
    require('"#ACK SET_BPID' in vofa_c,
            "VOFA must acknowledge balance PID updates distinctly")
    require('"#BPID BPOS %.5f %.5f %.5f BVEL %.5f %.5f %.5f\\r\\n"' in vofa_c,
            "GET must print current balance PID values")


if __name__ == "__main__":
    main()
