from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def text(name):
    return (ROOT / name).read_text(encoding="utf-8")


def main():
    empty_c = text("empty.c")
    board_config = text("board_config.h")
    syscfg = text("empty.syscfg")

    require('#include "vision_uart.h"' in empty_c, "empty.c must include vision_uart.h")
    require("VisionUart_Init();" in empty_c, "empty.c must initialize VisionUart")
    require("VisionUart_Poll(control_ticks_10ms);" in empty_c, "empty.c must poll VisionUart")
    require("VISION_UART_INST_INT_IRQN" in empty_c, "empty.c must configure vision UART IRQ")

    require("#define VISION_UART_INST" in board_config, "board_config must define vision UART instance fallback")
    require("UART0" in board_config, "vision UART fallback must use UART0")

    require('$name                            = "VISION_UART"' in syscfg, "syscfg must add VISION_UART")
    require('UART3  = UART.addInstance();' in syscfg, "syscfg must add a third UART instance")
    require('UART3.peripheral.$assign               = "UART0";' in syscfg, "VISION_UART must use UART0")
    require('UART3.peripheral.txPin.$assign         = "PA0";' in syscfg, "VISION_UART TX must use PA0")
    require('UART3.peripheral.rxPin.$assign         = "PA1";' in syscfg, "VISION_UART RX must use PA1")


if __name__ == "__main__":
    main()
