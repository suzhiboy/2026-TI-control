from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "k230" / "ball_position_uart.py"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    require(SCRIPT.exists(), "k230/ball_position_uart.py is missing")
    source = SCRIPT.read_text(encoding="utf-8")
    compile(source, str(SCRIPT), "exec")

    require("UART.UART2" in source, "K230 must use UART2")
    require("GPIO11" in source and "GPIO12" in source, "K230 UART pins must be documented")
    require("FPIOA.UART2_TXD" in source, "GPIO11 must map to UART2_TXD")
    require("FPIOA.UART2_RXD" in source, "GPIO12 must map to UART2_RXD")
    require("ROD_LEFT_MM = -120" in source, "left end must be -120 mm")
    require("ROD_RIGHT_MM = 120" in source, "right end must be +120 mm")
    require("BALL,{},{}" in source, "BALL output format must be present")
    require("BALL_LOST,{}" in source, "BALL_LOST output format must be present")
    require("K230_BALL seq=" in source, "K230 terminal must print ball coordinates")
    require("K230_BALL_LOST seq=" in source, "K230 terminal must print lost state")
    require("MAX_JUMP_MM = 30" in source, "jump rejection must be 30 mm")
    require("LOST_FRAME_LIMIT = 3" in source, "lost threshold must be 3 frames")
    require("TERMINAL_PRINT_EVERY_N_FRAMES = 30" in source, "terminal print must be throttled")
    require("LOST_SEND_EVERY_N_FRAMES = 5" in source, "lost state must be sent periodically")
    require("DEBUG_FRAME_STATUS = True" in source, "frame status debug output must be available")
    require("sent=" in source, "terminal output must show whether UART sent this frame")
    require("except KeyboardInterrupt:" in source, "K230 script must handle IDE stop")
    require("K230_STOP_BY_IDE" in source, "K230 script must print IDE stop status")
    require("K230_ERROR:" in source, "K230 script must print runtime errors")
    require("K230_RELEASE" in source, "K230 script must print resource release status")
    require("decode_canmv_yolo_result" in source, "K230 script must decode CanMV YOLO result format")
    require("boxes = result[0]" in source, "K230 script must read boxes from result[0]")
    require("scores = result[2]" in source, "K230 script must read scores from result[2]")
    require("ROD_LEFT_PX = (" in source, "rod left endpoint must use (x, y)")
    require("ROD_RIGHT_PX = (" in source, "rod right endpoint must use (x, y)")
    require("project_point_to_rod_mm" in source, "ball center must project onto rod axis")
    require("refine_ball_center_in_roi" in source, "ROI bright-center refinement must exist")
    require("ROI_EXPAND_RATIO" in source, "ROI expansion must be configurable")
    require("BRIGHT_PERCENTILE_DIVISOR" in source, "bright threshold must adapt to ROI")
    require("FILTER_ALPHA = 0.65" in source, "low-pass filter alpha must be configured")
    require("filtered_x_mm" in source, "filtered x position must be used")
    require("USE_ROI_REFINE = False" in source, "ROI refinement must be switchable and disabled by default")
    require("DRAW_RESULT = False" in source, "drawing YOLO boxes must be switchable and disabled by default")
    require("SHOW_IMAGE = True" in source, "image display must be switchable")
    require("PRINT_MEMORY = True" in source, "memory diagnostic output must be available")
    require("MEMORY_PRINT_EVERY_N_FRAMES = 30" in source, "memory diagnostics must be throttled")
    require("gc.mem_free()" in source, "K230 script must report free memory")


if __name__ == "__main__":
    main()
