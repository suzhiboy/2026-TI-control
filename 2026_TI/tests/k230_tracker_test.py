from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "k230" / "ball_position_uart.py"


def load_namespace():
    namespace = {"__name__": "k230_tracker_test"}
    source = SCRIPT.read_text(encoding="utf-8")
    exec(compile(source, str(SCRIPT), "exec"), namespace)
    return namespace


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    ns = load_namespace()
    tracker = ns["PositionTracker"]()

    accepted, x_mm = tracker.update(0)
    require(accepted, "tracker must accept the first valid ball frame immediately")
    require(x_mm == 0, "first accepted x must be 0 mm")

    accepted, x_mm = tracker.update(100)
    require(accepted, "tracker must accept fast in-pipe movement without multi-second reject")
    require(x_mm >= 80, "fast accepted position must stay close to the new ball position")


if __name__ == "__main__":
    main()
