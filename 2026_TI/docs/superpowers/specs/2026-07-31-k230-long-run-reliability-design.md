# K230 Long-Run Reliability Design

## Objective

Prevent the K230 vision loop from remaining permanently frozen during long car tests while preserving the current detector behavior, calibration, filtering, display output, and compact UART protocol.

## Confirmed Symptom

When the LCD image freezes, MSPM0 also stops receiving new UART frames. The failure therefore affects the K230 main loop or media pipeline, rather than only the LCD overlay.

## Scope

- Keep the current model, confidence threshold, NMS threshold, rod calibration, candidate selection, and position tracker unchanged.
- Keep UART at 115200 8N1 and preserve `B,seq,valid,x_mm,raw_x_mm,cx,cy,quality,fps\n` exactly.
- Release per-frame zero-copy camera and inference references deterministically.
- Add low-rate stage diagnostics that identify the last completed pipeline stage.
- Enable a hardware watchdog only through a firmware-compatible adapter. If the installed firmware does not expose a supported WDT API, print a clear startup warning and continue without it.
- Add an MSPM0 receive timeout that invalidates vision data and requests safe motor stop when fresh K230 frames stop arriving.

## K230 Frame Lifecycle

Each frame follows this order:

1. Feed the watchdog after the previous frame completed.
2. Acquire channel 2 with `sensor.snapshot()`.
3. Create the zero-copy numpy reference and temporary NN input tensor.
4. Run AI2D and KPU inference.
5. Copy KPU outputs, perform post-processing, tracking, UART transmission, and OSD update.
6. In a per-frame `finally` block, clear the KPU output list, detection list, temporary NN input tensor, numpy reference, and camera image reference.
7. Run garbage collection at the existing periodic interval, not once inside every inference and again periodically.
8. Leave a 2 ms scheduler interval between frames.

The persistent AI2D output tensor, KPU object, AI2D builder, sensor, UART, and OSD image remain allocated for the full session.

## Watchdog Behavior

The watchdog is fed only after a complete frame iteration. It is not fed before or during `snapshot()`, AI2D, or KPU calls. Therefore a native call that blocks indefinitely causes an automatic board reset and `/sdcard/main.py` starts again.

Watchdog initialization must be isolated behind a helper because CanMV firmware versions expose different constructor signatures. Unsupported initialization must not prevent vision startup.

## MSPM0 Fail-Safe

The UART receiver records the arrival time of every syntactically valid K230 frame, including `valid=0` lost-ball frames. If no new frame arrives for 200 ms:

- mark the vision sample invalid;
- do not reuse the previous `x_mm` value;
- expose a timeout state to the balance control layer;
- command the stepper control path to its existing safe-stop behavior.

The parser and wire format do not change.

## Diagnostics

During reliability testing, print a compact stage heartbeat at a low rate with frame number, completed stage, FPS, and free heap. This is diagnostic-only and is not sent over UART. Normal per-frame terminal logging remains disabled.

## Verification

- Source-level tests verify deterministic reference cleanup, watchdog feed placement, unchanged UART format, and timeout invalidation.
- Existing K230 logic and UART parser tests must continue to pass.
- Python syntax compilation must pass for both `main.py` and `ball_position_uart.py`.
- MSPM0 receiver code must compile with the TI ARM compiler where available.
- Hardware soak test: run display, inference, and UART continuously for at least 30 minutes; verify sequence growth and left/center/right coordinates.
- Stall test: intentionally prevent fresh K230 frames and verify MSPM0 enters timeout-safe state within 200 ms. If watchdog support is available, verify K230 restarts after the configured timeout.

## Non-Goals

- No detector retuning or coordinate recalibration.
- No UART field changes.
- No automatic motor movement during a vision timeout.
- No broad refactor of the existing vision file.
