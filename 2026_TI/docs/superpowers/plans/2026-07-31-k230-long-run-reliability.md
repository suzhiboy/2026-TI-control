# K230 Long-Run Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent a frozen K230 pipeline from leaving the car controller on stale ball coordinates, without changing detection behavior or the UART wire format.

**Architecture:** Keep all detection, calibration, tracking, display, and compact CSV code unchanged. Tighten the per-frame ownership lifetime of zero-copy camera objects, add optional CanMV watchdog and low-rate stage diagnostics, and extend the existing MSPM0 timeout state so all received frame types participate in a precise 200 ms link watchdog.

**Tech Stack:** CanMV MicroPython on K230, KPU/AI2D/aicube, K230 `machine.WDT` when available, MSPM0G3507 C, Python `unittest`, host GCC/TI ARM compiler.

---

### Task 1: Lock the Existing External Behavior

**Files:**
- Modify: `C:/Users/l2025/Documents/26电赛/host_tests/test_ball_position_logic.py`
- Modify: `C:/Users/l2025/Documents/26电赛/host_tests/vision_uart_compact_test.c`

- [ ] **Step 1: Add a failing source-contract test for K230 reliability controls**

Add a test that reads both K230 files and requires the same compact UART format, display settings, model/calibration constants, a 2 ms loop interval, optional watchdog helpers, stage heartbeat helper, and explicit clearing of per-frame input references. Keep the existing coordinate and frame-format assertions unchanged.

```python
def test_runtime_reliability_contract_is_present_in_both_k230_files(self):
    for path in (K230_SOURCE, OFFLINE_SOURCE):
        source = path.read_text(encoding="utf-8")
        self.assertIn('UART_FRAME_MODE = "compact_csv"', source)
        self.assertIn("ROD_LEFT_PX = (31, 219)", source)
        self.assertIn("ROD_CENTER_PX = (336, 199)", source)
        self.assertIn("ROD_RIGHT_PX = (745, 229)", source)
        self.assertIn("MAIN_LOOP_SLEEP_MS = 2", source)
        self.assertIn("def setup_watchdog", source)
        self.assertIn("def feed_watchdog", source)
        self.assertIn("def print_pipeline_heartbeat", source)
        self.assertIn("ai2d_input_tensor = None", source)
        self.assertIn("ai2d_input = None", source)
        self.assertIn("results = None", source)
```

- [ ] **Step 2: Run the K230 test and verify RED**

Run:

```powershell
python C:\Users\l2025\Documents\26电赛\host_tests\test_ball_position_logic.py
```

Expected: FAIL because watchdog and heartbeat helpers are missing and `main.py` still uses a zero millisecond loop interval.

- [ ] **Step 3: Add failing MSPM0 tests for all-frame timeout behavior**

Add tests that initialize the receiver, feed one valid frame and one explicit lost frame, poll at 19 and 20 ticks, and assert `timed_out` remains false before 200 ms and becomes true at 200 ms. Also assert a subsequent valid frame clears `timed_out`.

```c
static void test_link_timeout_applies_after_any_received_frame(void)
{
    VisionBallData data;

    VisionUart_Init();
    VisionUart_TestFeedString("B,1,0,0,0,0,0,0,58\n");
    VisionUart_Poll(100U);
    data = VisionUart_GetLatest();
    expect_int("lost frame not timeout", false, data.timed_out);

    VisionUart_Poll(119U);
    data = VisionUart_GetLatest();
    expect_int("before 200ms", false, data.timed_out);

    VisionUart_Poll(120U);
    data = VisionUart_GetLatest();
    expect_int("at 200ms", true, data.timed_out);
    expect_int("timeout invalid", false, data.valid);
    expect_int("timeout lost", true, data.lost);

    VisionUart_TestFeedString("B,2,1,10,11,360,200,90,58\n");
    VisionUart_Poll(121U);
    data = VisionUart_GetLatest();
    expect_int("fresh frame clears timeout", false, data.timed_out);
    expect_int("fresh frame valid", true, data.valid);
}
```

- [ ] **Step 4: Compile and run the MSPM0 host test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DHOST_TEST -I"E:\DIANSAI resources\26DIANSAI\DIANSAI_26\2026_TI" "C:\Users\l2025\Documents\26电赛\host_tests\vision_uart_compact_test.c" "E:\DIANSAI resources\26DIANSAI\DIANSAI_26\2026_TI\vision_uart.c" -o "C:\Users\l2025\Documents\26电赛\host_tests\vision_uart_compact_test.exe"
```

Expected: FAIL to compile because `VisionBallData.timed_out` does not exist.

### Task 2: Make K230 Per-Frame Ownership Deterministic

**Files:**
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/ball_position_uart.py`
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/main.py`

- [ ] **Step 1: Initialize all temporary frame references before acquisition**

At the start of each loop iteration, initialize `rgb888p_img`, `ai2d_input`, `ai2d_input_tensor`, `results`, and `det_boxes` to `None`. This makes cleanup valid even if `snapshot()`, AI2D, KPU, or post-processing raises.

```python
rgb888p_img = None
ai2d_input = None
ai2d_input_tensor = None
results = None
det_boxes = None
```

- [ ] **Step 2: Wrap the complete frame pipeline in one `try/finally`**

Keep the current operation order and results, but move UART, tracking, OSD, and diagnostics inside the frame `try`. In its `finally`, clear references from the most-derived object back to the camera frame:

```python
finally:
    det_boxes = None
    results = None
    ai2d_input_tensor = None
    ai2d_input = None
    rgb888p_img = None
```

Do not clear the persistent `ai2d_output_tensor`, `ai2d_builder`, `kpu`, `osd_img`, `sensor`, or `uart` objects.

- [ ] **Step 3: Remove redundant per-frame collection and add a scheduler interval**

Remove the unconditional `gc.collect()` immediately after post-processing. Keep the existing `GC_EVERY_N_FRAMES = 30` collection and set:

```python
MAIN_LOOP_SLEEP_MS = 2
```

Call `os.exitpoint()` once per loop in both development and offline files.

- [ ] **Step 4: Run the K230 test suite**

Run:

```powershell
python C:\Users\l2025\Documents\26电赛\host_tests\test_ball_position_logic.py
```

Expected: watchdog/heartbeat contract may still fail, while all detector, calibration, tracker, display, and UART format tests pass.

### Task 3: Add Optional Watchdog and Stage Diagnostics

**Files:**
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/ball_position_uart.py`
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/main.py`
- Test: `C:/Users/l2025/Documents/26电赛/host_tests/test_ball_position_logic.py`

- [ ] **Step 1: Import WDT without making it a startup dependency**

Extend the existing `machine` import with a separate optional import so older firmware can still boot:

```python
try:
    from machine import WDT
except ImportError:
    WDT = None
```

Define conservative controls:

```python
ENABLE_WATCHDOG = True
WATCHDOG_ID = 0
WATCHDOG_TIMEOUT_SECONDS = 5
PIPELINE_HEARTBEAT_EVERY_N_FRAMES = 300
```

- [ ] **Step 2: Add firmware-compatible watchdog helpers**

Use the documented positional constructor first and fall back to a keyword constructor only for firmware variation. Any failure prints once and returns `None` without blocking vision startup.

```python
def setup_watchdog():
    if not ENABLE_WATCHDOG or WDT is None:
        print("K230_WDT disabled unavailable")
        return None
    try:
        watchdog = WDT(WATCHDOG_ID, WATCHDOG_TIMEOUT_SECONDS)
        print("K230_WDT enabled timeout_s={}".format(WATCHDOG_TIMEOUT_SECONDS))
        return watchdog
    except TypeError:
        try:
            watchdog = WDT(WATCHDOG_ID, timeout=WATCHDOG_TIMEOUT_SECONDS)
            print("K230_WDT enabled timeout_s={}".format(WATCHDOG_TIMEOUT_SECONDS))
            return watchdog
        except Exception as e:
            print("K230_WDT disabled err={}".format(e))
            return None
    except Exception as e:
        print("K230_WDT disabled err={}".format(e))
        return None


def feed_watchdog(watchdog):
    if watchdog is not None:
        watchdog.feed()
```

- [ ] **Step 3: Feed only after a complete frame**

Create the watchdog after sensor startup. Call `feed_watchdog(watchdog)` only after UART/display work and per-frame cleanup complete. Do not feed immediately before `snapshot()`, `ai2d_builder.run()`, or `kpu.run()`.

- [ ] **Step 4: Add a low-rate stage heartbeat**

Track the latest stage (`snapshot`, `ai2d`, `kpu`, `post`, `uart`, `display`, `complete`) and print every 300 completed frames:

```python
def print_pipeline_heartbeat(frame_id, stage, fps):
    if PIPELINE_HEARTBEAT_EVERY_N_FRAMES <= 0:
        return
    if frame_id % PIPELINE_HEARTBEAT_EVERY_N_FRAMES != 0:
        return
    try:
        free_heap = gc.mem_free()
    except Exception:
        free_heap = -1
    print("K230_HEARTBEAT frame={} stage={} fps={} heap={}".format(
        frame_id, stage, round_int(fps), free_heap
    ))
```

- [ ] **Step 5: Verify Python tests and syntax**

Run:

```powershell
python C:\Users\l2025\Documents\26电赛\host_tests\test_ball_position_logic.py
python -m py_compile "E:\DIANSAI resources\26DIANSAI\DIANSAI_26\2026_TI\k230\ball_position_uart.py" "E:\DIANSAI resources\26DIANSAI\DIANSAI_26\2026_TI\k230\main.py"
```

Expected: all tests pass and both files compile without syntax errors.

### Task 4: Complete the MSPM0 Communication Timeout State

**Files:**
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/vision_uart.h`
- Modify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/vision_uart.c`
- Test: `C:/Users/l2025/Documents/26电赛/host_tests/vision_uart_compact_test.c`

- [ ] **Step 1: Add explicit timeout state**

Add `bool timed_out;` after `lost` in `VisionBallData`. Add internal `static bool vision_frame_received;`, reset it in `VisionUart_Init`, and set it after every successful parse, including explicit lost frames.

```c
typedef struct {
    bool valid;
    bool lost;
    bool timed_out;
    uint16_t seq;
    /* existing fields unchanged */
} VisionBallData;
```

- [ ] **Step 2: Apply timeout to every successfully received frame**

Change the timeout comparison to `>= VISION_TIMEOUT_TICKS`. On timeout, clear coordinates and quality so stale values cannot be consumed accidentally:

```c
if (vision_frame_received &&
    ((uint32_t)(now_tick_10ms - latest_ball.last_update_tick) >=
     VISION_TIMEOUT_TICKS)) {
    latest_ball.valid = false;
    latest_ball.lost = true;
    latest_ball.timed_out = true;
    latest_ball.x_mm = 0;
    latest_ball.raw_x_mm = 0;
    latest_ball.cx = 0U;
    latest_ball.cy = 0U;
    latest_ball.quality = 0U;
    latest_ball.conf_percent = 0U;
}
```

Successful parsing uses the existing `memset`, so `timed_out` returns to false automatically. The UART parser and CSV field count remain unchanged.

- [ ] **Step 3: Run compact and legacy host tests**

Run the GCC command from Task 1, then execute:

```powershell
& "C:\Users\l2025\Documents\26电赛\host_tests\vision_uart_compact_test.exe"
```

Expected: `PASS compact vision uart protocol`.

Rebuild and run the existing legacy test with the same `vision_uart.c`; expected result is its existing PASS output.

- [ ] **Step 4: Verify the existing motor-safe path remains connected**

Inspect `empty.c` and confirm the timer ISR still stops `PD42S1_PWM_INST` whenever `g_vision_ball.valid` is false. No control-loop change is required because the timeout now drives that existing branch.

### Task 5: Final Regression and Hardware Handoff

**Files:**
- Verify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/ball_position_uart.py`
- Verify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/k230/main.py`
- Verify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/vision_uart.c`
- Verify: `E:/DIANSAI resources/26DIANSAI/DIANSAI_26/2026_TI/vision_uart.h`
- Verify: `C:/Users/l2025/Documents/26电赛/host_tests/test_ball_position_logic.py`
- Verify: `C:/Users/l2025/Documents/26电赛/host_tests/vision_uart_compact_test.c`

- [ ] **Step 1: Run all host regression tests**

Run Python logic tests, compact UART tests, legacy UART tests, and any existing project unit tests. Record exact pass counts.

- [ ] **Step 2: Compile MSPM0 source**

Compile `vision_uart.c` with the available TI ARM compiler. Attempt the complete CCS build, but report the pre-existing missing `TM1637.c` dependency separately if it still blocks linking.

- [ ] **Step 3: Compare behavior-critical constants and frame builder**

Diff the pre-change and post-change sections and verify no changes to model path, thresholds, calibration points, tracker constants, `make_ball_frame`, baud rate, or display enablement.

- [ ] **Step 4: Prepare the SD-card artifact**

Use the updated `k230/main.py` as `/sdcard/main.py`; retain the existing `/sdcard/mp_deployment_source/` model and JSON files.

- [ ] **Step 5: Perform hardware checks**

Run for at least 30 minutes while checking that LCD motion, `K230_HEARTBEAT`, UART sequence growth, and left/center/right coordinates continue. Then disconnect or stop K230 TX and verify MSPM0 stops balance PWM at 200 ms. A deliberate K230 pipeline stall should result in a reboot after about five seconds only when startup prints `K230_WDT enabled`.
