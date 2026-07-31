# T3 Continuous Closed-Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Task 3 open-loop brake/rebound/center-lock behavior with continuous position feedback through final hold.

**Architecture:** Keep `t3_task` responsible only for target sequencing and timing. Keep `balance_control` responsible for velocity-aware PWM generation. Task 3 selects one travel profile and never enables a PWM override.

**Tech Stack:** C11 host tests with MinGW GCC, Python static checks, TI MSPM0G3507 CCS Debug build.

---

### Task 1: Specify continuous closed-loop behavior

**Files:**
- Modify: `tests/t3_task_test.c.disabled`

- [ ] Replace brake/lift tests with tests that enter `TO_NEGATIVE` and remain there while outside final tolerance.
- [ ] Add a test asserting repeated `Run` calls without 10 ms timer ticks cannot complete final stability.
- [ ] Add a test asserting duplicate vision sequence numbers cannot satisfy the positive two-frame confirmation.
- [ ] Add a test asserting a stale final vision frame cannot complete final stability.
- [ ] Add a test asserting the PWM override remains disabled before and after entering `HOLD_NEGATIVE`.
- [ ] Add a test asserting leaving the final tolerance resets the 200 ms stability count.
- [ ] Compile and run the test; expect failures because production code still enters `BRAKE_NEGATIVE` and enables PWM override.

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I. -x c tests\t3_task_test.c.disabled t3_task.c sys_state.c -o tests\t3_task_test.exe
.\tests\t3_task_test.exe
```

### Task 2: Simplify the production state machine

**Files:**
- Modify: `t3_task.h`
- Modify: `t3_task.c`

- [ ] Remove brake, lift, return, and settle tuning macros and enum phases.
- [ ] Remove open-loop counters and PWM override helper logic from `T3TaskState`.
- [ ] Keep only `IDLE`, `TO_POSITIVE`, `TO_NEGATIVE`, and `HOLD_NEGATIVE`.
- [ ] In `TO_POSITIVE`, require two distinct valid vision sequences at `x >= 40 mm`, then switch the controller reference immediately to `-50 mm`.
- [ ] In `TO_NEGATIVE`, measure 200 ms with `T3Task_Tick10ms`, reset outside `[-60, -40] mm`, and reject stale vision.
- [ ] In `HOLD_NEGATIVE`, keep the controller reference and output profile active; do not enable PWM override.
- [ ] Recompile and run the T3 host test; expect `PASS t3 task state machine`.

### Task 3: Align static checks with the design

**Files:**
- Modify: `tests/source_static_check.py`

- [ ] Update the current T3 travel limit expectation to the source value selected for this version.
- [ ] Remove assertions that require brake/lift/settle macros and phases.
- [ ] Add assertions that T3 contains no call that enables a PWM override and that final hold remains closed-loop.
- [ ] Run `python tests\source_static_check.py`; expect success.

### Task 4: Regression and firmware verification

**Files:**
- Verify: `tests/balance_control_direction_test.exe`
- Verify: `tests/vision_uart_test.exe`
- Build: `Debug/2026_TI.out`

- [ ] Rebuild and run the T3 host test from source.
- [ ] Run balance-control and UART protocol regression tests.
- [ ] Run both K230 Python checks.
- [ ] Run `mingw32-make -B -C Debug 2026_TI.out -j1` and confirm linker success.
- [ ] Record that host/build success does not prove physical motion; provide a board test sequence using positive peak, negative peak, final position, elapsed time, and OLED PWM.

### Task 5: Add T3 OLED diagnostics

**Files:**
- Modify: `empty.c`
- Modify: `tests/source_static_check.py`

- [ ] Add a failing static assertion for T3 `T/X`, `P/S`, and elapsed-time lines.
- [ ] Override menu lines 2 and 3 only while T3 is active; preserve line 1 and elapsed time on line 4.
- [ ] Show `TIMEOUT` or `LOST` instead of a stale X value while retaining PWM delta and sequence.
- [ ] Run static checks and rebuild `Debug/2026_TI.out`.
