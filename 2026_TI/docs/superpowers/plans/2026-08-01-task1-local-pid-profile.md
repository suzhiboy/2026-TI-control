# Task1 Local PID Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make all K1/T1 ball-tuning PID and output parameters editable in the Task1 section of `key_menu.c`.

**Architecture:** `key_menu.c` owns a constant `T3TuneProfile` built from `T1_TUNE_*` macros. `t3_task.h` exposes the profile type and tuning entry point, while `t3_task.c` applies the supplied profile without reading the shared `T3_CENTER_*` constants. Other task profiles remain unchanged.

**Tech Stack:** MSPM0G3507 C, TI Clang, Python static source checks, CCS gmake.

---

### Task 1: Define the expected Task1 ownership

**Files:**
- Modify: `tests/t1_ball_tune_static_check.py`

- [ ] Add assertions for the nine `T1_TUNE_*` macros and a local `T3TuneProfile` in `key_menu.c`.
- [ ] Require `T1_Init()` to call `T3Task_StartTuneHold(T1_TUNE_TARGET_MM, &t1_tune_profile)`.
- [ ] Require the tuning entry to apply the supplied output and PID values and not call `apply_center_profile()`.
- [ ] Run `python tests/t1_ball_tune_static_check.py` and confirm failure because the profile API does not exist.

### Task 2: Implement the local profile API

**Files:**
- Modify: `key_menu.c`
- Modify: `t3_task.h`
- Modify: `t3_task.c`

- [ ] Add `T3TuneProfile` with output limits and both PID triplets to `t3_task.h`.
- [ ] Change `T3Task_StartTuneHold()` to accept the target and profile pointer.
- [ ] Add the editable `T1_TUNE_*` block and constant profile beside `T1_Init()`.
- [ ] Apply the supplied profile after controller reset in `t3_task.c`; a null profile keeps the controller disabled at center.
- [ ] Run `python tests/t1_ball_tune_static_check.py` and confirm it passes.

### Task 3: Verify adjacent behavior

**Files:**
- Verify: `tests/balance_pid_static_check.py`
- Verify: `tests/t4_smooth_motion_static_check.py`
- Verify: `tests/t5_feedforward_direction_static_check.py`
- Verify: `tests/t6_static_check.py`
- Build: `Debug/2026_TI.out`

- [ ] Run all listed Python static checks.
- [ ] Run `D:\TMX_ENVSET\ccs\utils\bin\gmake.exe -k all` from `Debug`.
- [ ] Report exact source locations, parameter meaning, K1 usage, and remaining real-board verification.
