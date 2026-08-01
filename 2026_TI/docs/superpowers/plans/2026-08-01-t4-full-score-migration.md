# T4 Full-Score Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the proven T1 ball controller into T4, pass B before braking, preserve smooth motion, and stop safely after vision loss.

**Architecture:** T4 owns a `T3TuneProfile` initialized from the proven T1 values and starts a generic profiled hold through `t3_task`. The car accelerates to 22 cm/s, requests ramp braking only after the encoder reaches B, and uses only measured speed-ramp acceleration as feed-forward. A 100 ms vision-loss counter requests the same controlled brake path.

**Tech Stack:** MSPM0G3507 C, TI Clang, Python static checks, CCS gmake.

---

### Task 1: Specify T4 behavior

**Files:**
- Modify: `tests/t4_smooth_motion_static_check.py`

- [ ] Require a T4-owned ball profile and generic profiled-hold API.
- [ ] Require braking at or after `T4_AB_DISTANCE_CM` and reject pre-B brake lead.
- [ ] Reject fixed feed-forward hold pulses and require measured ramp acceleration.
- [ ] Require a 100 ms vision-loss controlled-brake counter.
- [ ] Run the T4 test and confirm it fails for the missing behavior.

### Task 2: Implement T4 migration

**Files:**
- Modify: `key_menu.c`
- Modify: `t3_task.c`
- Modify: `t3_task.h`
- Modify: `empty.c`

- [ ] Add `T3Task_StartProfileHold()` and apply the supplied output/PID profile.
- [ ] Define the T4 ball profile from the proven T1 values.
- [ ] Set T4 speed to 22 cm/s and brake after reaching B.
- [ ] Add the vision-loss brake counter and reset it on start/stop.
- [ ] Remove T4 fixed hold feed-forward state and use ramp acceleration only.

### Task 3: Verify

**Files:**
- Test: `tests/t4_smooth_motion_static_check.py`
- Build: `Debug/2026_TI.out`

- [ ] Run the T4 static check.
- [ ] Run the CCS build from `Debug`.
- [ ] Report code locations and the required low-speed board validation sequence.
