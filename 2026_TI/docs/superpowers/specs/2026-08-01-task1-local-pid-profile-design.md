# Task1 Local PID Profile Design

## Goal

Make every parameter used by the K1/T1 pure ball-tuning mode editable beside
`T1_Init()` in `key_menu.c`. T1 must no longer obtain its PID or output profile
from the shared `T3_CENTER_*` constants.

## Task1 Parameters

`key_menu.c` will define the complete T1 tuning profile:

- target position in millimeters;
- negative and positive PWM delta limits;
- minimum PWM drive;
- position-loop `kp`, `ki`, and `kd`;
- velocity-loop `kp`, `ki`, and `kd`.

The initial values preserve the current pure velocity-loop tuning state:
position PID is zero, velocity `kp` is `0.02`, both velocity `ki` and `kd` are
zero, output limits are `+/-100 us`, and minimum drive is zero.

## Interface

Add a `T3TuneProfile` value type to `t3_task.h`. Change
`T3Task_StartTuneHold()` to accept a target and a pointer to this profile.
`T1_Init()` constructs a constant profile from the local `T1_TUNE_*` macros
and passes it into the tuning entry point.

The T3 implementation applies the supplied output limits and both PID loops
after resetting the controller. It does not call `apply_center_profile()` in
the T1 tuning path. T3/T4/T5/T6 continue using their existing shared center,
travel, capture, and final profiles.

## Safety And Failure Handling

The tuning entry rejects a null profile by leaving the ball controller safely
disabled at center. The existing controller clamps negative gains to zero and
clamps PWM limits to the physical range. The wheels remain stopped and the
control state remains `CONTROL_STATIC_BALL`.

## Verification

Update `tests/t1_ball_tune_static_check.py` first so it requires the local T1
macros, the profile passed by `T1_Init()`, and the absence of
`apply_center_profile()` from `T3Task_StartTuneHold()`. Run that test and
confirm it fails before implementation, then implement the API and rerun all
balance/T1 static checks plus the CCS build.

Real-board verification remains required: flash the new MSPM0 image, stop the
automatic T2 task, press K1, confirm both wheels remain stopped, and verify
that changing only the `T1_TUNE_*` values changes the rod response.
