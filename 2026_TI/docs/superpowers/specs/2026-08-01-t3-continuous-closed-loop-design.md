# T3 Continuous Closed-Loop Design

## Goal

Complete Task 3 with the car stationary: move the ball from center into the
`+50 mm` target tolerance, reverse toward `-50 mm`, and remain within
`-50 +/- 10 mm`. The measured time stops only after 200 ms continuously in
the final tolerance.

## Control State Machine

The task has three phases:

1. `TO_POSITIVE`: command a `+50 mm` reference. Two distinct vision sequence
   numbers at or beyond `+40 mm` switch the reference immediately to `-50 mm`.
   Values above `+60 mm` still count so a fast sample cannot skip the
   transition. Re-reading one UART frame cannot satisfy both confirmations.
2. `TO_NEGATIVE`: keep the controller enabled with a `-50 mm` reference. Start
   a real 10 ms timer window when the ball enters `[-60, -40] mm`; reset the
   window immediately on any sample outside that interval. The window can
   complete only after 200 ms and while a new frame has arrived within the
   configured 200 ms freshness limit.
3. `HOLD_NEGATIVE`: freeze the elapsed-time result after 200 ms of continuous
   final stability, but keep the `-50 mm` reference and closed-loop controller
   active so the ball remains in tolerance.

## Removed Behavior

Remove the fixed-PWM brake, rebound detection, `+550 us` lift, slow return,
settle profile, and center-lock override from Task 3. Those open-loop actions
depend on arrival velocity and caused the observed `-120 mm` excursion. A
center override also prevented correction from the observed final `-62 mm`.

## Safety

Existing vision-loss behavior remains unchanged: an invalid vision frame
resets the current arrival/stability confirmation and the main loop softly
returns the actuator toward neutral. When valid vision returns, Task 3 resumes
the same target phase. Task selection and automatic-start configuration are
outside this change and are not modified by the implementation.

## Verification

Host tests must prove the tolerance transition, continuous final-stability
window, rejection of negative overshoot, closed-loop output in final hold, and
elapsed-time freeze. Static checks must reject reintroduction of T3 PWM
overrides. The complete MSPM0 Debug target must build after host tests pass.

## OLED Diagnostics

While Task 3 is active, the OLED keeps the task name on line 1, shows target
and measured position on line 2, shows PWM delta and vision sequence on line 3,
and keeps elapsed time on line 4. Lost or timed-out vision replaces the measured
position without removing PWM and sequence diagnostics.
