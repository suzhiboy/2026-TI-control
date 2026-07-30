#include "t3_task.h"

#include "line_follow.h"
#include "sys_state.h"

typedef struct {
    BalanceControl_t *controller;
    bool active;
    bool vision_valid;
    int16_t x_mm;
    int16_t target_mm;
    uint8_t arrival_ticks;
    T3TaskPhase phase;
} T3TaskState;

static T3TaskState t3 = {
    .controller = 0,
    .active = false,
    .vision_valid = false,
    .x_mm = 0,
    .target_mm = T3_POSITIVE_TARGET_MM,
    .arrival_ticks = 0U,
    .phase = T3_PHASE_IDLE,
};

static int16_t i16_abs_diff(int16_t a, int16_t b)
{
    int16_t diff = (int16_t)(a - b);
    return (diff < 0) ? (int16_t)(-diff) : diff;
}

static void set_target_mm(int16_t target_mm)
{
    t3.target_mm = target_mm;
    t3.arrival_ticks = 0U;

    if (t3.controller != 0) {
        BalanceControl_SetReference(t3.controller, (float)target_mm / 10.0f);
    }
}

void T3Task_AttachController(BalanceControl_t *controller)
{
    t3.controller = controller;
}

void T3Task_UpdateVision(bool valid, int16_t x_mm)
{
    t3.vision_valid = valid;
    t3.x_mm = x_mm;
}

void T3Task_Start(void)
{
    LineTrack_Stop();
    ControlState_Set(CONTROL_STATIC_BALL);

    t3.active = true;
    t3.phase = T3_PHASE_TO_POSITIVE;
    t3.arrival_ticks = 0U;

    if (t3.controller != 0) {
        BalanceControl_Reset(t3.controller);
        BalanceControl_Enable(t3.controller, true);
    }
    set_target_mm(T3_POSITIVE_TARGET_MM);
}

void T3Task_Run(void)
{
    if (!t3.active) {
        return;
    }

    LineTrack_Stop();
    ControlState_Set(CONTROL_STATIC_BALL);

    if (!t3.vision_valid) {
        t3.arrival_ticks = 0U;
        return;
    }

    if (i16_abs_diff(t3.x_mm, t3.target_mm) <= T3_TARGET_TOLERANCE_MM) {
        if (t3.arrival_ticks < T3_ARRIVAL_CONFIRM_TICKS) {
            t3.arrival_ticks++;
        }
    } else {
        t3.arrival_ticks = 0U;
    }

    if (t3.arrival_ticks < T3_ARRIVAL_CONFIRM_TICKS) {
        return;
    }

    if (t3.phase == T3_PHASE_TO_POSITIVE) {
        t3.phase = T3_PHASE_TO_NEGATIVE;
        set_target_mm(T3_NEGATIVE_TARGET_MM);
    } else if (t3.phase == T3_PHASE_TO_NEGATIVE) {
        t3.phase = T3_PHASE_HOLD_NEGATIVE;
        set_target_mm(T3_NEGATIVE_TARGET_MM);
    }
}

void T3Task_Stop(void)
{
    t3.active = false;
    t3.phase = T3_PHASE_IDLE;
    t3.arrival_ticks = 0U;

    if (t3.controller != 0) {
        BalanceControl_Enable(t3.controller, false);
        BalanceControl_Reset(t3.controller);
    }

    ControlState_Set(CONTROL_IDLE);
}

bool T3Task_IsActive(void)
{
    return t3.active;
}

T3TaskPhase T3Task_GetPhase(void)
{
    return t3.phase;
}

int16_t T3Task_GetTargetMM(void)
{
    return t3.target_mm;
}
