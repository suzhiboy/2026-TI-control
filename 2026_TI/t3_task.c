#include "t3_task.h"

#include "line_follow.h"
#include "sys_state.h"

typedef struct {
    BalanceControl_t *controller;
    bool active;
    bool vision_valid;
    int16_t x_mm;
    int16_t target_mm;
    float reference_mm;
    uint8_t arrival_ticks;
    T3TaskPhase phase;
} T3TaskState;

static T3TaskState t3 = {
    .controller = 0,
    .active = false,
    .vision_valid = false,
    .x_mm = 0,
    .target_mm = T3_POSITIVE_TARGET_MM,
    .reference_mm = 0.0f,
    .arrival_ticks = 0U,
    .phase = T3_PHASE_IDLE,
};

static bool has_reached_target(void)
{
    if (t3.target_mm >= 0) {
        return t3.x_mm >= (int16_t)(t3.target_mm - T3_TARGET_TOLERANCE_MM);
    }

    return t3.x_mm <= (int16_t)(t3.target_mm + T3_TARGET_TOLERANCE_MM);
}

static void set_target_mm(int16_t target_mm)
{
    t3.target_mm = target_mm;
    t3.arrival_ticks = 0U;
}

static void apply_reference_mm(float reference_mm)
{
    t3.reference_mm = reference_mm;
    if (t3.controller != 0) {
        BalanceControl_SetReference(t3.controller, t3.reference_mm / 10.0f);
    }
}

static void ramp_reference_toward_target(void)
{
    float target = (float)t3.target_mm;
    float diff = target - t3.reference_mm;

    if (diff > T3_REF_RAMP_MM_PER_TICK) {
        apply_reference_mm(t3.reference_mm + T3_REF_RAMP_MM_PER_TICK);
    } else if (diff < -T3_REF_RAMP_MM_PER_TICK) {
        apply_reference_mm(t3.reference_mm - T3_REF_RAMP_MM_PER_TICK);
    } else {
        apply_reference_mm(target);
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
    apply_reference_mm(0.0f);
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

    ramp_reference_toward_target();

    if (has_reached_target()) {
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
