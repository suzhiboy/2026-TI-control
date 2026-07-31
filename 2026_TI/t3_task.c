#include "t3_task.h"

#include "line_follow.h"
#include "oled_text.h"
#include "sys_state.h"

#include <stdio.h>

typedef struct {
    BalanceControl_t *controller;
    bool active;
    bool vision_valid;
    bool vision_timed_out;
    bool vision_sample_pending;
    bool has_vision_seq;
    bool stability_active;
    bool capture_profile_active;
    bool final_profile_active;
    bool final_freeze_active;
    bool has_still_x_mm;
    uint16_t vision_seq;
    int16_t x_mm;
    int16_t still_x_mm;
    int16_t target_mm;
    float reference_mm;
    uint8_t arrival_ticks;
    uint8_t still_frame_count;
    uint16_t freeze_return_start_pulse;
    uint32_t last_vision_tick;
    uint32_t stability_start_tick;
    uint32_t freeze_return_start_tick;
    volatile uint32_t elapsed_ticks_10ms;
    T3TaskPhase phase;
} T3TaskState;

static T3TaskState t3 = {
    .controller = 0,
    .active = false,
    .vision_valid = false,
    .vision_timed_out = false,
    .vision_sample_pending = false,
    .has_vision_seq = false,
    .stability_active = false,
    .capture_profile_active = false,
    .final_profile_active = false,
    .final_freeze_active = false,
    .has_still_x_mm = false,
    .vision_seq = 0U,
    .x_mm = 0,
    .still_x_mm = 0,
    .target_mm = T3_POSITIVE_TARGET_MM,
    .reference_mm = 0.0f,
    .arrival_ticks = 0U,
    .still_frame_count = 0U,
    .freeze_return_start_pulse = BC_PWM_CENTER_US,
    .last_vision_tick = 0U,
    .stability_start_tick = 0U,
    .freeze_return_start_tick = 0U,
    .elapsed_ticks_10ms = 0U,
    .phase = T3_PHASE_IDLE,
};

static bool has_reached_positive_tolerance(void)
{
    return t3.x_mm >=
        (int16_t)(T3_POSITIVE_TARGET_MM - T3_TARGET_TOLERANCE_MM);
}

static bool is_inside_negative_tolerance(void)
{
    return (t3.x_mm >=
            (int16_t)(T3_NEGATIVE_TARGET_MM - T3_TARGET_TOLERANCE_MM)) &&
           (t3.x_mm <=
            (int16_t)(T3_NEGATIVE_TARGET_MM + T3_TARGET_TOLERANCE_MM));
}

static bool is_inside_freeze_tolerance(void)
{
    return (t3.x_mm >=
            (int16_t)(T3_NEGATIVE_TARGET_MM - T3_FREEZE_TOLERANCE_MM)) &&
           (t3.x_mm <=
            (int16_t)(T3_NEGATIVE_TARGET_MM + T3_FREEZE_TOLERANCE_MM));
}

static bool vision_is_fresh(void)
{
    return t3.vision_valid && t3.has_vision_seq &&
        ((uint32_t)(t3.elapsed_ticks_10ms - t3.last_vision_tick) <
         T3_VISION_FRESH_TICKS);
}

static char sign_of_i32(int32_t value)
{
    return (value < 0) ? '-' : '+';
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t clamp_target_mm(int16_t target_mm)
{
    const int16_t min_mm = (int16_t)(BC_POS_MIN_CM * 10.0f);
    const int16_t max_mm = (int16_t)(BC_POS_MAX_CM * 10.0f);

    if (target_mm < min_mm) {
        return min_mm;
    }
    if (target_mm > max_mm) {
        return max_mm;
    }
    return target_mm;
}

static const char *phase_label(void)
{
    switch (t3.phase) {
        case T3_PHASE_TO_POSITIVE: return "POS";
        case T3_PHASE_TO_NEGATIVE: return "NEG";
        case T3_PHASE_FINAL_RETURN: return "RET";
        case T3_PHASE_HOLD_NEGATIVE: return "HLD";
        case T3_PHASE_IDLE:
        default: return "IDL";
    }
}

static void show_fixed_line(uint8_t line, const char *text)
{
    char fixed[17];

    snprintf(fixed, sizeof(fixed), "%-16s", text);
    OLED_ShowLineString(line, 1, fixed);
}

static void set_target_mm(int16_t target_mm)
{
    t3.target_mm = target_mm;
    t3.arrival_ticks = 0U;
}

static void apply_reference_mm(float reference_mm);

static void clear_final_freeze(void)
{
    if (t3.final_freeze_active && (t3.controller != 0)) {
        BalanceControl_SetPwmOverride(t3.controller, false,
            (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    }
    t3.final_freeze_active = false;
    t3.has_still_x_mm = false;
    t3.still_x_mm = 0;
    t3.still_frame_count = 0U;
    t3.freeze_return_start_pulse = BC_PWM_CENTER_US;
    t3.freeze_return_start_tick = 0U;
}

static uint16_t neutral_pulse_us(void)
{
    if (t3.controller == 0) {
        return BC_PWM_CENTER_US;
    }
    return (uint16_t)(t3.controller->pwm_neutral + 0.5f);
}

static uint16_t final_hold_pulse_us(void)
{
    int32_t neutral = (int32_t)neutral_pulse_us();
    int32_t start = (int32_t)t3.freeze_return_start_pulse;
    int32_t delta = start - neutral;
    int32_t hold_delta;
    int32_t pulse;

    if (T3_FREEZE_HOLD_BIAS_PERCENT >= 100U) {
        return t3.freeze_return_start_pulse;
    }

    hold_delta = (delta * (int32_t)T3_FREEZE_HOLD_BIAS_PERCENT) / 100;
    if (delta > 0) {
        if (hold_delta < (int32_t)T3_FREEZE_HOLD_MIN_DELTA_US) {
            hold_delta = (int32_t)T3_FREEZE_HOLD_MIN_DELTA_US;
        }
    } else if (delta < 0) {
        if (hold_delta > -(int32_t)T3_FREEZE_HOLD_MIN_DELTA_US) {
            hold_delta = -(int32_t)T3_FREEZE_HOLD_MIN_DELTA_US;
        }
    } else {
        hold_delta = (int32_t)T3_FREEZE_HOLD_MIN_DELTA_US;
    }

    pulse = neutral + hold_delta;
    if (pulse < (int32_t)BC_PWM_MIN_US) {
        pulse = (int32_t)BC_PWM_MIN_US;
    } else if (pulse > (int32_t)BC_PWM_MAX_US) {
        pulse = (int32_t)BC_PWM_MAX_US;
    }

    return (uint16_t)pulse;
}

static uint16_t interpolate_pulse(uint16_t start_pulse,
                                  uint16_t target_pulse,
                                  uint32_t elapsed_ticks,
                                  uint32_t total_ticks)
{
    int32_t delta;
    int32_t numerator;

    if ((total_ticks == 0U) || (elapsed_ticks >= total_ticks)) {
        return target_pulse;
    }

    delta = (int32_t)target_pulse - (int32_t)start_pulse;
    numerator = delta * (int32_t)elapsed_ticks;
    if (numerator >= 0) {
        delta = (numerator + (int32_t)(total_ticks / 2U)) /
            (int32_t)total_ticks;
    } else {
        delta = (numerator - (int32_t)(total_ticks / 2U)) /
            (int32_t)total_ticks;
    }

    return (uint16_t)((int32_t)start_pulse + delta);
}

static void update_freeze_return(void)
{
    uint32_t elapsed;
    uint16_t hold_pulse;
    uint16_t pulse;

    if ((t3.controller == 0) || !t3.final_freeze_active) {
        return;
    }

    hold_pulse = final_hold_pulse_us();
    elapsed = t3.elapsed_ticks_10ms - t3.freeze_return_start_tick;
    pulse = interpolate_pulse(t3.freeze_return_start_pulse, hold_pulse,
                              elapsed, T3_FREEZE_RETURN_TICKS);
    BalanceControl_SetPwmOverride(t3.controller, true, pulse);

    if (elapsed >= T3_FREEZE_RETURN_TICKS) {
        t3.phase = T3_PHASE_HOLD_NEGATIVE;
        BalanceControl_SetPwmOverride(t3.controller, true, hold_pulse);
        apply_reference_mm((float)T3_NEGATIVE_TARGET_MM);
    }
}

static void begin_freeze_return(void)
{
    if (t3.controller == 0) {
        return;
    }

    t3.freeze_return_start_pulse = t3.controller->pwm_pulse;
    t3.freeze_return_start_tick = t3.elapsed_ticks_10ms;
    t3.final_freeze_active = true;
    t3.phase = T3_PHASE_FINAL_RETURN;
    set_target_mm(T3_NEGATIVE_TARGET_MM);
    apply_reference_mm((float)T3_NEGATIVE_TARGET_MM);
    BalanceControl_SetPwmOverride(t3.controller, true,
        t3.freeze_return_start_pulse);
}

static void update_final_freeze(bool new_vision_sample)
{
    if (!new_vision_sample) {
        return;
    }

    if (!t3.has_still_x_mm || (t3.x_mm != t3.still_x_mm)) {
        if (t3.final_freeze_active && (t3.controller != 0)) {
            BalanceControl_SetPwmOverride(t3.controller, false,
                neutral_pulse_us());
        }
        t3.final_freeze_active = false;
        t3.has_still_x_mm = true;
        t3.still_x_mm = t3.x_mm;
        t3.still_frame_count = 1U;
        return;
    }

    if (t3.still_frame_count < T3_STILL_FRAME_FREEZE_COUNT) {
        t3.still_frame_count++;
    }

    if (!t3.final_freeze_active &&
        (t3.still_frame_count >= T3_STILL_FRAME_FREEZE_COUNT) &&
        (t3.controller != 0)) {
        begin_freeze_return();
    }
}

static void apply_reference_mm(float reference_mm)
{
    t3.reference_mm = reference_mm;
    if (t3.controller != 0) {
        BalanceControl_SetReference(t3.controller, t3.reference_mm / 10.0f);
    }
}

static void apply_travel_profile(void)
{
    if (t3.controller == 0) {
        return;
    }

    BalanceControl_SetPwmOverride(t3.controller, false,
        (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    BalanceControl_SetOutputProfile(t3.controller,
        T3_TRAVEL_NEGATIVE_DELTA_LIMIT_US,
        T3_TRAVEL_POSITIVE_DELTA_LIMIT_US,
        T3_TRAVEL_MIN_DRIVE_US);
    BalanceControl_SetPositionPD(t3.controller,
        BC_DEFAULT_POS_KP, BC_DEFAULT_POS_KD);
    BalanceControl_SetVelocityP(t3.controller, BC_DEFAULT_VEL_KP);
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();
}

static void apply_return_profile(void)
{
    if (t3.controller == 0) {
        return;
    }

    BalanceControl_SetPwmOverride(t3.controller, false,
        (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    BalanceControl_SetOutputProfile(t3.controller,
        T3_RETURN_NEGATIVE_DELTA_LIMIT_US,
        T3_RETURN_POSITIVE_DELTA_LIMIT_US,
        T3_RETURN_MIN_DRIVE_US);
    BalanceControl_SetPositionPD(t3.controller,
        BC_DEFAULT_POS_KP, BC_DEFAULT_POS_KD);
    BalanceControl_SetVelocityP(t3.controller, BC_DEFAULT_VEL_KP);
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();
}

static void apply_capture_profile(void)
{
    if ((t3.controller == 0) ||
        (t3.capture_profile_active && !t3.final_profile_active)) {
        return;
    }

    BalanceControl_SetPwmOverride(t3.controller, false,
        (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    BalanceControl_SetOutputProfile(t3.controller,
        T3_CAPTURE_NEGATIVE_DELTA_LIMIT_US,
        T3_CAPTURE_POSITIVE_DELTA_LIMIT_US,
        T3_CAPTURE_MIN_DRIVE_US);
    BalanceControl_SetPositionPD(t3.controller,
        T3_CAPTURE_POS_KP, T3_CAPTURE_POS_KD);
    BalanceControl_SetVelocityP(t3.controller, T3_CAPTURE_VEL_KP);
    t3.capture_profile_active = true;
    t3.final_profile_active = false;
    clear_final_freeze();
}

static void apply_final_profile(void)
{
    if ((t3.controller == 0) || t3.final_profile_active) {
        return;
    }

    BalanceControl_SetPwmOverride(t3.controller, false,
        (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    BalanceControl_SetOutputProfile(t3.controller,
        T3_FINAL_NEGATIVE_DELTA_LIMIT_US,
        T3_FINAL_POSITIVE_DELTA_LIMIT_US,
        T3_FINAL_MIN_DRIVE_US);
    BalanceControl_SetPositionPD(t3.controller,
        T3_FINAL_POS_KP, T3_FINAL_POS_KD);
    BalanceControl_SetVelocityP(t3.controller, T3_FINAL_VEL_KP);
    t3.capture_profile_active = true;
    t3.final_profile_active = true;
}

static void apply_center_profile(void)
{
    if (t3.controller == 0) {
        return;
    }

    BalanceControl_SetPwmOverride(t3.controller, false,
        (uint16_t)(t3.controller->pwm_neutral + 0.5f));
    BalanceControl_SetOutputProfile(t3.controller,
        T3_CENTER_DELTA_LIMIT_US,
        T3_CENTER_DELTA_LIMIT_US,
        T3_CENTER_MIN_DRIVE_US);
    BalanceControl_SetPositionPD(t3.controller,
        T3_CENTER_POS_KP, T3_CENTER_POS_KD);
    BalanceControl_SetVelocityP(t3.controller, T3_CENTER_VEL_KP);
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();
}

void T3Task_AttachController(BalanceControl_t *controller)
{
    t3.controller = controller;
}

void T3Task_UpdateVision(bool valid, bool timed_out, uint16_t seq, int16_t x_mm)
{
    bool new_sequence = (!t3.has_vision_seq) || (seq != t3.vision_seq);

    t3.vision_valid = valid;
    t3.vision_timed_out = timed_out;
    t3.vision_seq = seq;
    t3.x_mm = x_mm;

    if (!valid) {
        t3.vision_sample_pending = false;
        t3.has_vision_seq = false;
        if ((t3.phase != T3_PHASE_HOLD_NEGATIVE) &&
            (t3.phase != T3_PHASE_FINAL_RETURN)) {
            clear_final_freeze();
        }
        return;
    }

    if (new_sequence) {
        t3.has_vision_seq = true;
        t3.vision_sample_pending = true;
        t3.last_vision_tick = t3.elapsed_ticks_10ms;
    }
}

void T3Task_Tick10ms(void)
{
    if (t3.active && (t3.phase != T3_PHASE_HOLD_NEGATIVE) &&
        (t3.elapsed_ticks_10ms < UINT32_MAX)) {
        t3.elapsed_ticks_10ms++;
    }
}

void T3Task_Start(void)
{
    LineTrack_Stop();
    ControlState_Set(CONTROL_STATIC_BALL);

    t3.active = true;
    t3.phase = T3_PHASE_TO_POSITIVE;
    t3.arrival_ticks = 0U;
    t3.vision_sample_pending = false;
    t3.has_vision_seq = false;
    t3.stability_active = false;
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();
    t3.vision_seq = 0U;
    t3.last_vision_tick = 0U;
    t3.stability_start_tick = 0U;
    t3.elapsed_ticks_10ms = 0U;

    if (t3.controller != 0) {
        BalanceControl_Reset(t3.controller);
        BalanceControl_Enable(t3.controller, true);
        apply_travel_profile();
    }
    set_target_mm(T3_POSITIVE_TARGET_MM);
    apply_reference_mm((float)T3_POSITIVE_TARGET_MM);
}

void T3Task_Run(void)
{
    bool new_vision_sample;
    bool inside_negative_window;
    bool inside_freeze_window;

    if (!t3.active) {
        return;
    }

    LineTrack_Stop();
    ControlState_Set(CONTROL_STATIC_BALL);

    new_vision_sample = t3.vision_sample_pending;
    t3.vision_sample_pending = false;

    if (t3.phase == T3_PHASE_HOLD_NEGATIVE) {
        return;
    }

    if (t3.phase == T3_PHASE_FINAL_RETURN) {
        update_freeze_return();
        return;
    }

    if (!vision_is_fresh()) {
        t3.arrival_ticks = 0U;
        t3.stability_active = false;
        clear_final_freeze();
        return;
    }

    if (t3.phase == T3_PHASE_TO_POSITIVE) {
        if (!new_vision_sample) {
            return;
        }

        if (has_reached_positive_tolerance()) {
            if (t3.arrival_ticks < T3_ARRIVAL_CONFIRM_TICKS) {
                t3.arrival_ticks++;
            }
        } else {
            t3.arrival_ticks = 0U;
        }

        if (t3.arrival_ticks >= T3_ARRIVAL_CONFIRM_TICKS) {
            t3.phase = T3_PHASE_TO_NEGATIVE;
            t3.stability_active = false;
            t3.capture_profile_active = false;
            t3.final_profile_active = false;
            set_target_mm(T3_NEGATIVE_TARGET_MM);
            apply_return_profile();
            apply_reference_mm((float)T3_NEGATIVE_TARGET_MM);
        }
        return;
    }

    inside_negative_window = is_inside_negative_tolerance();
    inside_freeze_window = is_inside_freeze_tolerance();

    if (!inside_negative_window && (t3.x_mm <= T3_NEGATIVE_CAPTURE_ENTRY_MM)) {
        apply_capture_profile();
    }

    if (inside_negative_window) {
        apply_final_profile();
        if (inside_freeze_window) {
            update_final_freeze(new_vision_sample);
        } else {
            clear_final_freeze();
        }
        if (!t3.stability_active) {
            t3.stability_active = true;
            t3.stability_start_tick = t3.elapsed_ticks_10ms;
        }
    } else {
        clear_final_freeze();
        if (t3.final_profile_active) {
            apply_capture_profile();
        }
        t3.stability_active = false;
    }

    if (t3.phase == T3_PHASE_FINAL_RETURN) {
        update_freeze_return();
    }
}

void T3Task_Stop(void)
{
    t3.active = false;
    t3.phase = T3_PHASE_IDLE;
    t3.arrival_ticks = 0U;
    t3.vision_sample_pending = false;
    t3.has_vision_seq = false;
    t3.stability_active = false;
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();

    if (t3.controller != 0) {
        apply_travel_profile();
        BalanceControl_Enable(t3.controller, false);
        BalanceControl_Reset(t3.controller);
    }

    ControlState_Set(CONTROL_IDLE);
}

void T3Task_StartCenterHold(void)
{
    T3Task_StartTargetHold(0);
}

void T3Task_StartTargetHold(int16_t target_mm)
{
    t3.active = true;
    t3.phase = T3_PHASE_IDLE;
    t3.arrival_ticks = 0U;
    t3.vision_sample_pending = false;
    t3.has_vision_seq = false;
    t3.stability_active = false;
    t3.capture_profile_active = false;
    t3.final_profile_active = false;
    clear_final_freeze();
    target_mm = clamp_target_mm(target_mm);
    t3.target_mm = target_mm;
    t3.reference_mm = (float)target_mm;
    t3.vision_seq = 0U;
    t3.last_vision_tick = 0U;
    t3.stability_start_tick = 0U;
    t3.elapsed_ticks_10ms = 0U;

    if (t3.controller != 0) {
        BalanceControl_Reset(t3.controller);
        BalanceControl_Enable(t3.controller, true);
        apply_center_profile();
        BalanceControl_SetReference(t3.controller, t3.reference_mm / 10.0f);
    }
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

int16_t T3Task_GetBallXMM(void)
{
    return t3.x_mm;
}

bool T3Task_HasValidVision(void)
{
    return t3.vision_valid && !t3.vision_timed_out;
}

uint32_t T3Task_GetElapsedTicks10ms(void)
{
    return t3.elapsed_ticks_10ms;
}

void T3Task_OLED(void)
{
    char buf[22];
    int32_t pwm_delta = 0;
    uint32_t elapsed = T3Task_GetElapsedTicks10ms();
    const char *status = phase_label();

    if (t3.controller != 0) {
        pwm_delta = (int32_t)t3.controller->pwm_pulse -
            (int32_t)(t3.controller->pwm_neutral + 0.5f);
    }

    show_fixed_line(1, "T3 Ball Static");

    if (t3.vision_valid) {
        snprintf(buf, sizeof(buf), "X%c%03d P%c%03ld",
                 sign_of_i32(t3.x_mm),
                 (int)abs_i32(t3.x_mm),
                 sign_of_i32(pwm_delta),
                 (long)abs_i32(pwm_delta));
    } else if (t3.vision_timed_out) {
        snprintf(buf, sizeof(buf), "X---- P%c%03ld",
                 sign_of_i32(pwm_delta),
                 (long)abs_i32(pwm_delta));
        status = "TIMEOUT";
    } else {
        snprintf(buf, sizeof(buf), "X---- P%c%03ld",
                 sign_of_i32(pwm_delta),
                 (long)abs_i32(pwm_delta));
        status = "LOST";
    }
    show_fixed_line(2, buf);

    snprintf(buf, sizeof(buf), "T%c%03d S%04u",
             sign_of_i32(t3.target_mm),
             (int)abs_i32(t3.target_mm),
             (unsigned)t3.vision_seq);
    show_fixed_line(3, buf);

    snprintf(buf, sizeof(buf), "%02lu.%02lus %s",
             (unsigned long)(elapsed / 100U),
             (unsigned long)(elapsed % 100U),
             status);
    show_fixed_line(4, buf);
}
