#ifndef T3_TASK_H
#define T3_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "balance_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define T3_POSITIVE_TARGET_MM      (50)
#define T3_NEGATIVE_TARGET_MM      (-50)
#define T3_TARGET_TOLERANCE_MM     (10)
#define T3_FREEZE_TOLERANCE_MM     (5)
#define T3_ARRIVAL_CONFIRM_TICKS   (2U)
#define T3_FINAL_STABLE_TICKS      (10U)
#define T3_VISION_FRESH_TICKS      (20U)
#define T3_STILL_FRAME_FREEZE_COUNT (4U)
#define T3_FREEZE_RETURN_TICKS     (7U)
#define T3_FREEZE_HOLD_BIAS_PERCENT (45U)
#define T3_FREEZE_HOLD_MIN_DELTA_US (45U)
#define T3_TRAVEL_NEGATIVE_DELTA_LIMIT_US (350U)
#define T3_TRAVEL_POSITIVE_DELTA_LIMIT_US (250U)
#define T3_TRAVEL_MIN_DRIVE_US             (170U)
#define T3_RETURN_NEGATIVE_DELTA_LIMIT_US  (125U)
#define T3_RETURN_POSITIVE_DELTA_LIMIT_US  (125U)
#define T3_RETURN_MIN_DRIVE_US             (50U)
#define T3_NEGATIVE_CAPTURE_ENTRY_MM       (40)
#define T3_CAPTURE_NEGATIVE_DELTA_LIMIT_US (80U)
#define T3_CAPTURE_POSITIVE_DELTA_LIMIT_US (35U)
#define T3_CAPTURE_MIN_DRIVE_US            (0U)
#define T3_CAPTURE_POS_KP                  (0.055f)
#define T3_CAPTURE_POS_KD                  (0.06f)
#define T3_CAPTURE_VEL_KP                  (0.032f)
#define T3_FINAL_NEGATIVE_DELTA_LIMIT_US   (65U)
#define T3_FINAL_POSITIVE_DELTA_LIMIT_US   (65U)
#define T3_FINAL_MIN_DRIVE_US              (0U)
#define T3_FINAL_POS_KP                    (0.045f)
#define T3_FINAL_POS_KD                    (0.045f)
#define T3_FINAL_VEL_KP                    (0.024f)
#define T3_CENTER_DELTA_LIMIT_US           (100U)
#define T3_CENTER_MIN_DRIVE_US             (0U)
#define T3_CENTER_POS_KP                   (0.06f)
#define T3_CENTER_POS_KD                   (0.045f)
#define T3_CENTER_VEL_KP                   (0.02f)

typedef enum {
    T3_PHASE_IDLE = 0,
    T3_PHASE_TO_POSITIVE,
    T3_PHASE_TO_NEGATIVE,
    T3_PHASE_FINAL_RETURN,
    T3_PHASE_HOLD_NEGATIVE,
} T3TaskPhase;

void T3Task_AttachController(BalanceControl_t *controller);
void T3Task_UpdateVision(bool valid, bool timed_out, uint16_t seq, int16_t x_mm);
void T3Task_Tick10ms(void);
void T3Task_Start(void);
void T3Task_Run(void);
void T3Task_Stop(void);
void T3Task_StartCenterHold(void);
void T3Task_StartTargetHold(int16_t target_mm);

bool T3Task_IsActive(void);
T3TaskPhase T3Task_GetPhase(void);
int16_t T3Task_GetTargetMM(void);
int16_t T3Task_GetBallXMM(void);
bool T3Task_HasValidVision(void);
uint32_t T3Task_GetElapsedTicks10ms(void);
void T3Task_OLED(void);

#ifdef __cplusplus
}
#endif

#endif /* T3_TASK_H */
