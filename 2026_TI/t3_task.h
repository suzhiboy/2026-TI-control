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
#define T3_ARRIVAL_CONFIRM_TICKS   (5U)

typedef enum {
    T3_PHASE_IDLE = 0,
    T3_PHASE_TO_POSITIVE,
    T3_PHASE_TO_NEGATIVE,
    T3_PHASE_HOLD_NEGATIVE,
} T3TaskPhase;

void T3Task_AttachController(BalanceControl_t *controller);
void T3Task_UpdateVision(bool valid, int16_t x_mm);
void T3Task_Start(void);
void T3Task_Run(void);
void T3Task_Stop(void);

bool T3Task_IsActive(void);
T3TaskPhase T3Task_GetPhase(void);
int16_t T3Task_GetTargetMM(void);

#ifdef __cplusplus
}
#endif

#endif /* T3_TASK_H */
