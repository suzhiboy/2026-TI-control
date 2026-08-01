#ifndef VOFA_H_
#define VOFA_H_

#include <stdint.h>
#include "balance_control.h"

void Vofa_AttachBalanceControl(BalanceControl_t *controller);
void Vofa_Init(void);
void Vofa_Poll(void);
void Vofa_SendTelemetry(void);
void VOFA_INST_IRQHandler(void);

#endif
