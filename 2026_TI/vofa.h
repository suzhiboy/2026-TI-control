#ifndef VOFA_H_
#define VOFA_H_

#include <stdint.h>

void Vofa_Init(void);
void Vofa_Poll(void);
void Vofa_SendTelemetry(void);
void VOFA_INST_IRQHandler(void);

#endif
