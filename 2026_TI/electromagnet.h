#ifndef ELECTROMAGNET_H_
#define ELECTROMAGNET_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

void Electromagnet_Init(void);
void Electromagnet_On(void);
void Electromagnet_Off(void);
void Electromagnet_Set(bool state);
bool Electromagnet_GetState(void);

#endif
