#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define WHEEL_DIAMETER_CM 4.8f
#define ENCODER_RES       13.0f
#define GEAR_RATIO        20.0f
#define PI_F              3.1415926f
#define PULSE_TO_CM       ((WHEEL_DIAMETER_CM * PI_F) / (ENCODER_RES * GEAR_RATIO * 4.0f))

typedef struct {
    int16_t speed_left;
    int16_t speed_right;
    int32_t pulses_left;
    int32_t pulses_right;
    float distance_cm;
} Encoder_Data_t;

extern Encoder_Data_t g_Encoder;

void Encoder_Init(void);
void Encoder_UpdateData_10ms(void);
void Encoder_Clear(void);

#endif
