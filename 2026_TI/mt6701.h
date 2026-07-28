#ifndef MT6701_H
#define MT6701_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define MT6701_I2C_ADDR          (0x06U)
#define MT6701_I2C_ADDR_ALT      (0x46U)
#define MT6701_ANGLE_REG_H       (0x03U)
#define MT6701_RAW_MAX           (16384U)
#define MT6701_I2C_TIMEOUT       (100000UL)

typedef enum {
    MT6701_OK = 0,
    MT6701_ERR_TIMEOUT,
    MT6701_ERR_BUS,
    MT6701_ERR_NULL
} MT6701_Status;

typedef struct {
    uint16_t raw;
    float absolute_angle_deg;
    float angle_deg;
    float zero_deg;
    bool zero_valid;
} MT6701_Data;

void MT6701_Init(MT6701_Data *encoder);
MT6701_Status MT6701_Update(MT6701_Data *encoder);
MT6701_Status MT6701_ReadRaw(uint16_t *raw);
MT6701_Status MT6701_ReadAbsoluteAngle(float *angle_deg);
uint8_t MT6701_GetActiveAddress(void);
float MT6701_NormalizeAngle(float angle_deg);

#endif /* MT6701_H */
