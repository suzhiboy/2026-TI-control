#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define SENSOR_COUNT 8

void Sensor_Read_All(uint8_t results[SENSOR_COUNT]);
float Sensor_Get_Error(void);

#endif
