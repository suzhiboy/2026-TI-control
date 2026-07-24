#ifndef __SENSOR_H_
#define __SENSOR_H_

#include "ti_msp_dl_config.h"

/**
 * @brief 读取八路传感器的状态
 * @param results 长度为 8 的数组，存储 0/1（1代表识别到线）
 */
void Sensor_Read_All(uint8_t results[8]);

/**
 * @brief 计算巡线偏差值
 * @return 偏差范围约 -30 到 30，0 代表在正中间
 */
float Sensor_Get_Error(void);

#endif
