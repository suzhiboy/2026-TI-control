#include "sensor.h"

// 快速延时约 50us (根据实验调整，确保 74HC151 切换稳定)
#define DELAY_50US  (1600)

// 假设 1 为识别到黑线，0 为白色背景 (根据实际硬件逻辑调整)
#define LINE_DETECTED 1
#define LINE_UNDETECTED 0

/**
 * @brief 通过 3-8 译码器逻辑读取 8 路红外传感器
 * @param results 存储 0/1 状态的数组
 */
void Sensor_Read_All(uint8_t results[8])
{
    for (int i = 0; i < 8; i++) {
        // 设置通道选择引脚 AD0, AD1, AD2
        if (i & 0x01) DL_GPIO_setPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
        
        if (i & 0x02) DL_GPIO_setPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
        
        if (i & 0x04) DL_GPIO_setPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
        
        // 等待选择器切换稳定
        delay_cycles(DELAY_50US);
        
        // 读取公共输出端 OUT 引脚
        results[i] = DL_GPIO_readPins(GPIO_SENSOR_OUT_PORT, GPIO_SENSOR_OUT_PIN) ? LINE_DETECTED : LINE_UNDETECTED;
    }
}

/**
 * @brief 计算巡线偏差值 (带记忆功能)
 * @return 偏差值：负数代表偏左，正数代表偏右，0 代表正中心
 */
float Sensor_Get_Error(void)
{
    uint8_t data[8];
    static float last_valid_error = 0; // 静态变量：记录上一次有效的偏差值
    
    Sensor_Read_All(data);
    
    // 权值分配：根据传感器离中心线的距离赋予权重值
    // 权值越大，修正动作越剧烈
    int weights[8] = {30, 20, 10, 5, -5, -10, -20, -30};
    int weighted_sum = 0;
    int active_count = 0;
    
    for (int i = 0; i < 8; i++) {
        if (data[i] == LINE_DETECTED) { 
            weighted_sum += weights[i];
            active_count++;
        }
    }
    
    /* 情况 1: 正常巡线 (至少有一路探测到黑线) */
    if (active_count > 0) {
        // 计算当前加权平均偏差
        last_valid_error = (float)weighted_sum / active_count;
        return last_valid_error;
    } 
    
    /* 情况 2: 丢失黑线 (所有传感器都未探测到黑线) */
    // 这通常发生在急弯冲出跑道、或是遇到交叉口/间断线
    
    // 逻辑：如果刚才偏差很大（比如偏左 > 10），现在全白，说明线在左边飞掉了
    // 返回一个比正常范围略大的极值（如 35 或 -35），强制 PID 输出最大转向
    if (last_valid_error > 2.0f) {
        return last_valid_error + 0.5f;
    }
    else if (last_valid_error < -2.0f) {
        return last_valid_error - 0.5f;
    }
    
    // 如果初始状态就是全白，或者正中心丢失（可能是断头线），返回 0
    //return 0;
}
