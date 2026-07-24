#include "sensor.h"

#define SENSOR_SWITCH_DELAY_CYCLES 1600U
#define LINE_DETECTED             1U
#define LINE_UNDETECTED           0U

void Sensor_Read_All(uint8_t results[SENSOR_COUNT])
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (i & 0x01U) {
            DL_GPIO_setPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
        }

        if (i & 0x02U) {
            DL_GPIO_setPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
        }

        if (i & 0x04U) {
            DL_GPIO_setPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
        }

        delay_cycles(SENSOR_SWITCH_DELAY_CYCLES);

        results[i] = DL_GPIO_readPins(GPIO_SENSOR_OUT_PORT, GPIO_SENSOR_OUT_PIN) ?
                     LINE_DETECTED : LINE_UNDETECTED;
    }
}

float Sensor_Get_Error(void)
{
    uint8_t data[SENSOR_COUNT];
    static float last_valid_error = 0.0f;
    const int weights[SENSOR_COUNT] = {30, 20, 10, 5, -5, -10, -20, -30};
    int weighted_sum = 0;
    int active_count = 0;

    Sensor_Read_All(data);

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (data[i] == LINE_DETECTED) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count > 0) {
        last_valid_error = (float)weighted_sum / (float)active_count;
        return last_valid_error;
    }

    if (last_valid_error > 2.0f) {
        last_valid_error += 0.5f;
    } else if (last_valid_error < -2.0f) {
        last_valid_error -= 0.5f;
    }

    return last_valid_error;
}
