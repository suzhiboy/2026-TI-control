#include "imu601.h"

void IMU601_init()
{
    uint8_t IMU_reset[] = {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};
    UART_send_buffer(IMU601_INST, IMU_reset, sizeof(IMU_reset));

    delay_ms(500);

    uint8_t IMU_cali[] = {0xAA, 0x55, 0x60, 0x14, 0x04, 0xCD, 0x4C, 0xB4, 0x43, 0x88};
    UART_send_buffer(IMU601_INST, IMU_cali, sizeof(IMU_cali));

    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

uint8_t RX_buffer[12] = {0};
uint8_t RX_index = 0;
uint8_t last_byte = 0;
uint8_t is_receiving = 0;

#include <stdint.h>

Attitude_t current_attitude;

void parse_attitude_only(const uint8_t *payload, Attitude_t *out_attitude) {
    uint16_t yaw_raw   = (payload[1] << 8) | payload[0];
    int16_t  pitch_raw = (int16_t)((payload[3] << 8) | payload[2]);
    int16_t  roll_raw  = (int16_t)((payload[5] << 8) | payload[4]);
    out_attitude->yaw   = yaw_raw / 100.0f;
    out_attitude->pitch = pitch_raw / 100.0f;
    out_attitude->roll  = roll_raw / 100.0f;
}


void parse_imu601_data()
{
    uint8_t checksum = 0;
    for (int i = 2; i < 11; i++)
    {
        checksum += RX_buffer[i];
    }
    if (checksum == RX_buffer[11])
    {
        parse_attitude_only(&RX_buffer[5], &current_attitude);
    }
}

void IMU601_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(IMU601_INST))
    {
    case DL_UART_IIDX_RX:
    {

        RX_buffer[RX_index] = DL_UART_receiveData(IMU601_INST);
        if (RX_buffer[RX_index] == 0x55 && last_byte == 0xAA)
        {
            RX_index = 2;
            RX_buffer[0] = 0xAA;
            RX_buffer[1] = 0x55;
            is_receiving = 1;
        }
        else
        {
            RX_index++;
        }
        last_byte = RX_buffer[RX_index - 1];
        if (RX_index >= 12)
        {
            RX_index = 0;
            is_receiving = 0;
            parse_imu601_data();
        }
        break;
    }

    default:
        break;
    }
}
