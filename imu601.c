#include "imu601.h"

#define IMU601_MAX_FRAME_LEN 32
#define WIT_FRAME_LEN 11

static uint8_t RX_buffer[IMU601_MAX_FRAME_LEN] = {0};
static uint8_t RX_index = 0;
static uint8_t RX_expected_len = 0;

static uint8_t WIT_buffer[WIT_FRAME_LEN] = {0};
static uint8_t WIT_index = 0;

Attitude_t current_attitude = {0};

volatile uint32_t imu601_rx_count = 0;
volatile uint32_t imu601_header_count = 0;
volatile uint32_t imu601_valid_count = 0;
volatile uint32_t imu601_checksum_error_count = 0;
volatile uint8_t imu601_last_byte = 0;
volatile uint32_t imu601_aa_count = 0;
volatile uint32_t imu601_55_count = 0;
volatile uint32_t imu601_printable_count = 0;
volatile uint32_t imu601_newline_count = 0;
volatile uint32_t imu601_aa55_pair_count = 0;
volatile uint32_t imu601_55aa_pair_count = 0;
volatile uint32_t imu601_wit_header_count = 0;
volatile uint32_t imu601_wit_valid_count = 0;
volatile uint32_t imu601_wit_checksum_error_count = 0;
volatile uint8_t imu601_last4[4] = {0};

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void parse_attitude_only(const uint8_t *payload, Attitude_t *out_attitude)
{
    out_attitude->yaw = read_u16_le(&payload[0]) / 100.0f;
    out_attitude->pitch = read_i16_le(&payload[2]) / 100.0f;
    out_attitude->roll = read_i16_le(&payload[4]) / 100.0f;
}

static void parse_attitude_full(const uint8_t *payload, Attitude_t *out_attitude)
{
    out_attitude->pitch = read_i16_le(&payload[12]) / 100.0f;
    out_attitude->roll = read_i16_le(&payload[14]) / 100.0f;
    out_attitude->yaw = read_u16_le(&payload[16]) / 100.0f;
}

static void parse_wit_angle(const uint8_t *frame, Attitude_t *out_attitude)
{
    int16_t roll_raw = read_i16_le(&frame[2]);
    int16_t pitch_raw = read_i16_le(&frame[4]);
    int16_t yaw_raw = read_i16_le(&frame[6]);

    out_attitude->roll = roll_raw / 32768.0f * 180.0f;
    out_attitude->pitch = pitch_raw / 32768.0f * 180.0f;
    out_attitude->yaw = yaw_raw / 32768.0f * 180.0f;
}

void IMU601_init(void)
{
    const uint8_t imu_reset[] = {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};
    const uint8_t imu_cali[] = {0xAA, 0x55, 0x60, 0x14, 0x04,
                                0xCD, 0x4C, 0xB4, 0x43, 0x88};

    UART_send_buffer(IMU601_INST, imu_reset, sizeof(imu_reset));
    delay_ms(500);

    UART_send_buffer(IMU601_INST, imu_cali, sizeof(imu_cali));
    delay_ms(20);

    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

static void IMU601_parse_protocol_frame(void)
{
    uint8_t checksum = 0;
    uint8_t data_len = RX_buffer[4];
    uint8_t frame_len = (uint8_t)(data_len + 6);

    if (frame_len > IMU601_MAX_FRAME_LEN)
    {
        imu601_checksum_error_count++;
        return;
    }

    for (uint8_t i = 2; i < (uint8_t)(frame_len - 1); i++)
    {
        checksum += RX_buffer[i];
    }

    if (checksum != RX_buffer[frame_len - 1])
    {
        imu601_checksum_error_count++;
        return;
    }

    if (data_len == 6)
    {
        parse_attitude_only(&RX_buffer[5], &current_attitude);
    }
    else if (data_len == 18)
    {
        parse_attitude_full(&RX_buffer[5], &current_attitude);
    }

    imu601_valid_count++;
}

static void IMU601_handle_wit_byte(uint8_t byte)
{
    if (WIT_index == 0)
    {
        if (byte == 0x55)
        {
            WIT_buffer[0] = byte;
            WIT_index = 1;
        }
        return;
    }

    if (WIT_index == 1)
    {
        if (byte == 0x53)
        {
            WIT_buffer[1] = byte;
            WIT_index = 2;
            imu601_wit_header_count++;
        }
        else
        {
            WIT_index = (byte == 0x55) ? 1 : 0;
            WIT_buffer[0] = byte;
        }
        return;
    }

    WIT_buffer[WIT_index++] = byte;
    if (WIT_index >= WIT_FRAME_LEN)
    {
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < (WIT_FRAME_LEN - 1); i++)
        {
            checksum += WIT_buffer[i];
        }

        if (checksum == WIT_buffer[WIT_FRAME_LEN - 1])
        {
            parse_wit_angle(WIT_buffer, &current_attitude);
            imu601_wit_valid_count++;
            imu601_valid_count++;
        }
        else
        {
            imu601_wit_checksum_error_count++;
        }

        WIT_index = 0;
    }
}

static void IMU601_handle_protocol_byte(uint8_t byte)
{
    if (RX_index == 0)
    {
        if (byte == 0xAA)
        {
            RX_buffer[0] = byte;
            RX_index = 1;
        }
        return;
    }

    if (RX_index == 1)
    {
        if (byte == 0x55)
        {
            RX_buffer[1] = byte;
            RX_index = 2;
            RX_expected_len = 0;
            imu601_header_count++;
        }
        else
        {
            RX_index = (byte == 0xAA) ? 1 : 0;
            RX_buffer[0] = byte;
        }
        return;
    }

    RX_buffer[RX_index++] = byte;

    if (RX_index == 5)
    {
        if (RX_buffer[4] > (IMU601_MAX_FRAME_LEN - 6))
        {
            RX_index = 0;
            RX_expected_len = 0;
            imu601_checksum_error_count++;
            return;
        }

        RX_expected_len = (uint8_t)(RX_buffer[4] + 6);
    }

    if (RX_expected_len != 0 && RX_index >= RX_expected_len)
    {
        IMU601_parse_protocol_frame();
        RX_index = 0;
        RX_expected_len = 0;
    }
}

static void IMU601_handle_byte(uint8_t byte)
{
    uint8_t prev_byte = imu601_last_byte;

    imu601_rx_count++;
    imu601_last_byte = byte;
    imu601_last4[0] = imu601_last4[1];
    imu601_last4[1] = imu601_last4[2];
    imu601_last4[2] = imu601_last4[3];
    imu601_last4[3] = byte;

    if (byte == 0xAA)
    {
        imu601_aa_count++;
    }
    if (byte == 0x55)
    {
        imu601_55_count++;
    }
    if (byte >= 0x20 && byte <= 0x7E)
    {
        imu601_printable_count++;
    }
    if (byte == 0x0D || byte == 0x0A)
    {
        imu601_newline_count++;
    }
    if (prev_byte == 0xAA && byte == 0x55)
    {
        imu601_aa55_pair_count++;
    }
    if (prev_byte == 0x55 && byte == 0xAA)
    {
        imu601_55aa_pair_count++;
    }

    IMU601_handle_wit_byte(byte);
    IMU601_handle_protocol_byte(byte);
}

void IMU601_poll(void)
{
    uint8_t byte;

    NVIC_DisableIRQ(IMU601_INST_INT_IRQN);
    while (DL_UART_receiveDataCheck(IMU601_INST, &byte))
    {
        IMU601_handle_byte(byte);
    }
    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

void IMU601_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(IMU601_INST))
    {
    case DL_UART_IIDX_RX:
        IMU601_handle_byte(DL_UART_receiveData(IMU601_INST));
        break;

    default:
        break;
    }
}
