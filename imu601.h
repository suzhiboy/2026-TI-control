#ifndef IMU601_H
#define IMU601_H

#include "ti_msp_dl_config.h"
#include "uart.h"
#include "delay.h"
#include <stdint.h>

/*
 * HuiDianZi-601 module:
 *   V -> 5V
 *   G -> GND
 *   T -> MSPM0 UART RX, PA31
 *   R -> MSPM0 UART TX, PA28
 *
 * The board contains an ICM42688 plus a TI MCU. The external connector is the
 * module UART protocol, not the raw ICM42688 I2C/SPI bus.
 */

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;

void IMU601_init(void);
void IMU601_poll(void);

extern Attitude_t current_attitude;

extern volatile uint32_t imu601_rx_count;
extern volatile uint32_t imu601_header_count;
extern volatile uint32_t imu601_valid_count;
extern volatile uint32_t imu601_checksum_error_count;
extern volatile uint8_t imu601_last_byte;
extern volatile uint32_t imu601_aa_count;
extern volatile uint32_t imu601_55_count;
extern volatile uint32_t imu601_printable_count;
extern volatile uint32_t imu601_newline_count;
extern volatile uint32_t imu601_aa55_pair_count;
extern volatile uint32_t imu601_55aa_pair_count;
extern volatile uint32_t imu601_wit_header_count;
extern volatile uint32_t imu601_wit_valid_count;
extern volatile uint32_t imu601_wit_checksum_error_count;
extern volatile uint8_t imu601_last4[4];

#endif /* IMU601_H */
