#include "vision_uart.h"

#include <stdlib.h>
#include <string.h>

#ifndef HOST_TEST
#include "board_config.h"
#include "ti_msp_dl_config.h"
#endif

#define VISION_RX_BUF_SIZE 80U
#define VISION_TIMEOUT_TICKS 20U
#define VISION_X_MM_MIN (-120)
#define VISION_X_MM_MAX (120)
#define VISION_CX_MAX (799)
#define VISION_CY_MAX (479)
#define VISION_QUALITY_MAX (100U)
#define VISION_FPS_MAX (120U)

static volatile char vision_rx_buf[VISION_RX_BUF_SIZE];
static volatile uint8_t vision_rx_len;
static volatile bool vision_line_ready;
static char vision_line[VISION_RX_BUF_SIZE];
static VisionBallData latest_ball;
static bool vision_frame_received;
static volatile VisionUartStats vision_stats;

static bool parse_u16_field(const char *text, uint16_t *out)
{
    char *end = NULL;
    unsigned long value;

    if ((text == NULL) || (*text == '\0')) {
        return false;
    }

    value = strtoul(text, &end, 10);
    if ((*end != '\0') || (value > 65535UL)) {
        return false;
    }

    *out = (uint16_t)value;
    return true;
}

static bool parse_i16_range_field(const char *text, int min_value,
                                  int max_value, int16_t *out)
{
    char *end = NULL;
    long value;

    if ((text == NULL) || (*text == '\0')) {
        return false;
    }

    value = strtol(text, &end, 10);
    if ((*end != '\0') || (value < min_value) || (value > max_value)) {
        return false;
    }

    *out = (int16_t)value;
    return true;
}

static bool parse_u8_range_field(const char *text, uint8_t max_value,
                                 uint8_t *out)
{
    char *end = NULL;
    unsigned long value;

    if ((text == NULL) || (*text == '\0')) {
        return false;
    }

    value = strtoul(text, &end, 10);
    if ((*end != '\0') || (value > (unsigned long)max_value)) {
        return false;
    }

    *out = (uint8_t)value;
    return true;
}

static bool parse_conf_percent(const char *text, uint8_t *out)
{
    const char *dot;
    unsigned int whole;
    unsigned int tenths = 0;
    unsigned int hundredths = 0;

    if ((text == NULL) || (*text == '\0')) {
        return false;
    }

    dot = strchr(text, '.');
    if (dot == NULL) {
        if ((strcmp(text, "0") == 0) || (strcmp(text, "1") == 0)) {
            *out = (text[0] == '1') ? 100U : 0U;
            return true;
        }
        return false;
    }

    if (((dot - text) != 1) || ((text[0] != '0') && (text[0] != '1'))) {
        return false;
    }

    whole = (unsigned int)(text[0] - '0');
    if ((dot[1] < '0') || (dot[1] > '9')) {
        return false;
    }
    tenths = (unsigned int)(dot[1] - '0');
    if (dot[2] != '\0') {
        if ((dot[2] < '0') || (dot[2] > '9') || (dot[3] != '\0')) {
            return false;
        }
        hundredths = (unsigned int)(dot[2] - '0');
    }

    if ((whole == 1U) && ((tenths != 0U) || (hundredths != 0U))) {
        return false;
    }

    *out = (uint8_t)(whole * 100U + tenths * 10U + hundredths);
    return true;
}

static bool split_csv_fields(char *line, char **fields, size_t field_count)
{
    size_t i;
    char *cursor;

    if ((line == NULL) || (fields == NULL) || (field_count == 0U)) {
        return false;
    }

    cursor = line;
    fields[0] = cursor;
    for (i = 1U; i < field_count; i++) {
        cursor = strchr(cursor, ',');
        if (cursor == NULL) {
            return false;
        }
        *cursor++ = '\0';
        fields[i] = cursor;
    }

    if (strchr(fields[field_count - 1U], ',') != NULL) {
        return false;
    }
    return true;
}

VisionParseResult VisionProtocol_ParseLine(const char *line,
                                           VisionBallData *out,
                                           uint32_t now_tick_10ms)
{
    char local[VISION_RX_BUF_SIZE];
    char *fields[9];
    uint8_t valid_field;
    int16_t cx_field;
    int16_t cy_field;
    VisionBallData parsed;

    if ((line == NULL) || (out == NULL) || (line[0] == '\0')) {
        return VISION_PARSE_EMPTY;
    }

    if (strncmp(line, "BALL_LOST,", 10) == 0) {
        memset(&parsed, 0, sizeof(parsed));
        parsed.valid = false;
        parsed.lost = true;
        parsed.last_update_tick = now_tick_10ms;
        if (!parse_u16_field(line + 10, &parsed.seq)) {
            return VISION_PARSE_BAD_VALUE;
        }
        *out = parsed;
        return VISION_PARSE_OK;
    }

    if (strncmp(line, "B,", 2) == 0) {
        strncpy(local, line, sizeof(local) - 1U);
        local[sizeof(local) - 1U] = '\0';
        if (!split_csv_fields(local, fields, 9U)) {
            return VISION_PARSE_BAD_VALUE;
        }

        memset(&parsed, 0, sizeof(parsed));
        parsed.last_update_tick = now_tick_10ms;
        if ((strcmp(fields[0], "B") != 0) ||
            !parse_u16_field(fields[1], &parsed.seq) ||
            !parse_u8_range_field(fields[2], 1U, &valid_field) ||
            !parse_i16_range_field(fields[3], VISION_X_MM_MIN,
                                   VISION_X_MM_MAX, &parsed.x_mm) ||
            !parse_i16_range_field(fields[4], VISION_X_MM_MIN,
                                   VISION_X_MM_MAX, &parsed.raw_x_mm) ||
            !parse_i16_range_field(fields[5], 0, VISION_CX_MAX, &cx_field) ||
            !parse_i16_range_field(fields[6], 0, VISION_CY_MAX, &cy_field) ||
            !parse_u8_range_field(fields[7], VISION_QUALITY_MAX,
                                  &parsed.quality) ||
            !parse_u8_range_field(fields[8], VISION_FPS_MAX, &parsed.fps)) {
            return VISION_PARSE_BAD_VALUE;
        }

        parsed.valid = (valid_field == 1U);
        parsed.lost = !parsed.valid;
        parsed.cx = (uint16_t)cx_field;
        parsed.cy = (uint16_t)cy_field;
        parsed.conf_percent = parsed.quality;
        if (!parsed.valid) {
            parsed.x_mm = 0;
            parsed.raw_x_mm = 0;
            parsed.cx = 0U;
            parsed.cy = 0U;
            parsed.quality = 0U;
            parsed.conf_percent = 0U;
        }
        *out = parsed;
        return VISION_PARSE_OK;
    }

    if (strncmp(line, "BALL,", 5) != 0) {
        return VISION_PARSE_UNKNOWN;
    }

    strncpy(local, line, sizeof(local) - 1U);
    local[sizeof(local) - 1U] = '\0';
    if (!split_csv_fields(local, fields, 5U)) {
        return VISION_PARSE_BAD_VALUE;
    }
    if (strcmp(fields[0], "BALL") != 0) {
        return VISION_PARSE_UNKNOWN;
    }

    memset(&parsed, 0, sizeof(parsed));
    parsed.valid = true;
    parsed.lost = false;
    parsed.last_update_tick = now_tick_10ms;

    if (!parse_u16_field(fields[1], &parsed.seq) ||
        !parse_i16_range_field(fields[2], VISION_X_MM_MIN, VISION_X_MM_MAX,
                               &parsed.x_mm) ||
        !parse_conf_percent(fields[3], &parsed.conf_percent) ||
        !parse_u8_range_field(fields[4], VISION_FPS_MAX, &parsed.fps)) {
        return VISION_PARSE_BAD_VALUE;
    }

    parsed.raw_x_mm = parsed.x_mm;
    parsed.quality = parsed.conf_percent;

    *out = parsed;
    return VISION_PARSE_OK;
}

static void handle_rx_byte(uint8_t byte)
{
    vision_stats.rx_bytes++;

    if (vision_line_ready) {
        return;
    }

    if ((byte == '\r') || (byte == '\n')) {
        if (vision_rx_len > 0U) {
            vision_rx_buf[vision_rx_len] = '\0';
            vision_line_ready = true;
        }
        return;
    }

    if (vision_rx_len < (VISION_RX_BUF_SIZE - 1U)) {
        vision_rx_buf[vision_rx_len++] = (char)byte;
    } else {
        vision_rx_len = 0U;
        vision_stats.rx_overflow++;
    }
}

void VisionUart_Init(void)
{
    memset((void *)vision_rx_buf, 0, sizeof(vision_rx_buf));
    memset(&latest_ball, 0, sizeof(latest_ball));
    vision_rx_len = 0U;
    vision_line_ready = false;
    vision_frame_received = false;
    memset((void *)&vision_stats, 0, sizeof(vision_stats));

#ifndef HOST_TEST
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
#endif
}

void VisionUart_Poll(uint32_t now_tick_10ms)
{
    VisionBallData parsed;

    if (vision_line_ready) {
#ifndef HOST_TEST
        NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
#endif
        strncpy(vision_line, (const char *)vision_rx_buf,
                sizeof(vision_line) - 1U);
        vision_line[sizeof(vision_line) - 1U] = '\0';
        vision_rx_len = 0U;
        vision_rx_buf[0] = '\0';
        vision_line_ready = false;
#ifndef HOST_TEST
        NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
#endif

        VisionParseResult parse_result =
            VisionProtocol_ParseLine(vision_line, &parsed, now_tick_10ms);
        vision_stats.rx_lines++;
        if (parse_result == VISION_PARSE_OK) {
            vision_stats.parse_ok++;
            latest_ball = parsed;
            vision_frame_received = true;
        } else if (parse_result == VISION_PARSE_UNKNOWN) {
            vision_stats.parse_unknown++;
        } else if (parse_result == VISION_PARSE_BAD_VALUE) {
            vision_stats.parse_bad_value++;
        }
    }

    if (((!vision_frame_received) &&
         (now_tick_10ms >= VISION_TIMEOUT_TICKS)) ||
        (vision_frame_received &&
         ((uint32_t)(now_tick_10ms - latest_ball.last_update_tick) >=
          VISION_TIMEOUT_TICKS))) {
        latest_ball.valid = false;
        latest_ball.lost = true;
        latest_ball.timed_out = true;
        latest_ball.x_mm = 0;
        latest_ball.raw_x_mm = 0;
        latest_ball.cx = 0U;
        latest_ball.cy = 0U;
        latest_ball.quality = 0U;
        latest_ball.conf_percent = 0U;
    }
}

VisionBallData VisionUart_GetLatest(void)
{
    return latest_ball;
}

VisionUartStats VisionUart_GetStats(void)
{
    return vision_stats;
}

#ifdef HOST_TEST
void VisionUart_TestFeedString(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        handle_rx_byte((uint8_t)*text++);
    }
}
#else
void VISION_UART_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(VISION_UART_INST)) {
    case DL_UART_IIDX_RX:
        handle_rx_byte(DL_UART_receiveData(VISION_UART_INST));
        break;
    default:
        break;
    }
}
#endif
