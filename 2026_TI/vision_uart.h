#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool lost;
    uint16_t seq;
    int16_t x_mm;
    int16_t raw_x_mm;
    uint16_t cx;
    uint16_t cy;
    uint8_t quality;
    uint8_t conf_percent;
    uint8_t fps;
    uint32_t last_update_tick;
} VisionBallData;

typedef enum {
    VISION_PARSE_EMPTY = 0,
    VISION_PARSE_UNKNOWN,
    VISION_PARSE_BAD_VALUE,
    VISION_PARSE_OK,
} VisionParseResult;

void VisionUart_Init(void);
void VisionUart_Poll(uint32_t now_tick_10ms);
VisionBallData VisionUart_GetLatest(void);
VisionParseResult VisionProtocol_ParseLine(const char *line,
                                           VisionBallData *out,
                                           uint32_t now_tick_10ms);

#ifdef HOST_TEST
void VisionUart_TestFeedString(const char *text);
#endif

#endif /* VISION_UART_H */
