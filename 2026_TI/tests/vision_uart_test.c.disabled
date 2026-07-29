#ifdef HOST_TEST

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vision_uart.h"

static int failures = 0;

static void expect_int(const char *name, int expected, int actual)
{
    if (expected != actual) {
        fprintf(stderr, "FAIL %s: expected %d got %d\n", name, expected, actual);
        failures++;
    }
}

static void expect_bool(const char *name, bool expected, bool actual)
{
    if (expected != actual) {
        fprintf(stderr, "FAIL %s: expected %d got %d\n", name, expected, actual);
        failures++;
    }
}

static void test_parse_ball_line(void)
{
    VisionBallData data;
    VisionParseResult result;

    memset(&data, 0, sizeof(data));
    result = VisionProtocol_ParseLine("BALL,1234,-37,0.82,24", &data, 7U);

    expect_int("parse ball result", VISION_PARSE_OK, result);
    expect_bool("ball valid", true, data.valid);
    expect_bool("ball lost", false, data.lost);
    expect_int("ball seq", 1234, data.seq);
    expect_int("ball x mm", -37, data.x_mm);
    expect_int("ball conf percent", 82, data.conf_percent);
    expect_int("ball fps", 24, data.fps);
    expect_int("ball update tick", 7, (int)data.last_update_tick);
}

static void test_parse_lost_line(void)
{
    VisionBallData data;
    VisionParseResult result;

    memset(&data, 0, sizeof(data));
    result = VisionProtocol_ParseLine("BALL_LOST,1235", &data, 8U);

    expect_int("parse lost result", VISION_PARSE_OK, result);
    expect_bool("lost valid", false, data.valid);
    expect_bool("lost flag", true, data.lost);
    expect_int("lost seq", 1235, data.seq);
    expect_int("lost update tick", 8, (int)data.last_update_tick);
}

static void test_reject_bad_lines(void)
{
    VisionBallData data;

    memset(&data, 0, sizeof(data));
    expect_int("reject unknown", VISION_PARSE_UNKNOWN,
        VisionProtocol_ParseLine("HELLO,1,2,3", &data, 1U));
    expect_int("reject bad x", VISION_PARSE_BAD_VALUE,
        VisionProtocol_ParseLine("BALL,1,999,0.80,20", &data, 1U));
    expect_int("reject bad conf", VISION_PARSE_BAD_VALUE,
        VisionProtocol_ParseLine("BALL,1,0,1.50,20", &data, 1U));
    expect_int("reject bad fps", VISION_PARSE_BAD_VALUE,
        VisionProtocol_ParseLine("BALL,1,0,0.80,255", &data, 1U));
}

static void test_rx_line_buffer_and_timeout(void)
{
    VisionBallData data;

    VisionUart_Init();
    VisionUart_TestFeedString("BALL,9,42,0.75,21\n");
    VisionUart_Poll(10U);
    data = VisionUart_GetLatest();

    expect_bool("rx valid", true, data.valid);
    expect_int("rx seq", 9, data.seq);
    expect_int("rx x", 42, data.x_mm);
    expect_int("rx conf", 75, data.conf_percent);
    expect_int("rx fps", 21, data.fps);

    VisionUart_Poll(31U);
    data = VisionUart_GetLatest();
    expect_bool("timeout valid false", false, data.valid);
    expect_bool("timeout lost true", true, data.lost);
}

int main(void)
{
    test_parse_ball_line();
    test_parse_lost_line();
    test_reject_bad_lines();
    test_rx_line_buffer_and_timeout();

    if (failures != 0) {
        return EXIT_FAILURE;
    }

    printf("PASS vision uart protocol\n");
    return EXIT_SUCCESS;
}

#else

int vision_uart_test_target_build_guard;

#endif
