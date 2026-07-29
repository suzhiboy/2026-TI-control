#ifdef HOST_TEST

#include <stdio.h>
#include <stdlib.h>

#include "vofa_protocol.h"

static int failures = 0;

static void expect_int(const char *name, int expected, int actual)
{
    if (expected != actual) {
        fprintf(stderr, "FAIL %s: expected %d got %d\n", name, expected, actual);
        failures++;
    }
}

int main(void)
{
    VofaCommand cmd;
    VofaParseResult result;

    result = VofaProtocol_ParseLine("MTESTL=750", &cmd);
    expect_int("parse result", VOFA_PARSE_OK, result);
    expect_int("command type", VOFA_CMD_MOTOR_TEST, cmd.type);
    expect_int("motor test side", VOFA_MOTOR_TEST_LEFT, cmd.motor_test_side);
    expect_int("motor test pwm", 750, (int)cmd.value);

    result = VofaProtocol_ParseLine("MTESTR=1200", &cmd);
    expect_int("right parse result", VOFA_PARSE_OK, result);
    expect_int("right test side", VOFA_MOTOR_TEST_RIGHT, cmd.motor_test_side);
    expect_int("right test pwm", 1200, (int)cmd.value);

    result = VofaProtocol_ParseLine("MTEST=0", &cmd);
    expect_int("stop parse result", VOFA_PARSE_OK, result);
    expect_int("stop test side", VOFA_MOTOR_TEST_OFF, cmd.motor_test_side);

    result = VofaProtocol_ParseLine("MTESTEXIT", &cmd);
    expect_int("exit parse result", VOFA_PARSE_OK, result);
    expect_int("exit test side", VOFA_MOTOR_TEST_EXIT, cmd.motor_test_side);

    result = VofaProtocol_ParseLine("MTESTL=2001", &cmd);
    expect_int("reject over max", VOFA_PARSE_BAD_VALUE, result);

    result = VofaProtocol_ParseLine("MSTAT", &cmd);
    expect_int("mstat parse result", VOFA_PARSE_OK, result);
    expect_int("mstat command type", VOFA_CMD_MOTOR_STATUS, cmd.type);

    if (failures != 0) {
        return EXIT_FAILURE;
    }

    printf("PASS motor test command\n");
    return EXIT_SUCCESS;
}

#else

int vofa_motor_test_protocol_test_target_build_guard;

#endif
