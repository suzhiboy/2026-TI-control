#ifndef VOFA_PROTOCOL_H_
#define VOFA_PROTOCOL_H_

typedef enum {
    VOFA_PARSE_OK = 0,
    VOFA_PARSE_EMPTY,
    VOFA_PARSE_UNKNOWN,
    VOFA_PARSE_BAD_VALUE
} VofaParseResult;

typedef enum {
    VOFA_CMD_NONE = 0,
    VOFA_CMD_SET_PID,
    VOFA_CMD_SET_BASE_SPEED,
    VOFA_CMD_SAVE,
    VOFA_CMD_LOAD,
    VOFA_CMD_RESET,
    VOFA_CMD_GET,
    VOFA_CMD_CLEAR_PID,
    VOFA_CMD_MOTOR_TEST,
    VOFA_CMD_MOTOR_STATUS
} VofaCommandType;

typedef enum {
    VOFA_MOTOR_TEST_OFF = 0,
    VOFA_MOTOR_TEST_LEFT,
    VOFA_MOTOR_TEST_RIGHT,
    VOFA_MOTOR_TEST_EXIT
} VofaMotorTestSide;

typedef enum {
    VOFA_PID_LINE = 0,
    VOFA_PID_SPEED_LEFT,
    VOFA_PID_SPEED_RIGHT,
    VOFA_PID_BALANCE_POS,
    VOFA_PID_BALANCE_VEL
} VofaPidGroup;

typedef enum {
    VOFA_PID_KP = 0,
    VOFA_PID_KI,
    VOFA_PID_KD
} VofaPidTerm;

typedef struct {
    VofaCommandType type;
    VofaPidGroup pid_group;
    VofaPidTerm pid_term;
    VofaMotorTestSide motor_test_side;
    float value;
} VofaCommand;

VofaParseResult VofaProtocol_ParseLine(const char *line, VofaCommand *cmd);

#endif
