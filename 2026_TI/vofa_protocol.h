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
    VOFA_CMD_CLEAR_PID
} VofaCommandType;

typedef enum {
    VOFA_PID_LINE = 0,
    VOFA_PID_SPEED_LEFT,
    VOFA_PID_SPEED_RIGHT
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
    float value;
} VofaCommand;

VofaParseResult VofaProtocol_ParseLine(const char *line, VofaCommand *cmd);

#endif
