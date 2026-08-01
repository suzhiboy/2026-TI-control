#include "vofa_protocol.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_KEY_MAX_LEN 16

static int streq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

static void normalize_key(const char *begin, const char *end, char *out, size_t out_size)
{
    size_t i = 0;

    while ((begin < end) && isspace((unsigned char)*begin)) {
        begin++;
    }
    while ((end > begin) && isspace((unsigned char)*(end - 1))) {
        end--;
    }

    while ((begin < end) && (i + 1U < out_size)) {
        out[i++] = (char)toupper((unsigned char)*begin++);
    }
    out[i] = '\0';
}

static VofaParseResult parse_float_value(const char *text, float *value)
{
    char *end = NULL;

    while (isspace((unsigned char)*text)) {
        text++;
    }
    if (*text == '\0') {
        return VOFA_PARSE_BAD_VALUE;
    }

    *value = strtof(text, &end);
    if (end == text) {
        return VOFA_PARSE_BAD_VALUE;
    }
    while (isspace((unsigned char)*end)) {
        end++;
    }
    if (*end != '\0') {
        return VOFA_PARSE_BAD_VALUE;
    }

    return VOFA_PARSE_OK;
}

static int parse_pid_key(const char *key, VofaPidGroup *group, VofaPidTerm *term)
{
    if (strncmp(key, "LINE_", 5) == 0) {
        *group = VOFA_PID_LINE;
        key += 5;
    } else if (strncmp(key, "SL_", 3) == 0) {
        *group = VOFA_PID_SPEED_LEFT;
        key += 3;
    } else if (strncmp(key, "SR_", 3) == 0) {
        *group = VOFA_PID_SPEED_RIGHT;
        key += 3;
    } else if (strncmp(key, "BPOS_", 5) == 0) {
        *group = VOFA_PID_BALANCE_POS;
        key += 5;
    } else if (strncmp(key, "BVEL_", 5) == 0) {
        *group = VOFA_PID_BALANCE_VEL;
        key += 5;
    } else {
        return 0;
    }

    if (streq(key, "KP")) {
        *term = VOFA_PID_KP;
    } else if (streq(key, "KI")) {
        *term = VOFA_PID_KI;
    } else if (streq(key, "KD")) {
        *term = VOFA_PID_KD;
    } else {
        return 0;
    }

    return 1;
}

VofaParseResult VofaProtocol_ParseLine(const char *line, VofaCommand *cmd)
{
    const char *eq = NULL;
    char key[VOFA_KEY_MAX_LEN];

    if ((line == NULL) || (cmd == NULL)) {
        return VOFA_PARSE_EMPTY;
    }

    memset(cmd, 0, sizeof(*cmd));

    while (isspace((unsigned char)*line)) {
        line++;
    }
    if (*line == '\0') {
        return VOFA_PARSE_EMPTY;
    }

    eq = strchr(line, '=');
    if (eq == NULL) {
        const char *end = line + strlen(line);
        normalize_key(line, end, key, sizeof(key));

        if (streq(key, "SAVE")) {
            cmd->type = VOFA_CMD_SAVE;
        } else if (streq(key, "LOAD")) {
            cmd->type = VOFA_CMD_LOAD;
        } else if (streq(key, "RESET")) {
            cmd->type = VOFA_CMD_RESET;
        } else if (streq(key, "GET")) {
            cmd->type = VOFA_CMD_GET;
        } else if (streq(key, "PIDCLR")) {
            cmd->type = VOFA_CMD_CLEAR_PID;
        } else if (streq(key, "MTESTEXIT")) {
            cmd->type = VOFA_CMD_MOTOR_TEST;
            cmd->motor_test_side = VOFA_MOTOR_TEST_EXIT;
        } else if (streq(key, "MSTAT")) {
            cmd->type = VOFA_CMD_MOTOR_STATUS;
        } else {
            return VOFA_PARSE_UNKNOWN;
        }

        return VOFA_PARSE_OK;
    }

    normalize_key(line, eq, key, sizeof(key));
    if (parse_float_value(eq + 1, &cmd->value) != VOFA_PARSE_OK) {
        return VOFA_PARSE_BAD_VALUE;
    }

    if (streq(key, "BASE")) {
        if (cmd->value <= 0.0f) {
            return VOFA_PARSE_BAD_VALUE;
        }
        cmd->type = VOFA_CMD_SET_BASE_SPEED;
        return VOFA_PARSE_OK;
    }

    if (streq(key, "MTEST")) {
        if (cmd->value != 0.0f) {
            return VOFA_PARSE_BAD_VALUE;
        }
        cmd->type = VOFA_CMD_MOTOR_TEST;
        cmd->motor_test_side = VOFA_MOTOR_TEST_OFF;
        return VOFA_PARSE_OK;
    }

    if ((streq(key, "MTESTL")) || (streq(key, "MTESTR"))) {
        if ((cmd->value <= 0.0f) || (cmd->value > 2000.0f)) {
            return VOFA_PARSE_BAD_VALUE;
        }
        cmd->type = VOFA_CMD_MOTOR_TEST;
        cmd->motor_test_side = streq(key, "MTESTL") ?
            VOFA_MOTOR_TEST_LEFT : VOFA_MOTOR_TEST_RIGHT;
        return VOFA_PARSE_OK;
    }

    if (parse_pid_key(key, &cmd->pid_group, &cmd->pid_term)) {
        cmd->type = VOFA_CMD_SET_PID;
        return VOFA_PARSE_OK;
    }

    return VOFA_PARSE_UNKNOWN;
}
