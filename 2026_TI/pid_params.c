#include "pid_params.h"

#include <stddef.h>
#include <string.h>

static int finite_nonnegative(float value, float max_value)
{
    return (value == value) && (value >= 0.0f) && (value <= max_value);
}

static int gains_are_valid(const PidGainSet *gains)
{
    return finite_nonnegative(gains->kp, 5000.0f) &&
           finite_nonnegative(gains->ki, 5000.0f) &&
           finite_nonnegative(gains->kd, 5000.0f);
}

void PidParams_SetDefaults(PidTuningParams *params)
{
    if (params == NULL) {
        return;
    }

    memset(params, 0, sizeof(*params));

    params->line.kp = 0.25f;
    params->line.ki = 0.0f;
    params->line.kd = 0.05f;

    params->speed_left.kp = 15.0f;
    params->speed_left.ki = 0.0f;
    params->speed_left.kd = 0.0f;

    params->speed_right.kp = 15.0f;
    params->speed_right.ki = 0.0f;
    params->speed_right.kd = 0.0f;

    params->base_speed = 40.0f;
}

int PidParams_AreValid(const PidTuningParams *params)
{
    if (params == NULL) {
        return 0;
    }

    return gains_are_valid(&params->line) &&
           gains_are_valid(&params->speed_left) &&
           gains_are_valid(&params->speed_right) &&
           finite_nonnegative(params->base_speed, 200.0f) &&
           (params->base_speed > 0.0f);
}

uint32_t PidParams_Crc32(const void *data, uint32_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;

    while (len-- > 0U) {
        crc ^= *bytes++;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

void PidParams_BuildRecord(const PidTuningParams *params, PidTuningRecord *record)
{
    if ((params == NULL) || (record == NULL)) {
        return;
    }

    memset(record, 0, sizeof(*record));
    record->magic = PID_PARAMS_MAGIC;
    record->version = PID_PARAMS_VERSION;
    record->params = *params;
    record->crc = PidParams_Crc32(&record->params, (uint32_t)sizeof(record->params));
}

int PidParams_RecordIsValid(const PidTuningRecord *record)
{
    uint32_t crc;

    if (record == NULL) {
        return 0;
    }
    if ((record->magic != PID_PARAMS_MAGIC) || (record->version != PID_PARAMS_VERSION)) {
        return 0;
    }
    if (!PidParams_AreValid(&record->params)) {
        return 0;
    }

    crc = PidParams_Crc32(&record->params, (uint32_t)sizeof(record->params));
    return crc == record->crc;
}

int PidParams_ExtractRecord(const PidTuningRecord *record, PidTuningParams *params)
{
    if ((record == NULL) || (params == NULL)) {
        return 0;
    }
    if (!PidParams_RecordIsValid(record)) {
        return 0;
    }

    *params = record->params;
    return 1;
}
