#ifndef PID_PARAMS_H_
#define PID_PARAMS_H_

#include <stdint.h>

#define PID_PARAMS_MAGIC   (0x50494456UL)
#define PID_PARAMS_VERSION (1UL)

typedef struct {
    float kp;
    float ki;
    float kd;
} PidGainSet;

typedef struct {
    PidGainSet line;
    PidGainSet speed_left;
    PidGainSet speed_right;
    float base_speed;
} PidTuningParams;

typedef struct {
    uint32_t magic;
    uint32_t version;
    PidTuningParams params;
    uint32_t crc;
    uint32_t reserved[3];
} PidTuningRecord;

void PidParams_SetDefaults(PidTuningParams *params);
int PidParams_AreValid(const PidTuningParams *params);
void PidParams_BuildRecord(const PidTuningParams *params, PidTuningRecord *record);
int PidParams_RecordIsValid(const PidTuningRecord *record);
int PidParams_ExtractRecord(const PidTuningRecord *record, PidTuningParams *params);
uint32_t PidParams_Crc32(const void *data, uint32_t len);

#endif
