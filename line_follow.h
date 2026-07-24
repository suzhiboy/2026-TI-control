#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include "pid.h"

extern PID_TypeDef pid_line;
extern PID_TypeDef pid_speed_L;
extern PID_TypeDef pid_speed_R;

void LineTrack_Init(void);
void LineTrack_Start(float base_speed);
void LineTrack_Stop(void);
void LineTrack_Reset(void);
void LineTrack_Loop_10ms(void);
bool LineTrack_IsRunning(void);
float LineTrack_Get_Error(void);
float LineTrack_Get_FilteredLeft(void);
float LineTrack_Get_FilteredRight(void);

#endif
