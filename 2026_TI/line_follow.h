#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include "pid.h"
#include "pid_params.h"

extern PID_TypeDef pid_line;
extern PID_TypeDef pid_speed_L;
extern PID_TypeDef pid_speed_R;

void LineTrack_Init(void);
void LineTrack_Start(float base_speed);
void LineTrack_Stop(void);
void LineTrack_Reset(void);
void LineTrack_Loop_10ms(void);
void LineTrack_SetParams(const PidTuningParams *params);
void LineTrack_GetParams(PidTuningParams *params);
void LineTrack_ClearPidState(void);
void LineTrack_SetMotorTest(int16_t left_pwm, int16_t right_pwm);
void LineTrack_ExitMotorTest(void);
bool LineTrack_IsRunning(void);
float LineTrack_Get_Error(void);
float LineTrack_Get_TurnOut(void);
float LineTrack_Get_BaseSpeed(void);
float LineTrack_Get_FilteredLeft(void);
float LineTrack_Get_FilteredRight(void);
float LineTrack_Get_LeftTarget(void);
float LineTrack_Get_RightTarget(void);
float LineTrack_Get_LeftPwm(void);
float LineTrack_Get_RightPwm(void);

#endif
