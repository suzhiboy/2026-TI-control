/*
 * sys_state.c — 控制算法调度状态机实现
 */

#include "sys_state.h"

volatile ControlState g_control_state = CONTROL_IDLE;

void ControlState_Set(ControlState new_state)
{
    g_control_state = new_state;
}

ControlState ControlState_Get(void)
{
    return g_control_state;
}

const char* ControlState_Name(ControlState s)
{
    switch (s) {
        case CONTROL_IDLE:         return "IDLE";
        case CONTROL_TRACK_ONLY:   return "TRACK_ONLY";
        case CONTROL_STATIC_BALL:  return "STATIC_BALL";
        case CONTROL_DYNAMIC_BALL: return "DYNAMIC_BALL";
        case CONTROL_TASK3:        return "TASK3";
        default:                   return "???";
    }
}