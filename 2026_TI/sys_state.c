/*
 * sys_state.c — 系统状态机实现
 */

#include "sys_state.h"

volatile SysState g_sys_state = STATE_IDLE;

void SysState_Set(SysState new_state)
{
    g_sys_state = new_state;
}

SysState SysState_Get(void)
{
    return g_sys_state;
}

const char* SysState_Name(SysState s)
{
    switch (s) {
        case STATE_IDLE:         return "IDLE";
        case STATE_TRACK_ONLY:   return "TRACK_ONLY";
        case STATE_STATIC_BALL:  return "STATIC_BALL";
        case STATE_DYNAMIC_BALL: return "DYNAMIC_BALL";
        default:                 return "???";
    }
}
