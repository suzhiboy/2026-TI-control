/*
 * sys_state.h — 系统状态机定义
 *
 * 车载平衡滚球运动控制系统 (2025 电赛 H 题)
 *
 * 状态迁移:
 *   STATE_IDLE ──(start)──→ STATE_DYNAMIC_BALL ──(lap_done)──→ STATE_IDLE
 *      │                        │
 *      └──(VOFA cmd)──→ STATE_TRACK_ONLY   ←──(VOFA cmd)───┘
 *      └──(VOFA cmd)──→ STATE_STATIC_BALL  ←──(VOFA cmd)───┘
 */

#ifndef SYS_STATE_H
#define SYS_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== *
 *  状态枚举
 * ======================================================================== */
typedef enum {
    STATE_IDLE          = 0,    /* 小车停止, 摆杆水平 (pwm=1500)             */
    STATE_TRACK_ONLY    = 1,    /* 纯循迹, 摆杆锁定水平                      */
    STATE_STATIC_BALL   = 2,    /* 小车停止, 运行平衡算法稳定小球              */
    STATE_DYNAMIC_BALL  = 3,    /* 循迹 + 平衡算法同时运行                    */
} SysState;

/* ======================================================================== *
 *  全局状态变量
 * ======================================================================== */
extern volatile SysState g_sys_state;

/* ======================================================================== *
 *  API
 * ======================================================================== */

/**
 * @brief  安全切换状态
 * @param  new_state  目标状态
 * @note   自动处理退出旧状态/进入新状态的清理工作
 */
void SysState_Set(SysState new_state);

/**
 * @brief  查询当前状态
 */
SysState SysState_Get(void);

/**
 * @brief  获取描述字符串 (调试用)
 */
const char* SysState_Name(SysState s);

#ifdef __cplusplus
}
#endif

#endif /* SYS_STATE_H */
