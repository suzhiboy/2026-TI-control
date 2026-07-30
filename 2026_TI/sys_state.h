/*
 * sys_state.h — 控制算法调度状态机
 *
 * 区别于 key_menu.h 的 SysState (MENU/READY/RUNNING 等系统态),
 * 本模块的 ControlState 仅控制 ISR 中调用哪些算法:
 *   IDLE          → 停车, 摆杆水平
 *   TRACK_ONLY    → 纯循迹, 摆杆锁定水平
 *   STATIC_BALL   → 停车调球
 *   DYNAMIC_BALL  → 循迹 + 调球
 */

#ifndef SYS_STATE_H
#define SYS_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== *
 *  控制状态枚举
 * ======================================================================== */
typedef enum {
    CONTROL_IDLE          = 0,    /* 小车停止, 摆杆水平 (pwm=1500)          */
    CONTROL_TRACK_ONLY    = 1,    /* 纯循迹, 摆杆锁定水平                   */
    CONTROL_STATIC_BALL   = 2,    /* 小车停止, 运行平衡算法稳定小球           */
    CONTROL_DYNAMIC_BALL  = 3,    /* 循迹 + 平衡算法同时运行                 */
    CONTROL_TASK3         = 4,    /* 执行要求3：静止状态下+5cm到-5cm        */
} ControlState;

/* ======================================================================== *
 *  全局状态变量
 * ======================================================================== */
extern volatile ControlState g_control_state;

/* ======================================================================== *
 *  API
 * ======================================================================== */

void ControlState_Set(ControlState new_state);
ControlState ControlState_Get(void);
const char* ControlState_Name(ControlState s);

#ifdef __cplusplus
}
#endif

#endif /* SYS_STATE_H */