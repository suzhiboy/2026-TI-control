/*
 * key_menu.h — 4 键菜单系统 (按键扫描 + 状态机 + 任务注册)
 *
 * 平台: MSPM0G3507  扫描周期: 10ms
 * 按键: K1=PB12  K2=PB13(与AD2复用)  K3=PB2  K4=PB3
 *
 * 长按 >= 1000ms (100 ticks), 短按 < 1000ms
 */

#ifndef KEY_MENU_H
#define KEY_MENU_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== *
 *  按键索引
 * ======================================================================== */

#define KEY_COUNT           (4U)

#define KEY_IDX_K1          (0U)   /* PB12 */
#define KEY_IDX_K2          (1U)   /* PB13 */
#define KEY_IDX_K3          (2U)   /* PB2  */
#define KEY_IDX_K4          (3U)   /* PB3  */

/* ======================================================================== *
 *  长按/短按阈值 (10ms ticks)
 * ======================================================================== */

#define KEY_LONG_TICKS      (100U)  /* 1000ms */
#define KEY_STARTUP_TICKS   (200U)  /* 2000ms 上电 target 调节窗口 */

/* ======================================================================== *
 *  系统状态
 * ======================================================================== */

typedef enum {
    SYS_MENU    = 0,
    SYS_READY   = 1,
    SYS_RUNNING = 2,
    SYS_STOPPED = 3,
    SYS_FAULT   = 4,
} SysState;

/* ======================================================================== *
 *  任务 ID
 * ======================================================================== */

typedef enum {
    TASK_T2 = 2,   /* 一圈停车 */
    TASK_T3 = 3,   /* 静态球控 */
    TASK_T4 = 4,   /* A 到 B 稳中心 */
    TASK_T5 = 5,   /* 一圈稳中心 */
    TASK_T6 = 6,   /* 一圈稳任意点 */
} TaskID;

#define TASK_ID_MIN         (2)
#define TASK_ID_MAX         (6)
#define TASK_ID_DEFAULT     (2)

/* ======================================================================== *
 *  按键事件标志 (每个键独立)
 * ======================================================================== */

typedef struct {
    bool          pressed;         /* 当前是否按下                    */
    bool          short_flag;      /* 短按事件 (消费后清零)           */
    bool          long_flag;       /* 长按事件 (消费后清零)           */
    uint16_t      press_ticks;     /* 按下持续 tick 数                */
} KeyState;

/* ======================================================================== *
 *  任务定义
 * ======================================================================== */

typedef struct {
    const char   *name;            /* OLED 显示名称                   */
    TaskID        id;              /* 任务 ID                         */
    void        (*init)(void);     /* 进入 RUNNING 前调用一次         */
    void        (*run)(void);      /* 10ms 周期调用                   */
    void        (*stop)(void);     /* 离开 RUNNING 时调用             */
    bool          needs_sensor;    /* 是否需要 AD2 传感器 (影响 PB13)  */
} TaskDef;

/* ======================================================================== *
 *  全局菜单状态
 * ======================================================================== */

typedef struct {
    SysState      state;           /* 当前系统状态                    */
    TaskID        task_id;         /* 当前选中的任务                  */
    int16_t       target_mm;       /* 目标点 mm (T3/T6 使用)          */
    uint32_t      boot_ticks;      /* 上电后累计 tick                 */
    bool          startup_window;  /* 上电 2 秒窗口是否仍有效         */
    KeyState      keys[KEY_COUNT]; /* 4 键状态数组                    */
} MenuState;

/* ======================================================================== *
 *  公共 API
 * ======================================================================== */

/**
 * @brief  初始化按键 GPIO 和菜单状态机
 *
 *         配置 PB12/PB13/PB2/PB3 为 GPIO 输入 + 上拉.
 *         PB13 初始化为 GPIO 输入模式 (非 RUNNING 状态使用).
 *         系统初始状态 = MENU, 任务默认 = T4, target_mm = 0.
 */
void KeyMenu_Init(void);
void KeyMenu_StartTask(TaskID task_id);

/**
 * @brief  10ms 周期按键扫描 + 状态机处理
 *
 *         调用位置: TIMER_0 ISR (10ms 周期)
 *
 *         每 tick:
 *           - 读取 4 键电平, 更新 press_ticks
 *           - 检测短按/长按事件, 置位 flags
 *           - 根据当前状态路由到对应行为
 *           - boot_ticks 累加, 200 ticks 后关闭启动窗口
 */
void KeyMenu_Scan(void);

/**
 * @brief  将菜单信息刷新到 OLED 4 行显示
 *
 *         调用位置: main loop (低频, 例如每 10 ticks)
 *
 *         Line1: 任务名
 *         Line2: target_mm
 *         Line3: 系统状态标签
 *         Line4: 保留 (由调用方在之后覆盖小球信息)
 */
void KeyMenu_OLED(void);

/**
 * @brief  获取当前系统状态
 */
SysState KeyMenu_GetState(void);

/**
 * @brief  获取当前任务 ID
 */
TaskID KeyMenu_GetTaskID(void);

/**
 * @brief  获取当前 target_mm
 */
int16_t KeyMenu_GetTargetMM(void);

/**
 * @brief  触发故障 (外部模块调用, 进入 FAULT 状态)
 */
void KeyMenu_SetFault(void);

/**
 * @brief  获取当前选中任务的 TaskDef 指针 (用于 RUNNING 时调用 run)
 */
const TaskDef *KeyMenu_GetCurrentTask(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_MENU_H */
