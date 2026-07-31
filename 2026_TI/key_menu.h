/*
 * key_menu.h - six-key task launcher
 *
 * Scan period: 10 ms.
 * Short press: release before KEY_LONG_TICKS.
 * Long press: held until KEY_LONG_TICKS.
 */

#ifndef KEY_MENU_H
#define KEY_MENU_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_COUNT           (6U)

#define KEY_IDX_K1          (0U)
#define KEY_IDX_K2          (1U)
#define KEY_IDX_K3          (2U)
#define KEY_IDX_K4          (3U)
#define KEY_IDX_K5          (4U)
#define KEY_IDX_K6          (5U)

#define KEY_LONG_TICKS      (100U)
#define KEY_STARTUP_TICKS   (200U)

typedef enum {
    SYS_MENU    = 0,
    SYS_READY   = 1,
    SYS_RUNNING = 2,
    SYS_STOPPED = 3,
    SYS_FAULT   = 4,
} SysState;

typedef enum {
    TASK_T1 = 1,
    TASK_T2 = 2,
    TASK_T3 = 3,
    TASK_T4 = 4,
    TASK_T5 = 5,
    TASK_T6 = 6,
} TaskID;

#define TASK_ID_MIN         (1)
#define TASK_ID_MAX         (6)
#define TASK_ID_DEFAULT     (1)

typedef struct {
    bool          pressed;
    bool          short_flag;
    bool          long_flag;
    uint16_t      press_ticks;
} KeyState;

typedef struct {
    const char   *name;
    TaskID        id;
    void        (*init)(void);
    void        (*run)(void);
    void        (*stop)(void);
    void        (*oled)(void);
    bool          needs_sensor;
} TaskDef;

typedef struct {
    SysState      state;
    TaskID        task_id;
    int16_t       target_mm;
    uint32_t      boot_ticks;
    bool          startup_window;
    KeyState      keys[KEY_COUNT];
} MenuState;

void KeyMenu_Init(void);
void KeyMenu_StartTask(TaskID task_id);
void KeyMenu_Scan(void);
void KeyMenu_OLED(void);

SysState KeyMenu_GetState(void);
TaskID KeyMenu_GetTaskID(void);
int16_t KeyMenu_GetTargetMM(void);
void KeyMenu_SetFault(void);
const TaskDef *KeyMenu_GetCurrentTask(void);

extern float user_target_x_cm;

#ifdef __cplusplus
}
#endif

#endif /* KEY_MENU_H */
