/*
 * key_menu.c — 4 键菜单系统实现
 *
 * 按键扫描 + 5 态 FSM + 任务注册表 + OLED 显示 + PB13 复用
 */

#include "key_menu.h"
#include "oled.h"
#include "board_config.h"
#include <stdio.h>

/* ======================================================================== *
 *  任务占位函数 (后续对接实际控制代码)
 * ======================================================================== */

static void T2_Init(void)  { /* TODO: 循迹一圈停车初始化 */ }
static void T2_Run(void)   { /* TODO: 循迹 + 停车控制循环 */ }
static void T2_Stop(void)  { /* TODO: 停车, 关闭电机 */ }

static void T3_Init(void)  { /* TODO: 静态摆球初始化 (target_mm 已设) */ }
static void T3_Run(void)   { /* TODO: 摆球到 target_mm + 稳住 */ }
static void T3_Stop(void)  { /* TODO: 停止摆球控制 */ }

static void T4_Init(void)  { /* TODO: A到B稳中心初始化 */ }
static void T4_Run(void)   { /* TODO: 移动 + 稳球在中心 */ }
static void T4_Stop(void)  { /* TODO: 停止 */ }

static void T5_Init(void)  { /* TODO: 一圈稳中心初始化 */ }
static void T5_Run(void)   { /* TODO: 循迹 + 稳球在中心 */ }
static void T5_Stop(void)  { /* TODO: 停止 */ }

static void T6_Init(void)  { /* TODO: 一圈稳任意点初始化 */ }
static void T6_Run(void)   { /* TODO: 循迹 + 稳球在 target_mm */ }
static void T6_Stop(void)  { /* TODO: 停止 */ }

/* ======================================================================== *
 *  任务注册表
 * ======================================================================== */

static const TaskDef task_table[] = {
    [TASK_T2] = { "T2 一圈停车",     TASK_T2, T2_Init, T2_Run, T2_Stop, true  },
    [TASK_T3] = { "T3 静态球控",     TASK_T3, T3_Init, T3_Run, T3_Stop, false },
    [TASK_T4] = { "T4 A到B稳中心",   TASK_T4, T4_Init, T4_Run, T4_Stop, false },
    [TASK_T5] = { "T5 一圈稳中心",   TASK_T5, T5_Init, T5_Run, T5_Stop, true  },
    [TASK_T6] = { "T6 一圈稳任意点", TASK_T6, T6_Init, T6_Run, T6_Stop, true  },
};

/* ======================================================================== *
 *  全局菜单实例
 * ======================================================================== */

static MenuState menu;

/* ======================================================================== *
 *  状态名查找表
 * ======================================================================== */

static const char *state_names[] = {
    [SYS_MENU]    = "[MENU]",
    [SYS_READY]   = "[READY]",
    [SYS_RUNNING] = "[RUNNING]",
    [SYS_STOPPED] = "[STOPPED]",
    [SYS_FAULT]   = "[FAULT]",
};

/* ======================================================================== *
 *  PB13 模式切换 (K2 与 AD2 分时复用)
 *
 *  K1/K3/K4 由 SYSCFG_DL_GPIO_init() 统一配置为 GPIO 输入+上拉.
 *  K2 (PB13) 在非 RUNNING 态手动切为 GPIO 输入+上拉读键;
 *          在 RUNNING 态切回数字输出供循迹 AD2 使用.
 * ======================================================================== */

static void PB13_SetGPIO(void)
{
    DL_GPIO_initDigitalInputFeatures(
        GPIO_SENSOR_AD2_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void PB13_SetSensorMuxOutput(void)
{
    DL_GPIO_initDigitalOutput(GPIO_SENSOR_AD2_IOMUX);
    DL_GPIO_enableOutput(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
    DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
}

/* ======================================================================== *
 *  按键 GPIO 读取宏
 * ======================================================================== */

/* 上拉输入: 按下=低电平(0), 松开=高电平(非0) */
#define KEY_PRESSED(port, pin)  (DL_GPIO_readPins(port, pin) == 0U)

/* ======================================================================== *
 *  KeyMenu_Init
 * ======================================================================== */

void KeyMenu_Init(void)
{
    /*
     * K1(PB12)/K3(PB2)/K4(PB3) 已由 SYSCFG_DL_GPIO_init() 配置为
     * GPIO 输入 + 上拉 (见 GPIO_KEY 组). 这里只需额外配置 K2(PB13).
     */
    PB13_SetGPIO();  /* PB13 初始为 GPIO 模式 (MENU 态需要读键) */

    /* 状态初始化 */
    menu.state          = SYS_MENU;
    menu.task_id        = TASK_ID_DEFAULT;
    menu.target_mm      = 0;
    menu.boot_ticks     = 0;
    menu.startup_window = true;

    for (int i = 0; i < KEY_COUNT; i++) {
        menu.keys[i].pressed     = false;
        menu.keys[i].short_flag  = false;
        menu.keys[i].long_flag   = false;
        menu.keys[i].press_ticks = 0;
    }
}

/* ======================================================================== *
 *  单键扫描 (每个 10ms tick 调用一次)
 * ======================================================================== */

static void Key_ScanOne(KeyState *k, bool is_pressed)
{
    if (is_pressed) {
        /* 按下中 */
        if (k->press_ticks < 0xFFFFU) {
            k->press_ticks++;
        }
        /* 到达长按阈值: 置长按标志 (仅触发一次, 之后清零计数防重复) */
        if (k->press_ticks == KEY_LONG_TICKS) {
            k->long_flag  = true;
            k->press_ticks = 0;    /* 防止重复触发 */
        }
    } else {
        /* 松开了: 判断是短按还是长按已触发过 */
        if (k->press_ticks > 0 && k->press_ticks < KEY_LONG_TICKS) {
            k->short_flag = true;
        }
        k->press_ticks = 0;
    }
    k->pressed = is_pressed;
}

/* ======================================================================== *
 *  消费按键标志 (返回是否有事件待处理)
 * ======================================================================== */

static bool Key_ConsumeShort(uint8_t idx)
{
    if (menu.keys[idx].short_flag) {
        menu.keys[idx].short_flag = false;
        return true;
    }
    return false;
}

static bool Key_ConsumeLong(uint8_t idx)
{
    if (menu.keys[idx].long_flag) {
        menu.keys[idx].long_flag = false;
        return true;
    }
    return false;
}

/* ======================================================================== *
 *  清除所有按键标志 (状态切换时丢弃残留事件)
 * ======================================================================== */

static void Key_FlushAll(void)
{
    for (int i = 0; i < KEY_COUNT; i++) {
        menu.keys[i].short_flag = false;
        menu.keys[i].long_flag  = false;
        menu.keys[i].press_ticks = 0;
    }
}

/* ======================================================================== *
 *  target_mm — 物理单位: mm（毫米）
 *  注: 1 mm = 0.1 cm, 即每步 0.1 厘米
 *  管道总行程约 ±120 mm (±12 cm), 超出自动 clamp
 * ======================================================================== */

#define TARGET_MIN_MM   (-120)   /* 水管左极限 (mm) = -12.0 cm */
#define TARGET_MAX_MM   ( 120)   /* 水管右极限 (mm) = +12.0 cm */
#define TARGET_STEP_MM  (   1)   /* K3/K4 每按一次增减量: 1 mm = 0.1 cm */

static void Target_Clamp(void)
{
    if (menu.target_mm < TARGET_MIN_MM) menu.target_mm = TARGET_MIN_MM;
    if (menu.target_mm > TARGET_MAX_MM) menu.target_mm = TARGET_MAX_MM;
}

/* ======================================================================== *
 *  状态行为处理
 * ======================================================================== */

static void FSM_Menu(void)
{
    /* K1 短按: task_id++ */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        if ((int)menu.task_id < TASK_ID_MAX) {
            menu.task_id = (TaskID)((int)menu.task_id + 1);
        }
    }

    /* K1 长按: 确认任务 → READY */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 短按: task_id-- */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        if ((int)menu.task_id > TASK_ID_MIN) {
            menu.task_id = (TaskID)((int)menu.task_id - 1);
        }
    }

    /* K2 长按: 恢复默认参数 */
    if (Key_ConsumeLong(KEY_IDX_K2)) {
        menu.task_id   = TASK_ID_DEFAULT;
        menu.target_mm = 0;
    }

    /* 上电 2 秒窗口: K3/K4 调节 target_mm */
    if (menu.startup_window) {
        if (Key_ConsumeShort(KEY_IDX_K3)) {
            menu.target_mm += TARGET_STEP_MM;
            Target_Clamp();
        }
        if (Key_ConsumeShort(KEY_IDX_K4)) {
            menu.target_mm -= TARGET_STEP_MM;
            Target_Clamp();
        }
    }
}

static void FSM_Ready(void)
{
    /* K1 短按: 启动任务 → RUNNING */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        const TaskDef *task = &task_table[menu.task_id];
        /* 切换 PB13 为模拟 (如果任务需要传感器) */
        if (task->needs_sensor) {
            PB13_SetSensorMuxOutput();
        }
        if (task->init) {
            task->init();
        }
        menu.state = SYS_RUNNING;
        Key_FlushAll();
        return;
    }

    /* K2 短按: 回 MENU */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }

    /* K1 长按: 清零编码器 + 计时 + 摆杆零点 */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        /* TODO: 调用编码器清零 API */
        /* TODO: 调用计时清零 API */
        /* TODO: 调用摆杆角度零点校准 API */
    }
}

static void FSM_Running(void)
{
    /* K1 长按: 急停 → STOPPED */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        const TaskDef *task = &task_table[menu.task_id];
        if (task->stop) {
            task->stop();
        }
        /* 恢复 PB13 为 GPIO */
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        Key_FlushAll();
        return;
    }

    /* K3 短按: 人工中止 → STOPPED */
    if (Key_ConsumeShort(KEY_IDX_K3)) {
        const TaskDef *task = &task_table[menu.task_id];
        if (task->stop) {
            task->stop();
        }
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        Key_FlushAll();
        return;
    }

    /* K2 不响应 (PB13 此时用作 AD2) */
}

static void FSM_Stopped(void)
{
    /* K1 短按: 重新进入 READY */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 短按: 回 MENU */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }
}

static void FSM_Fault(void)
{
    /* K1 短按: 清故障 → READY */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 长按: 回 MENU */
    if (Key_ConsumeLong(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }
}

/* ======================================================================== *
 *  KeyMenu_Scan — 10ms 周期入口
 * ======================================================================== */

void KeyMenu_Scan(void)
{
    /* ---- 读取 4 键电平 ---- */
    bool k1 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_12);  /* PB12 */
    bool k2 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_13);  /* PB13 */
    bool k3 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_2);   /* PB2  */
    bool k4 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_3);   /* PB3  */

    /* ---- 逐个扫描 (RUNNING 态跳过 K2, 因为 PB13 是 AD2) ---- */
    Key_ScanOne(&menu.keys[KEY_IDX_K1], k1);
    if (menu.state != SYS_RUNNING) {
        Key_ScanOne(&menu.keys[KEY_IDX_K2], k2);
    }
    Key_ScanOne(&menu.keys[KEY_IDX_K3], k3);
    Key_ScanOne(&menu.keys[KEY_IDX_K4], k4);

    /* ---- 上电计时 ---- */
    if (menu.boot_ticks < 0xFFFFU) {
        menu.boot_ticks++;
    }
    if (menu.boot_ticks >= KEY_STARTUP_TICKS) {
        menu.startup_window = false;
    }

    /* ---- 状态机分发 ---- */
    switch (menu.state) {
        case SYS_MENU:    FSM_Menu();    break;
        case SYS_READY:   FSM_Ready();   break;
        case SYS_RUNNING: FSM_Running(); break;
        case SYS_STOPPED: FSM_Stopped(); break;
        case SYS_FAULT:   FSM_Fault();   break;
    }
}

/* ======================================================================== *
 *  KeyMenu_OLED — 刷新 OLED 4 行菜单信息
 * ======================================================================== */

void KeyMenu_OLED(void)
{
    char buf[22];

    /* Line 1: 任务名 */
    if (menu.task_id >= TASK_ID_MIN && menu.task_id <= TASK_ID_MAX) {
        snprintf(buf, sizeof(buf), "%-16d", (int)menu.task_id);
    } else {
        snprintf(buf, sizeof(buf), "?               ");
    }
    OLED_ShowLineString(1, 1, buf);

    /* Line 2: 目标点 */
    snprintf(buf, sizeof(buf), "target: %4d mm", menu.target_mm);
    OLED_ShowLineString(2, 1, buf);

    /* Line 3: 系统状态 */
    OLED_ShowLineString(3, 1, state_names[menu.state]);
}

/* ======================================================================== *
 *  公共访问器
 * ======================================================================== */

SysState KeyMenu_GetState(void)
{
    return menu.state;
}

TaskID KeyMenu_GetTaskID(void)
{
    return menu.task_id;
}

int16_t KeyMenu_GetTargetMM(void)
{
    return menu.target_mm;
}

void KeyMenu_SetFault(void)
{
    if (menu.state == SYS_RUNNING) {
        const TaskDef *task = &task_table[menu.task_id];
        if (task->stop) {
            task->stop();
        }
        PB13_SetGPIO();
    }
    menu.state = SYS_FAULT;
    Key_FlushAll();
}

const TaskDef *KeyMenu_GetCurrentTask(void)
{
    if (menu.task_id >= TASK_ID_MIN && menu.task_id <= TASK_ID_MAX) {
        return &task_table[menu.task_id];
    }
    return NULL;
}
