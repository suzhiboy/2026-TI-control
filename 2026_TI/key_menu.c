/*
 * key_menu.c 鈥?4 閿彍鍗曠郴缁熷疄鐜?
 *
 * 鎸夐敭鎵弿 + 5 鎬?FSM + 浠诲姟娉ㄥ唽琛?+ OLED 鏄剧ず + PB13 澶嶇敤
 */

#include "key_menu.h"
#include "encoder.h"
#include "line_follow.h"
#include "oled.h"
#include "board_config.h"
#include "sensor.h"
#include "sys_state.h"
#include "t3_task.h"
#include <stdio.h>

static void PB13_SetGPIO(void);
static void PB13_SetSensorMuxOutput(void);
static void Start_SelectedTask(void);

#define T2_LAP_DISTANCE_CM          (614.0f)
#define T2_START_LINE_IGNORE_CM     (100.0f)
#define T2_SEARCH_START_CM          (560.0f)
#define T2_FAILSAFE_OVERRUN_CM      (5.0f)
#define T2_STOP_COMPENSATION_CM     (5.0f)
#define T2_MIN_RUN_TICKS_10MS       (100U)
#define T2_LINE_ACTIVE_MIN          (3U)
#define T2_LINE_ACTIVE_MAX          (4U)
#define T2_LINE_CONFIRM_TICKS       (3U)

typedef enum {
    T2_STATE_IDLE = 0,
    T2_STATE_IGNORE_START_LINE,
    T2_STATE_LAP,
    T2_STATE_FIND_A_LINE,
    T2_STATE_ADVANCE_TO_MARK,
    T2_STATE_BRAKE,
} T2State;

static MenuState menu;
static uint32_t t2_elapsed_ticks_10ms = 0U;
static uint32_t t2_last_logic_tick = 0U;
static uint32_t t2_finish_ticks_10ms = 0U;
static bool t2_brake_requested = false;
static T2State t2_state = T2_STATE_IDLE;
static float t2_line_seen_distance_cm = 0.0f;
static uint8_t t2_line_confirm_ticks = 0U;

static uint8_t T2_CountActiveSensors(void)
{
    uint8_t data[SENSOR_COUNT];
    uint8_t active_count = 0U;

    Sensor_Read_All(data);
    for (uint8_t i = 0U; i < SENSOR_COUNT; i++) {
        if (data[i] != 0U) {
            active_count++;
        }
    }

    return active_count;
}

static bool T2_StopLineDetected(void)
{
    uint8_t active_count = T2_CountActiveSensors();

    return (active_count >= T2_LINE_ACTIVE_MIN) &&
           (active_count <= T2_LINE_ACTIVE_MAX);
}

static void T2_RequestBrake(void)
{
    if (!t2_brake_requested) {
        t2_brake_requested = true;
        t2_state = T2_STATE_BRAKE;
        LineTrack_Brake();
    }
}

static void T2_Init(void)
{
    T3Task_Stop();
    t2_elapsed_ticks_10ms = 0U;
    t2_last_logic_tick = 0U;
    t2_finish_ticks_10ms = 0U;
    t2_brake_requested = false;
    t2_state = T2_STATE_IGNORE_START_LINE;
    t2_line_seen_distance_cm = 0.0f;
    t2_line_confirm_ticks = 0U;
    ControlState_Set(CONTROL_TRACK_ONLY);
    LineTrack_Start(LineTrack_Get_BaseSpeed());
}
static void T2_Run(void)
{
    if (t2_finish_ticks_10ms != 0U) {
        return;
    }

    if (t2_last_logic_tick == t2_elapsed_ticks_10ms) {
        return;
    }
    t2_last_logic_tick = t2_elapsed_ticks_10ms;

    switch (t2_state) {
        case T2_STATE_IGNORE_START_LINE:
            if ((t2_elapsed_ticks_10ms >= T2_MIN_RUN_TICKS_10MS) &&
                (g_Encoder.distance_cm >= T2_START_LINE_IGNORE_CM)) {
                t2_state = T2_STATE_LAP;
            }
            break;

        case T2_STATE_LAP:
            if (g_Encoder.distance_cm >= T2_SEARCH_START_CM) {
                t2_state = T2_STATE_FIND_A_LINE;
                t2_line_confirm_ticks = 0U;
            }
            break;

        case T2_STATE_FIND_A_LINE:
            if (T2_StopLineDetected()) {
                if (t2_line_confirm_ticks < T2_LINE_CONFIRM_TICKS) {
                    t2_line_confirm_ticks++;
                }
            } else {
                t2_line_confirm_ticks = 0U;
            }

            if (t2_line_confirm_ticks >= T2_LINE_CONFIRM_TICKS) {
                t2_line_seen_distance_cm = g_Encoder.distance_cm;
                t2_state = T2_STATE_ADVANCE_TO_MARK;
            } else if (g_Encoder.distance_cm >=
                       (T2_LAP_DISTANCE_CM + T2_FAILSAFE_OVERRUN_CM)) {
                T2_RequestBrake();
            }
            break;

        case T2_STATE_ADVANCE_TO_MARK:
            if ((g_Encoder.distance_cm - t2_line_seen_distance_cm) >=
                T2_STOP_COMPENSATION_CM) {
                T2_RequestBrake();
            }
            break;

        case T2_STATE_BRAKE:
        case T2_STATE_IDLE:
        default:
            break;
    }

    if (t2_brake_requested && !LineTrack_IsRunning()) {
        t2_finish_ticks_10ms = t2_elapsed_ticks_10ms;
        ControlState_Set(CONTROL_IDLE);
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        t2_state = T2_STATE_IDLE;
    }
}
static void T2_Stop(void)
{
    /* ISR 涓?CONTROL_IDLE 鍒嗘敮浼氬仠鐢垫満 + 澶嶄綅绉垎 + PWM=1500 */
    ControlState_Set(CONTROL_IDLE);
}

static void T3_Init(void)  { T3Task_Start(); }
static void T3_Run(void)   { T3Task_Run(); }
static void T3_Stop(void)  { T3Task_Stop(); }

static void T4_Init(void)  { /* TODO: A鍒癇绋充腑蹇冨垵濮嬪寲 */ }
static void T4_Run(void)   { /* TODO: 绉诲姩 + 绋崇悆鍦ㄤ腑蹇?*/ }
static void T4_Stop(void)  { /* TODO: 鍋滄 */ }

static void T5_Init(void)  { /* TODO: 涓€鍦堢ǔ涓績鍒濆鍖?*/ }
static void T5_Run(void)   { /* TODO: 寰抗 + 绋崇悆鍦ㄤ腑蹇?*/ }
static void T5_Stop(void)  { /* TODO: 鍋滄 */ }

static void T6_Init(void)  { /* TODO: 涓€鍦堢ǔ浠绘剰鐐瑰垵濮嬪寲 */ }
static void T6_Run(void)   { /* TODO: 寰抗 + 绋崇悆鍦?target_mm */ }
static void T6_Stop(void)  { /* TODO: 鍋滄 */ }

/* ======================================================================== *
 *  浠诲姟娉ㄥ唽琛?
 * ======================================================================== */

static const TaskDef task_table[] = {
    [TASK_T2] = { "T2 Lap Stop",    TASK_T2, T2_Init, T2_Run, T2_Stop, true  },
    [TASK_T3] = { "T3 Ball Static", TASK_T3, T3_Init, T3_Run, T3_Stop, false },
    [TASK_T4] = { "T4 AB Center",   TASK_T4, T4_Init, T4_Run, T4_Stop, false },
    [TASK_T5] = { "T5 Lap Center",  TASK_T5, T5_Init, T5_Run, T5_Stop, true  },
    [TASK_T6] = { "T6 Lap Target",  TASK_T6, T6_Init, T6_Run, T6_Stop, true  },
};

/* ======================================================================== *
 *  鐘舵€佸悕鏌ユ壘琛?
 * ======================================================================== */

static const char *state_names[] = {
    [SYS_MENU]    = "[MENU]",
    [SYS_READY]   = "[READY]",
    [SYS_RUNNING] = "[RUNNING]",
    [SYS_STOPPED] = "[STOPPED]",
    [SYS_FAULT]   = "[FAULT]",
};

/* ======================================================================== *
 *  PB13 妯″紡鍒囨崲 (K2 涓?AD2 鍒嗘椂澶嶇敤)
 *
 *  K1/K3/K4 鐢?SYSCFG_DL_GPIO_init() 缁熶竴閰嶇疆涓?GPIO 杈撳叆+涓婃媺.
 *  K2 (PB13) 鍦ㄩ潪 RUNNING 鎬佹墜鍔ㄥ垏涓?GPIO 杈撳叆+涓婃媺璇婚敭;
 *          鍦?RUNNING 鎬佸垏鍥炴暟瀛楄緭鍑轰緵寰抗 AD2 浣跨敤.
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
 *  鎸夐敭 GPIO 璇诲彇瀹?
 * ======================================================================== */

/* 涓婃媺杈撳叆: 鎸変笅=浣庣數骞?0), 鏉惧紑=楂樼數骞?闈?) */
#define KEY_PRESSED(port, pin)  (DL_GPIO_readPins(port, pin) == 0U)

/* ======================================================================== *
 *  KeyMenu_Init
 * ======================================================================== */

void KeyMenu_Init(void)
{
    /*
     * K1(PB12)/K3(PB2)/K4(PB3) 宸茬敱 SYSCFG_DL_GPIO_init() 閰嶇疆涓?
     * GPIO 杈撳叆 + 涓婃媺 (瑙?GPIO_KEY 缁?. 杩欓噷鍙渶棰濆閰嶇疆 K2(PB13).
     */
    PB13_SetGPIO();  /* PB13 鍒濆涓?GPIO 妯″紡 (MENU 鎬侀渶瑕佽閿? */

    /* 鐘舵€佸垵濮嬪寲 */
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

void KeyMenu_StartTask(TaskID task_id)
{
    if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)) {
        return;
    }

    menu.task_id = task_id;
    Start_SelectedTask();
}

/* ======================================================================== *
 *  鍗曢敭鎵弿 (姣忎釜 10ms tick 璋冪敤涓€娆?
 * ======================================================================== */

static void Key_ScanOne(KeyState *k, bool is_pressed)
{
    if (is_pressed) {
        /* 鎸変笅涓?*/
        if (k->press_ticks < 0xFFFFU) {
            k->press_ticks++;
        }
        /* 鍒拌揪闀挎寜闃堝€? 缃暱鎸夋爣蹇?(浠呰Е鍙戜竴娆? 涔嬪悗娓呴浂璁℃暟闃查噸澶? */
        if (k->press_ticks == KEY_LONG_TICKS) {
            k->long_flag  = true;
            k->press_ticks = 0;    /* 闃叉閲嶅瑙﹀彂 */
        }
    } else {
        /* 鏉惧紑浜? 鍒ゆ柇鏄煭鎸夎繕鏄暱鎸夊凡瑙﹀彂杩?*/
        if (k->press_ticks > 0 && k->press_ticks < KEY_LONG_TICKS) {
            k->short_flag = true;
        }
        k->press_ticks = 0;
    }
    k->pressed = is_pressed;
}

/* ======================================================================== *
 *  娑堣垂鎸夐敭鏍囧織 (杩斿洖鏄惁鏈変簨浠跺緟澶勭悊)
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
 *  娓呴櫎鎵€鏈夋寜閿爣蹇?(鐘舵€佸垏鎹㈡椂涓㈠純娈嬬暀浜嬩欢)
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
 *  target_mm 鈥?鐗╃悊鍗曚綅: mm锛堟绫筹級
 *  娉? 1 mm = 0.1 cm, 鍗虫瘡姝?0.1 鍘樼背
 *  绠￠亾鎬昏绋嬬害 卤120 mm (卤12 cm), 瓒呭嚭鑷姩 clamp
 * ======================================================================== */

#define TARGET_MIN_MM   (-120)   /* 姘寸宸︽瀬闄?(mm) = -12.0 cm */
#define TARGET_MAX_MM   ( 120)   /* 姘寸鍙虫瀬闄?(mm) = +12.0 cm */
#define TARGET_STEP_MM  (   1)   /* K3/K4 姣忔寜涓€娆″鍑忛噺: 1 mm = 0.1 cm */

static void Target_Clamp(void)
{
    if (menu.target_mm < TARGET_MIN_MM) menu.target_mm = TARGET_MIN_MM;
    if (menu.target_mm > TARGET_MAX_MM) menu.target_mm = TARGET_MAX_MM;
}

static void Start_SelectedTask(void)
{
    const TaskDef *task = &task_table[menu.task_id];

    if (task->needs_sensor) {
        PB13_SetSensorMuxOutput();
    }
    if (task->init) {
        task->init();
    }
    menu.state = SYS_RUNNING;
    Key_FlushAll();
}

/* ======================================================================== *
 *  鐘舵€佽涓哄鐞?
 * ======================================================================== */

static void FSM_Menu(void)
{
    /* K1 鐭寜: task_id++ */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        if ((int)menu.task_id < TASK_ID_MAX) {
            menu.task_id = (TaskID)((int)menu.task_id + 1);
        }
    }

    /* K1 闀挎寜: 纭浠诲姟 鈫?READY */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 鐭寜: task_id-- */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        if ((int)menu.task_id > TASK_ID_MIN) {
            menu.task_id = (TaskID)((int)menu.task_id - 1);
        }
    }

    /* K2 闀挎寜: 鎭㈠榛樿鍙傛暟 */
    if (Key_ConsumeLong(KEY_IDX_K2)) {
        menu.task_id   = TASK_ID_DEFAULT;
        menu.target_mm = 0;
    }

    /* 涓婄數 2 绉掔獥鍙? K3/K4 璋冭妭 target_mm */
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
    /* K1 鐭寜: 鍚姩浠诲姟 鈫?RUNNING */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        Start_SelectedTask();
        return;
    }

    /* K2 鐭寜: 鍥?MENU */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }

    /* K1 闀挎寜: 娓呴浂缂栫爜鍣?+ 璁℃椂 + 鎽嗘潌闆剁偣 */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        /* TODO: 璋冪敤缂栫爜鍣ㄦ竻闆?API */
        /* TODO: 璋冪敤璁℃椂娓呴浂 API */
        /* TODO: 璋冪敤鎽嗘潌瑙掑害闆剁偣鏍″噯 API */
    }
}

static void FSM_Running(void)
{
    /* K1 闀挎寜: 鎬ュ仠 鈫?STOPPED */
    if (Key_ConsumeLong(KEY_IDX_K1)) {
        const TaskDef *task = &task_table[menu.task_id];
        if (task->stop) {
            task->stop();
        }
        /* 鎭㈠ PB13 涓?GPIO */
        PB13_SetGPIO();
        menu.state = SYS_STOPPED;
        Key_FlushAll();
        return;
    }

    /* K3 鐭寜: 浜哄伐涓 鈫?STOPPED */
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

    /* K2 涓嶅搷搴?(PB13 姝ゆ椂鐢ㄤ綔 AD2) */
}

static void FSM_Stopped(void)
{
    /* K1 鐭寜: 閲嶆柊杩涘叆 READY */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 鐭寜: 鍥?MENU */
    if (Key_ConsumeShort(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }
}

static void FSM_Fault(void)
{
    /* K1 鐭寜: 娓呮晠闅?鈫?READY */
    if (Key_ConsumeShort(KEY_IDX_K1)) {
        menu.state = SYS_READY;
        Key_FlushAll();
        return;
    }

    /* K2 闀挎寜: 鍥?MENU */
    if (Key_ConsumeLong(KEY_IDX_K2)) {
        menu.state = SYS_MENU;
        Key_FlushAll();
        return;
    }
}

/* ======================================================================== *
 *  KeyMenu_Scan 鈥?10ms 鍛ㄦ湡鍏ュ彛
 * ======================================================================== */

void KeyMenu_Scan(void)
{
    /* ---- 璇诲彇 4 閿數骞?---- */
    bool k1 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_12);  /* PB12 */
    bool k2 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_13);  /* PB13 */
    bool k3 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_2);   /* PB2  */
    bool k4 = KEY_PRESSED(GPIOB, DL_GPIO_PIN_3);   /* PB3  */

    /* ---- 閫愪釜鎵弿 (RUNNING 鎬佽烦杩?K2, 鍥犱负 PB13 鏄?AD2) ---- */
    Key_ScanOne(&menu.keys[KEY_IDX_K1], k1);
    if (menu.state != SYS_RUNNING) {
        Key_ScanOne(&menu.keys[KEY_IDX_K2], k2);
    }
    if (menu.state != SYS_RUNNING) {
        Key_ScanOne(&menu.keys[KEY_IDX_K3], k3);
        Key_ScanOne(&menu.keys[KEY_IDX_K4], k4);
    }

    /* ---- 涓婄數璁℃椂 ---- */
    if (menu.boot_ticks < 0xFFFFU) {
        menu.boot_ticks++;
    }
    if (menu.boot_ticks >= KEY_STARTUP_TICKS) {
        menu.startup_window = false;
    }

    if ((menu.state == SYS_RUNNING) &&
        (menu.task_id == TASK_T2) &&
        (t2_finish_ticks_10ms == 0U) &&
        (t2_elapsed_ticks_10ms < 0xFFFFFFFFU)) {
        t2_elapsed_ticks_10ms++;
    }

    /* ---- 鐘舵€佹満鍒嗗彂 ---- */
    switch (menu.state) {
        case SYS_MENU:    FSM_Menu();    break;
        case SYS_READY:   FSM_Ready();   break;
        case SYS_RUNNING: FSM_Running(); break;
        case SYS_STOPPED: FSM_Stopped(); break;
        case SYS_FAULT:   FSM_Fault();   break;
    }
}

/* ======================================================================== *
 *  KeyMenu_OLED 鈥?鍒锋柊 OLED 4 琛岃彍鍗曚俊鎭?
 * ======================================================================== */

void KeyMenu_OLED(void)
{
    char buf[22];
    uint32_t shown_ticks = (t2_finish_ticks_10ms != 0U) ?
        t2_finish_ticks_10ms : t2_elapsed_ticks_10ms;

    /* Line 1: 浠诲姟鍚?*/
    if (menu.task_id >= TASK_ID_MIN && menu.task_id <= TASK_ID_MAX) {
        snprintf(buf, sizeof(buf), "%-16d", (int)menu.task_id);
    } else {
        snprintf(buf, sizeof(buf), "?               ");
    }
    OLED_ShowLineString(1, 1, buf);

    /* Line 2: 鐩爣鐐?*/
    if (menu.task_id == TASK_T2) {
        snprintf(buf, sizeof(buf), "t:%2lu.%02lus d:%3d",
                 (unsigned long)(shown_ticks / 100U),
                 (unsigned long)(shown_ticks % 100U),
                 (int)g_Encoder.distance_cm);
    } else {
        snprintf(buf, sizeof(buf), "target: %4d mm", menu.target_mm);
    }
    OLED_ShowLineString(2, 1, buf);

    /* Line 3: 绯荤粺鐘舵€?*/
    OLED_ShowLineString(3, 1, state_names[menu.state]);
}

/* ======================================================================== *
 *  鍏叡璁块棶鍣?
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
