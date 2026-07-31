/*
 * balance_control.h — 车载平衡滚球运动控制系统
 *                    串级 PID + 运动学解耦 + 加速度前馈
 *
 * 平台: MSPM0G3507 (TI Clang / TI DriverLib)
 * 周期: 10ms (100 Hz)
 *
 * ===== 物理模型 =====
 *   水管长 L = 25 cm, 左端铰链固定 (-12.5 cm), 右端顶升 (+12.5 cm)
 *   钢球坐标 x ∈ [-12.0, 12.0] cm, 中心为 0
 *   管倾角 θ > 0 表示右端高于左端
 *
 * ===== 核心架构 =====
 *   ┌──────────┐   a_base   ┌──────────────┐   a_des   ┌──────────┐   θ_raw   ┌───────┐   θ_tgt   ┌──────────────┐
 *   │ Position │───(m/s²)──→│  Velocity     │──(m/s²)──→│  Accel→  │──(rad)──→│ Feed  │──(rad)──→│  Inverse     │──→ PWM
 *   │ PD       │            │  Inner Loop   │           │  Angle   │          │forward│          │  Kinematics  │   [500,2500]
 *   │ (outer)  │  + Kp·v    │  P: Kp_vel    │           │  -1/g    │          │ +θ_ff │          │  θ→h→PWM     │
 *   └──────────┘            └──────────────┘           └──────────┘          └───────┘          └──────────────┘
 *        ↑ x, v                   ↑ v (反馈)                       ↑ a_car (IMU)
 *        └── State Estimator (LPF + 差分)
 *               ↑ x_raw (视觉串口)
 *
 * ===== 控制律详解 =====
 *   位置 PD (外环):     a_base = Kp_pos · (x_ref - x) - Kd_pos · v
 *   速度内环 (P):       a_des  = a_base - Kp_vel · v          [额外速度阻尼]
 *   逆动力学:            θ_raw = -a_des / g                   [加速度→倾角]
 *   前馈:                θ_ff  = -arcsin(a_car / g) ≈ -a_car / g
 *   综合:                θ_target = θ_raw + θ_ff
 *   运动学逆解:          PWM = PWM_center + sin(θ) · rad_to_pwm_scale
 *
 *   Kd_pos 与 Kp_vel 都提供速度阻尼, 但作用不同:
 *     - Kd_pos: 对位置误差变化率的响应 (位置超调抑制)
 *     - Kp_vel: 对绝对速度的纯阻尼 (震荡抑制)
 *     调参建议: 先调 Kp_pos + Kp_vel 让系统稳定, 再微调 Kd_pos 改善超调.
 *
 * ===== 步进电机 PD42S1 =====
 *   模式: 脉宽位置模式 (需先发 F5H 指令使能)
 *   PWM: 50 Hz (20 ms), 500~2500 µs 脉宽
 *   映射: 500 µs → 0 micro-step, 2500 µs → 25600 micro-step
 *
 * ===== 使用说明 =====
 *   1. BalanceControl_Init()       — 启动时调用一次
 *   2. BalanceControl_PD42S1_Init() — 配置步进电机为 PWM 位置模式
 *   3. 视觉 UART 回调中 → BalanceControl_SetRawPosition()
 *   4. IMU 更新时 → BalanceControl_SetCarAccel()
 *   5. 10 ms 定时器 → BalanceControl_Run()
 *   6. 读取 bc->pwm_pulse → 写入硬件 TIMER 比较寄存器
 */

#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== *
 *  物理与系统常量
 * ======================================================================== */

#define BC_GRAVITY                  (9.81f)         /* 重力加速度      m/s²  */
#define BC_PIPE_LENGTH_CM           (25.0f)         /* 水管全长         cm   */
#define BC_PIPE_HALF_CM             (12.5f)         /* 水管半长         cm   */
#define BC_POS_MIN_CM               (-12.0f)        /* 钢球坐标下限     cm   */
#define BC_POS_MAX_CM               (12.0f)         /* 钢球坐标上限     cm   */
#define BC_DT_S                     (0.010f)        /* 控制周期 10 ms   s    */

/* PD42S1 步进电机 PWM 约束 */
#define BC_PWM_MIN_US               (500U)          /* 最小脉宽        µs   */
#define BC_PWM_MAX_US               (2500U)         /* 最大脉宽        µs   */
#define BC_PWM_CENTER_US            (1500U)         /* 中位 (θ=0)     µs   */
#define BC_PWM_RANGE_US             (1000U)         /* 单侧范围        µs   */
#define BC_PWM_PERIOD_US            (20000U)        /* 50 Hz 周期      µs   */
#define BC_PWM_NEGATIVE_DELTA_LIMIT_US (130U)
#define BC_PWM_POSITIVE_DELTA_LIMIT_US (250U)
#define BC_PWM_MIN_DRIVE_US         (130U)
#define BC_PWM_SLEW_LIMIT_US         (15U)
#define BC_PWM_REVERSE_SLEW_LIMIT_US (80U)
#define BC_PWM_DIRECTION_SIGN       (1.0f)
#define BC_RAD_TO_PWM_SCALE_DEFAULT (10000.0f)
#define BC_MIN_DRIVE_ERROR_CM       (1.5f)
#define BC_FULL_DRIVE_ERROR_CM      (6.0f)

/* 安全限幅 */
#define BC_ANGLE_MAX_RAD            (0.09f)         /* 初调最大倾角 ≈ 5.2° */
#define BC_ACCEL_MAX_MS2            (0.7f)          /* 初调最大期望加速度 m/s² */

/* 默认 PID 参数 (需现场整定) */
#define BC_DEFAULT_POS_KP           (0.12f)         /* 位置外环 比例增益    */
#define BC_DEFAULT_POS_KD           (0.018f)        /* 位置外环 微分增益    */
#define BC_DEFAULT_VEL_KP           (0.006f)        /* 速度内环 比例增益    */
#define BC_POSITION_LOOKAHEAD_S      (0.20f)

/* 低通滤波器系数 (100 Hz 采样) */
#define BC_LPF_ALPHA_POS            (0.20f)         /* 位置 LPF 系数        */
#define BC_LPF_ALPHA_VEL            (0.15f)         /* 速度 LPF 系数        */

/* PD42S1 UART 指令帧定义 */
#define BC_PD42S1_HEADER            (0xC5U)
#define BC_PD42S1_ADDR             (0x01U)
#define BC_PD42S1_CMD_PWM_MODE     (0xF5U)
#define BC_PD42S1_TAIL             (0x5CU)

/* ======================================================================== *
 *  控制状态结构体 (集中管理, 拒绝全局变量泛滥)
 * ======================================================================== */
typedef struct {

    /* ---------- 原始输入 (由外部 UART/IMU 回调写入, Run 中消费) ---------- */
    float        x_raw;             /* 视觉原始坐标    cm                  */
    bool         x_raw_updated;     /* 新数据标志, Run 消费后清零          */
    float        x_sample_dt_s;     /* 当前视觉样本与上一样本的间隔 s      */
    bool         has_position_sample;

    /* ---------- 状态估计 ---------- */
    float        x_pos;             /* 滤波后球位置    cm                  */
    float        x_vel;             /* 估计球速度      cm/s                */
    float        x_prev;            /* 上一拍滤波位置 (用于差分求速)       */
    float        lpf_pos_state;     /* 位置一阶 LPF 内部状态               */
    float        lpf_vel_state;     /* 速度一阶 LPF 内部状态               */

    /* ---------- 参考值与扰动 ---------- */
    float        x_ref;             /* 目标位置        cm                  */
    float        a_car;             /* 小车纵向加速度  m/s²  (向前为正)    */

    /* ---------- 位置外环 PD ---------- */
    float        pos_kp;            /* 比例增益                            */
    float        pos_kd;            /* 微分增益 (作用于速度, 抑制超调)     */
    float        a_base;            /* PD 输出的基础期望加速度  m/s²       */
    float        pos_out_max;       /* 加速度输出限幅  m/s²                */

    /* ---------- 速度内环 P (额外速度阻尼) ---------- */
    float        vel_kp;            /* 速度阻尼增益                        */
    float        a_des;             /* 经内环修正后的最终期望加速度 m/s²   */

    /* ---------- 倾角计算 ---------- */
    float        theta_raw;         /* 由 a_des 映射的原始倾角  rad        */
    float        theta_ff;          /* 加速度前馈补偿角         rad        */
    float        theta_target;      /* 最终目标倾角 (含限幅)    rad        */

    /* ---------- 运动学逆解 ---------- */
    float        rad_to_pwm_scale;  /* rad → PWM 脉宽缩放因子              */
    float        pwm_neutral;       /* PWM 中位值 (float 域, 防截断误差)   */
    uint16_t     pwm_negative_delta_limit_us;
    uint16_t     pwm_positive_delta_limit_us;
    uint16_t     pwm_min_drive_us;
    bool         pwm_override_enabled;
    uint16_t     pwm_override_pulse_us;

    /* ---------- 最终输出 ---------- */
    uint16_t     pwm_pulse;         /* PWM 脉宽 [500, 2500] µs            */

    /* ---------- 运行控制 ---------- */
    bool         enabled;           /* 控制使能 (禁用时输出中位)           */

} BalanceControl_t;

/* 检查结构体大小, 确保栈/内存分配合理 */
#define BC_CONTROL_SIZEOF   sizeof(BalanceControl_t)   /* 约 30 × 4 + 7 ≈ 127 字节 */

/* ======================================================================== *
 *  公共 API
 * ======================================================================== */

/**
 * @brief  初始化控制器
 * @param  bc  控制器实例 (非空)
 */
void BalanceControl_Init(BalanceControl_t *bc);

/**
 * @brief  配置 PD42S1 进入 PWM 脉宽位置模式
 *
 *         发送指令: C5 01 F5 01 <CHKSUM> 5C
 *         使能后 STP 引脚接收 50Hz PWM, 脉宽 500~2500 µs 映射到微步位置.
 *
 * @param   uart_write  用户提供的串口写函数, 接收 (data, len) 参数
 *
 * @note   调用此函数前需保证 UART 已初始化且 PD42S1 供电正常.
 *         如需自定义配置(如修改高/低电平映射), 可参考手册 F5H 指令格式.
 */
void BalanceControl_PD42S1_Init(void (*uart_write)(const uint8_t *data, uint32_t len));

/**
 * @brief  注入视觉原始坐标 (由 UART 中断/轮询回调调用)
 * @param  bc        控制器实例
 * @param  x_raw_cm 视觉测得的钢球坐标 (cm), 自动限幅到 [-12.0, 12.0]
 *
 * @note   线程安全: 单写一个 float + bool, 无临界区需求
 */
void BalanceControl_SetRawPosition(BalanceControl_t *bc, float x_raw_cm);
void BalanceControl_SetRawPositionTimed(BalanceControl_t *bc, float x_raw_cm,
                                        float sample_dt_s);

/**
 * @brief  设置目标位置
 * @param  bc        控制器实例
 * @param  x_ref_cm  目标坐标 (cm), 自动限幅至 [-12.0, 12.0]
 */
void BalanceControl_SetReference(BalanceControl_t *bc, float x_ref_cm);

/**
 * @brief  设置小车加速度 (由 IMU 数据更新)
 * @param  bc         控制器实例
 * @param  a_car_ms2  小车纵向加速度 (m/s²), 向前为正
 *
 * @note   用于前馈补偿: 小车加速 → 前馈倾角抵消惯性
 */
void BalanceControl_SetCarAccel(BalanceControl_t *bc, float a_car_ms2);

/**
 * @brief  设置位置外环 PD 参数
 */
void BalanceControl_SetPositionPD(BalanceControl_t *bc, float kp, float kd);

/**
 * @brief  设置速度内环 P 参数
 */
void BalanceControl_SetVelocityP(BalanceControl_t *bc, float kp);

/**
 * @brief  标定 PWM 中位 (水管水平时调用, 记录当前 PWM 为零位)
 * @param  bc  控制器实例
 * @param  pwm_current  水管水平时的 PWM 脉宽 (通常 1500)
 */
void BalanceControl_CalibrateCenter(BalanceControl_t *bc, uint16_t pwm_current);

/**
 * @brief  设置 rad → PWM 转换比例
 * @param  bc    控制器实例
 * @param  scale  每弧度对应的 PWM 脉宽变化量
 *
 * @note   标定方法: 倾斜已知角度 θ₀ (可用水平仪+角度尺),
 *         记录稳定后的 PWM 值 P₀, 则 scale = |P₀ - 1500| / |θ₀|.
 *         默认 ~6667 (当满行程 ±1000PWM 对应 ±0.15rad).
 */
void BalanceControl_SetPwmScale(BalanceControl_t *bc, float scale);

void BalanceControl_SetOutputProfile(BalanceControl_t *bc,
                                     uint16_t negative_limit_us,
                                     uint16_t positive_limit_us,
                                     uint16_t minimum_drive_us);
void BalanceControl_SetPwmOverride(BalanceControl_t *bc, bool enabled,
                                   uint16_t pulse_us);

/**
 * @brief  使能/禁能控制器输出
 * @param  bc  控制器实例
 * @param  en  true=使能 (复位内部状态后启动), false=禁能 (输出安全中位)
 */
void BalanceControl_Enable(BalanceControl_t *bc, bool en);

/**
 * @brief  复位所有内部状态 (滤波器, 历史值, 加速度), 使能状态保持不变
 */
void BalanceControl_Reset(BalanceControl_t *bc);

/**
 * @brief  10 ms 周期主控制循环 —— 核心算法入口
 *
 *         执行管线:
 *           step 1) 状态估计
 *                    LPF(x_raw) → x_pos
 *                    后向差分 + LPF → x_vel
 *           step 2) 位置外环 PD
 *                    a_base = Kp_pos·(x_ref - x_pos) - Kd_pos·x_vel
 *           step 3) 速度内环 (P)
 *                    a_des = a_base - Kp_vel·x_vel
 *           step 4) 逆动力学 (加速度 → 倾角)
 *                    θ_raw = -a_des / g
 *           step 5) 加速度前馈
 *                    θ_ff = -arcsin(a_car / g) ≈ -a_car/g
 *           step 6) 综合目标倾角 + 限幅
 *                    θ_target = clamp(θ_raw + θ_ff, ±θ_max)
 *           step 7) 运动学逆解 (θ → PWM)
 *                    pwm = neutral + sin(θ)·scale
 *           step 8) PWM 硬限幅 [500, 2500]
 *
 * @param  bc  控制器实例
 */
void BalanceControl_Run(BalanceControl_t *bc);

/**
 * @brief  获取当前状态幅度 (用于调试/自适应调参)
 * @return  |x_ref - x| + |v·T|  位置误差与速度的综合范数 (近似)
 *
 * @note   使用 L1 范数替代 L2 以避免 sqrt() 软件仿真开销.
 *         在量级上等效于 sqrt(e² + (vT)²), 仅用于定性评估.
 */
float BalanceControl_GetStateMagnitude(const BalanceControl_t *bc);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CONTROL_H */
