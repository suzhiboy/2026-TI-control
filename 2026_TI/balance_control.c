/*
 * balance_control.c — 串级 PID + 运动学解耦 + 加速度前馈 核心实现
 *
 * ===== 物理模型与符号约定 =====
 *   x      钢球在管内的位置 (cm), 0 为中心, + 向右
 *   θ      水管倾角 (rad),  >0 表示右端高于左端
 *   a_car  小车纵向加速度 (m/s²), >0 表示向前加速
 *
 *   小球沿管动力学 (小角度近似, |θ| < 0.15 rad):
 *     x_ddot = -g·θ - a_car
 *
 * ===== 控制律 =====
 *   位置 PD (外环):     a_base = Kp_pos·(x_ref - x) - Kd_pos·v
 *   速度内环 (P):       a_des  = a_base - Kp_vel·v
 *   逆动力学:            θ_raw = -a_des / g
 *   前馈:                θ_ff  = -arcsin(a_car / g) ≈ -a_car / g
 *   综合:                θ_target = clamp(θ_raw + θ_ff, ±θ_max)
 *   逆解:                PWM = neutral + sin(θ_target) · scale
 *
 * ===== 滤波器 (100 Hz) =====
 *   位置: 一阶 IIR, α=0.20 → ~3.2 Hz 截止 (滤除视觉抖动)
 *   速度: 后向差分 + 一阶 IIR, α=0.15 → ~2.4 Hz 截止
 *
 * ===== PWM 映射 =====
 *   PD42S1 脉宽位置模式 (F5H 指令使能):
 *     50Hz (20ms) PWM, 500µs=0步, 2500µs=25600步, 线性插值
 *     机械: 丝杠导程 × 微步数 = 顶升高度
 *     scale = ΔPWM/Δθ 由现场标定 (默认 6667 rad⁻¹)
 *
 * 编译器: TI Clang  (C99/C11)
 * 平台:   MSPM0G3507 (Cortex-M0+, 无 FPU, 全部 soft-float)
 * 优化:   无 math.h 调用, 所有超越函数 (sin, arcsin, sqrt) 均使用
 *         多项式近似替代, 仅在定时器中断上下文中执行几十条整数/浮点指令.
 */

#include <stddef.h>      /* NULL */
#include "balance_control.h"

/* ======================================================================== *
 *  TI Clang 兼容性: 某些版本不支持 __builtin_expect, 用宏包装
 * ======================================================================== */
#ifndef __TI_COMPILER_VERSION
  #define bc_likely(x)   __builtin_expect((x), 1)
  #define bc_unlikely(x) __builtin_expect((x), 0)
#else
  /* TI Clang 可能不支持 __builtin_expect, 退化为空 */
  #define bc_likely(x)   (x)
  #define bc_unlikely(x) (x)
#endif

/* ======================================================================== *
 *  内部工具: 浮点限幅 (纯 float 运算, 无 math.h 调用)
 * ======================================================================== */

static inline float fclamp(float val, float lo, float hi)
{
    return (val < lo) ? lo : ((val > hi) ? hi : val);
}

static inline uint16_t u16clamp(uint16_t val, uint16_t lo, uint16_t hi)
{
    return (val < lo) ? lo : ((val > hi) ? hi : val);
}

static inline float slew_limit_pwm(float target, float previous, float neutral)
{
    float diff = target - previous;
    float target_delta = target - neutral;
    float previous_delta = previous - neutral;
    float limit = (float)BC_PWM_SLEW_LIMIT_US;

    if ((target_delta * previous_delta) < 0.0f) {
        limit = (float)BC_PWM_REVERSE_SLEW_LIMIT_US;
    }

    if (diff > limit) {
        return previous + limit;
    }
    if (diff < -limit) {
        return previous - limit;
    }
    return target;
}

/* ======================================================================== *
 *  内部工具: 浮点绝对值 (union 避免 strict-aliasing 问题)
 * ======================================================================== */
static inline float fabsf_(float x)
{
    union { float f; uint32_t u; } fu = { .f = x };
    fu.u &= 0x7FFFFFFFU;   /* 清符号位 */
    return fu.f;
}

/* ======================================================================== *
 *  内部工具: 小角度 sin 近似 (避免 sinf 的软件仿真开销)
 * ======================================================================== */

/**
 * @brief  泰勒展开 sin(x) ≈ x - x³/6 + x⁵/120
 *
 *         在 |x| < 0.2 rad (~11.5°) 时相对误差 < 1e-6
 *         仅使用乘法和加法, 无 math.h 调用, 约 8 个周期.
 */
static inline float fast_sin_small(float x)
{
    float x2 = x * x;
    /* sin(x) = x(1 - x²/6 + x⁴/120) */
    return x * (1.0f - x2 * 0.1666667f + x2 * x2 * 0.008333333f);
}

/* ======================================================================== *
 *  内部工具: 小角度 arcsin 近似 (避免 asinf 开销)
 * ======================================================================== */

/**
 * @brief  泰勒展开 arcsin(x) ≈ x(1 + x²/6 + 3x⁴/40)
 *
 *         |x| < 0.7 时误差 < 0.3%; |x| < 0.85 时 < 1%
 *         纯乘加, 约 6 个周期.
 */
static inline float fast_asin_small(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * 0.1666667f + x2 * x2 * 0.075f);
}

/* ======================================================================== *
 *  内部工具: 一阶 IIR 低通
 * ======================================================================== */

static inline float lpf_update(float input, float *state, float alpha)
{
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

/* ======================================================================== *
 *  API 实现
 * ======================================================================== */

/* ------------------------------------------------------------------ */
/*  初始化                                                             */
/* ------------------------------------------------------------------ */

void BalanceControl_Init(BalanceControl_t *bc)
{
    if (bc_unlikely(bc == NULL)) return;

    /* --- 原始输入 --- */
    bc->x_raw          = 0.0f;
    bc->x_raw_updated  = false;
    bc->x_sample_dt_s  = BC_DT_S;
    bc->has_position_sample = false;

    /* --- 状态估计 --- */
    bc->x_pos          = 0.0f;
    bc->x_vel          = 0.0f;
    bc->x_prev         = 0.0f;
    bc->lpf_pos_state  = 0.0f;
    bc->lpf_vel_state  = 0.0f;

    /* --- 参考与扰动 --- */
    bc->x_ref          = 0.0f;
    bc->a_car          = 0.0f;

    /* --- 位置 PD --- */
    bc->pos_kp         = BC_DEFAULT_POS_KP;
    bc->pos_kd         = BC_DEFAULT_POS_KD;
    bc->a_base         = 0.0f;
    bc->pos_out_max    = BC_ACCEL_MAX_MS2;

    /* --- 速度内环 --- */
    bc->vel_kp         = BC_DEFAULT_VEL_KP;
    bc->a_des          = 0.0f;

    /* --- 倾角 --- */
    bc->theta_raw      = 0.0f;
    bc->theta_ff       = 0.0f;
    bc->theta_target   = 0.0f;

    /*
     * --- 运动学逆解 ---
     *
     * 默认标定 (需现场修正):
     *   满行程 ±1000 PWM 对应 ±0.15 rad (≈±8.6°)
     *   scale = 1000 / 0.15 ≈ 6667
     *
     * 标定步骤:
     *   1) 使水管水平, 记录 PWM_CENTER (调用 CalibrateCenter)
     *   2) 发送一个已知角度 θ₀ (如 0.05 rad ≈ 2.86°)
     *   3) 读取稳定后的 PWM 值 P₀
     *   4) scale = |P₀ - PWM_CENTER| / |θ₀|
     */
    bc->rad_to_pwm_scale = BC_RAD_TO_PWM_SCALE_DEFAULT;
    bc->pwm_neutral       = (float)BC_PWM_CENTER_US;
    bc->pwm_negative_delta_limit_us = BC_PWM_NEGATIVE_DELTA_LIMIT_US;
    bc->pwm_positive_delta_limit_us = BC_PWM_POSITIVE_DELTA_LIMIT_US;
    bc->pwm_min_drive_us = BC_PWM_MIN_DRIVE_US;
    bc->pwm_override_enabled = false;
    bc->pwm_override_pulse_us = BC_PWM_CENTER_US;

    /* --- 输出 --- */
    bc->pwm_pulse      = BC_PWM_CENTER_US;

    /* --- 运行控制 --- */
    bc->enabled        = false;
}

/* ------------------------------------------------------------------ */
/*  PD42S1 步进电机初始化: 配置 PWM 脉宽位置模式                       */
/* ------------------------------------------------------------------ */

void BalanceControl_PD42S1_Init(void (*uart_write)(const uint8_t *data, uint32_t len))
{
    /*
     * 指令帧: C5 01 F5 01 <CHKSUM> 5C
     *   C5       — 帧头
     *   01       — 驱动器地址 (默认 0x01)
     *   F5       — PWM 位置模式命令
     *   01       — 模式: 1 = 开启
     *   CHKSUM   — 校验和 (累加 C5..01 的前一字节, 取低 8 位)
     *   5C       — 帧尾
     *
     * 如需自定义 500/2500µs → 微步映射, 使用完整的 12 字节写入:
     *   C5 01 F5 <HT_LO> <HT_HI> <LT_LO> <LT_HI>
     *          <POS_HI_LO> <POS_HI_HI> <POS_HI_EX> <POS_HI_EX2>
     *          <POS_LO_LO> <POS_LO_HI> <POS_LO_EX> <POS_LO_EX2>
     *          <CHKSUM> 5C
     *   其中 HT = 高电平时长 (µs, uint16 LE), LT = 低电平时长 (µs, uint16 LE)
     *   POS_HI = 高电平对应微步位置 (int32 LE), POS_LO = 低电平对应位置
     */
    if (bc_unlikely(uart_write == NULL)) return;

    uint8_t cmd[] = {
        BC_PD42S1_HEADER,           /* 帧头       */
        BC_PD42S1_ADDR,             /* 驱动器地址  */
        BC_PD42S1_CMD_PWM_MODE,     /* PWM 位置模式命令 */
        0x01,                       /* 参数: 1 = 使能  */
        0x00,                       /* 校验和占位 */
        BC_PD42S1_TAIL              /* 帧尾       */
    };

    /* 计算校验和: 累加 C5..01 (不含帧尾) */
    uint8_t sum = 0;
    for (uint32_t i = 0; i < sizeof(cmd) - 2; i++) {
        sum += cmd[i];
    }
    cmd[4] = sum;   /* 填入校验和 */

    uart_write(cmd, sizeof(cmd));
}

/* ------------------------------------------------------------------ */
/*  传感器/参数注入                                                    */
/* ------------------------------------------------------------------ */

void BalanceControl_SetRawPosition(BalanceControl_t *bc, float x_raw_cm)
{
    BalanceControl_SetRawPositionTimed(bc, x_raw_cm, BC_DT_S);
}

void BalanceControl_SetRawPositionTimed(BalanceControl_t *bc, float x_raw_cm,
                                        float sample_dt_s)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->x_raw         = fclamp(x_raw_cm, BC_POS_MIN_CM, BC_POS_MAX_CM);
    if ((sample_dt_s >= 0.005f) && (sample_dt_s <= 0.2f)) {
        bc->x_sample_dt_s = sample_dt_s;
    } else {
        bc->x_sample_dt_s = BC_DT_S;
    }
    bc->x_raw_updated = true;
}

void BalanceControl_SetReference(BalanceControl_t *bc, float x_ref_cm)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->x_ref = fclamp(x_ref_cm, BC_POS_MIN_CM, BC_POS_MAX_CM);
}

void BalanceControl_SetCarAccel(BalanceControl_t *bc, float a_car_ms2)
{
    if (bc_unlikely(bc == NULL)) return;

    /* 限幅防 IMU 野值, 超过 ±9 m/s² (~1g) 的读数视为异常 */
    bc->a_car = fclamp(a_car_ms2, -9.0f, 9.0f);
}

void BalanceControl_SetPositionPD(BalanceControl_t *bc, float kp, float kd)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->pos_kp = (kp >= 0.0f) ? kp : 0.0f;
    bc->pos_kd = (kd >= 0.0f) ? kd : 0.0f;
}

void BalanceControl_SetVelocityP(BalanceControl_t *bc, float kp)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->vel_kp = (kp >= 0.0f) ? kp : 0.0f;
}

/* ------------------------------------------------------------------ */
/*  标定                                                              */
/* ------------------------------------------------------------------ */

void BalanceControl_CalibrateCenter(BalanceControl_t *bc, uint16_t pwm_current)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->pwm_neutral = (float)u16clamp(pwm_current, BC_PWM_MIN_US, BC_PWM_MAX_US);
}

void BalanceControl_SetPwmScale(BalanceControl_t *bc, float scale)
{
    if (bc_unlikely(bc == NULL)) return;

    /* 有效 scale 至少 100, 防止被意外 0 值除 */
    bc->rad_to_pwm_scale = (scale > 100.0f) ? scale : BC_RAD_TO_PWM_SCALE_DEFAULT;
}

void BalanceControl_SetOutputProfile(BalanceControl_t *bc,
                                     uint16_t negative_limit_us,
                                     uint16_t positive_limit_us,
                                     uint16_t minimum_drive_us)
{
    uint16_t smaller_limit;

    if (bc_unlikely(bc == NULL)) return;

    bc->pwm_negative_delta_limit_us =
        u16clamp(negative_limit_us, 0U, BC_PWM_RANGE_US);
    bc->pwm_positive_delta_limit_us =
        u16clamp(positive_limit_us, 0U, BC_PWM_RANGE_US);
    smaller_limit = (bc->pwm_negative_delta_limit_us <
                     bc->pwm_positive_delta_limit_us) ?
        bc->pwm_negative_delta_limit_us :
        bc->pwm_positive_delta_limit_us;
    bc->pwm_min_drive_us =
        u16clamp(minimum_drive_us, 0U, smaller_limit);
}

void BalanceControl_SetPwmOverride(BalanceControl_t *bc, bool enabled,
                                   uint16_t pulse_us)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->pwm_override_enabled = enabled;
    bc->pwm_override_pulse_us =
        u16clamp(pulse_us, BC_PWM_MIN_US, BC_PWM_MAX_US);
}

/* ------------------------------------------------------------------ */
/*  使能/复位                                                         */
/* ------------------------------------------------------------------ */

void BalanceControl_Enable(BalanceControl_t *bc, bool en)
{
    if (bc_unlikely(bc == NULL)) return;

    if (en && !bc->enabled) {
        /* 从禁用到使能: 清空内部状态, 防止积分/滤波器冲激 */
        BalanceControl_Reset(bc);
    }

    bc->enabled = en;

    if (!en) {
        /* 禁用时输出安全中位, 电机回到水平位置 */
        bc->pwm_pulse = (uint16_t)(bc->pwm_neutral + 0.5f);
    }
}

void BalanceControl_Reset(BalanceControl_t *bc)
{
    if (bc_unlikely(bc == NULL)) return;

    bc->x_pos         = bc->lpf_pos_state;   /* 保留滤波状态避免跳变 */
    bc->x_vel         = 0.0f;
    bc->x_prev        = bc->x_pos;
    bc->lpf_vel_state = 0.0f;
    bc->has_position_sample = false;
    bc->x_raw_updated = false;
    bc->pwm_override_enabled = false;
    bc->pwm_override_pulse_us = (uint16_t)(bc->pwm_neutral + 0.5f);
    bc->a_base        = 0.0f;
    bc->a_des         = 0.0f;
    bc->theta_raw     = 0.0f;
    bc->theta_ff      = 0.0f;
    bc->theta_target  = 0.0f;
    bc->pwm_pulse     = (uint16_t)(bc->pwm_neutral + 0.5f);
}

/* ======================================================================== *
 *  核心控制循环 (每 10 ms 调用一次)
 *
 *  内部数据流 (每一步的输入与产出):
 *     [x_raw] → Step1 → [x_pos, x_vel]
 *     [x_ref, x_pos, x_vel] → Step2 → [a_base]
 *     [a_base, x_vel] → Step3 → [a_des]
 *     [a_des] → Step4 → [theta_raw]
 *     [a_car] → Step5 → [theta_ff]
 *     [theta_raw, theta_ff] → Step6 → [theta_target]
 *     [theta_target] → Step7 → [pwm_pulse]
 * ======================================================================== */

void BalanceControl_Run(BalanceControl_t *bc)
{
    float x_pos_filt;       /* 本周期滤波位置              */
    float raw_vel;          /* 原始差分速度                */

    if (bc_unlikely(bc == NULL)) return;

    if (bc->pwm_override_enabled) {
        bc->pwm_pulse = bc->pwm_override_pulse_us;
        return;
    }

    /* ---------------------------------------------------------------- *
     *  Step 1 — 状态估计
     *
     *  位置 LPF:
     *     y[k] = α · u[k] + (1-α) · y[k-1]
     *     仅在有新视觉数据时更新滤波器输入; 否则维持当前状态.
     *
     *  速度估计:
     *     v_raw = (x_filt[k] - x_filt[k-1]) / Δt
     *     v[k]  = α_v · v_raw + (1-α_v) · v[k-1]
     *
     *  速度 LPF 至关重要: 差分运算放大了视觉噪声, 不滤波会导致 PD
     *  输出剧烈抖动.  α_v=0.15 对应 ~2.4 Hz 截止, 保留系统带宽.
     * ---------------------------------------------------------------- */
    if (bc->x_raw_updated) {
        if (!bc->has_position_sample) {
            bc->lpf_pos_state = bc->x_raw;
            bc->x_prev = bc->x_raw;
            bc->x_vel = 0.0f;
            bc->lpf_vel_state = 0.0f;
            bc->has_position_sample = true;
            x_pos_filt = bc->x_raw;
        } else {
            x_pos_filt = lpf_update(bc->x_raw, &bc->lpf_pos_state,
                                    BC_LPF_ALPHA_POS);
            raw_vel = (x_pos_filt - bc->x_prev) / bc->x_sample_dt_s;
            bc->x_vel = lpf_update(raw_vel, &bc->lpf_vel_state,
                                   BC_LPF_ALPHA_VEL);
            bc->x_prev = x_pos_filt;
        }
        bc->x_raw_updated = false;
    } else {
        x_pos_filt = bc->lpf_pos_state;
    }

    bc->x_pos = x_pos_filt;

    /* ---------------------------------------------------------------- *
     *  Step 2 — 位置外环: PD 控制 → a_base
     *
     *  注意: 微分项直接作用于 -v (而非位置误差的差分), 优点:
     *    - 参考跳变 (x_ref 阶跃) 时不产生微分爆冲
     *    - 量化噪声不被二次放大
     *  这等价于 ITAE 标准型中的"微分先行"结构.
     *
     *  公式: a_base = Kp · (x_ref - x_pos) - Kd · x_vel
     * ---------------------------------------------------------------- */
    {
        float predicted_pos = bc->x_pos +
                              bc->x_vel * BC_POSITION_LOOKAHEAD_S;
        predicted_pos = fclamp(predicted_pos,
                               BC_POS_MIN_CM, BC_POS_MAX_CM);
        bc->a_base = bc->pos_kp * (bc->x_ref - predicted_pos)
                   - bc->pos_kd * bc->x_vel;
    }

    /* 加速度限幅: 防止物理不可实现的请求 */
    bc->a_base = fclamp(bc->a_base, -bc->pos_out_max, bc->pos_out_max);

    /* ---------------------------------------------------------------- *
     *  Step 3 — 速度内环: P 控制
     *
     *  在经过位置 PD 输出的 a_base 基础上, 叠加额外速度阻尼:
     *     a_des = a_base - Kp_vel · v
     *
     *  之所以不用 PI 结构, 是因为:
     *    - 积分项在位置外环已通过 Kp 隐含存在 (位置误差累积)
     *    - 速度环 I 会造成响应迟滞和 windup
     *    - 纯 P 阻尼已足够抑制震荡
     *
     *  与 Kd_pos 的协同:
     *    - Kd_pos: 快速抑制位置误差变化率 (减小超调)
     *    - Kp_vel: 直接给速度"刹车" (抑制持续震荡)
     *    - 调参时: 先定 Kp_pos 保证响应, Kp_vel 消除震荡, Kd_pos 微调
     * ---------------------------------------------------------------- */
    bc->a_des = bc->a_base - bc->vel_kp * bc->x_vel;

    /* 二次限幅 (内环可能使信号放大) */
    bc->a_des = fclamp(bc->a_des, -bc->pos_out_max, bc->pos_out_max);

    /* ---------------------------------------------------------------- *
     *  Step 4 — 逆动力学: 期望加速度 → 倾角
     *
     *  物理关系: x_ddot = -g · θ - a_car
     *  令 x_ddot = a_des:  a_des = -g · θ_raw - a_car
     *  得: θ_raw = -(a_des + a_car) / g
     *
     *  此处先计算不含前馈的 θ_raw (只含 a_des 项):
     *     θ_raw = -a_des / g
     *  前馈项 θ_ff 在下一步独立计算, 更直观.
     * ---------------------------------------------------------------- */
    bc->theta_raw = -bc->a_des / BC_GRAVITY;

    /* ---------------------------------------------------------------- *
     *  Step 5 — 加速度前馈
     *
     *  小车加速时, 球因惯性产生相对运动. 前馈补偿:
     *     θ_ff = -arcsin(a_car / g)
     *
     *  小角度线性化: arcsin(a_car/g) ≈ a_car/g, 在 a_car < 5 m/s²
     *  时误差 < 4%. 此处使用精确近似 fast_asin_small().
     *
     *  符号直觉: 小车突然向前 (+a_car), 球相对向后滚.
     *  为抵消, 右端需向下 (-θ_ff), 使球受重力向前滚回.
     *  θ_ff 为负 → 右端下降 → 正确.
     * ---------------------------------------------------------------- */
    {
        /* |a_car/g| 限幅到 0.85 避免定义域溢出 (arcsin 定义域 [-1,1]) */
        float ratio = fclamp(bc->a_car / BC_GRAVITY, -0.85f, 0.85f);
        bc->theta_ff = -fast_asin_small(ratio);
    }

    /* ---------------------------------------------------------------- *
     *  Step 6 — 综合目标倾角 + 软件限幅
     *
     *  θ_target = θ_raw + θ_ff
     *           = -a_des/g - arcsin(a_car/g)
     *
     *  限幅保护: |θ| > θ_max 会损坏机械 (拉杆撞扯或丝杠卡死)
     * ---------------------------------------------------------------- */
    bc->theta_target = fclamp(bc->theta_raw + bc->theta_ff,
                              -BC_ANGLE_MAX_RAD, BC_ANGLE_MAX_RAD);

    /* ---------------------------------------------------------------- *
     *  Step 7 — 运动学逆解: θ_target → PWM 脉宽
     *
     *  物理链路:
     *    θ [rad] → h = L · sin(θ) [cm] → ΔPWM = h · K_h2p [µs/cm]
     *    PWM = PWM_CENTER + ΔPWM
     *
     *  合并: PWM = PWM_CENTER + sin(θ) · L · K_h2p
     *           = PWM_CENTER + sin(θ) · rad_to_pwm_scale
     *
     *  其中 rad_to_pwm_scale = L · K_h2p, 由现场标定得到.
     *
     *  使用 fast_sin_small() — 在 |θ| < θ_max = 0.15 rad 时
     *  相对误差 < 1e-6, 替代 sinf() 以规避软件仿真开销.
     * ---------------------------------------------------------------- */
    {
        float pwm_delta = BC_PWM_DIRECTION_SIGN *
                          fast_sin_small(bc->theta_target) *
                          bc->rad_to_pwm_scale;
        float pwm_float;
        float pos_error = bc->x_ref - bc->x_pos;
        if ((bc->pwm_min_drive_us > 0U) &&
            (fabsf_(pos_error) >= BC_MIN_DRIVE_ERROR_CM)) {
            float abs_error = fabsf_(pos_error);
            float abs_delta = fabsf_(pwm_delta);
            float drive_floor = (float)bc->pwm_min_drive_us;
            float direction_limit;

            if (pwm_delta > 0.001f) {
                direction_limit = (float)bc->pwm_positive_delta_limit_us;
            } else if (pwm_delta < -0.001f) {
                direction_limit = (float)bc->pwm_negative_delta_limit_us;
            } else {
                direction_limit = (pos_error > 0.0f) ?
                    (float)bc->pwm_negative_delta_limit_us :
                    (float)bc->pwm_positive_delta_limit_us;
            }

            if (abs_error >= BC_FULL_DRIVE_ERROR_CM) {
                drive_floor = direction_limit;
            } else if (abs_error > BC_MIN_DRIVE_ERROR_CM) {
                float ratio = (abs_error - BC_MIN_DRIVE_ERROR_CM) /
                    (BC_FULL_DRIVE_ERROR_CM - BC_MIN_DRIVE_ERROR_CM);
                drive_floor += ratio *
                    (direction_limit - (float)bc->pwm_min_drive_us);
            }

            if ((abs_delta > 0.001f) && (abs_delta < drive_floor)) {
                pwm_delta = (pwm_delta > 0.0f) ?
                    drive_floor :
                    -drive_floor;
            } else if (abs_delta <= 0.001f) {
                pwm_delta = (pos_error > 0.0f) ?
                    -BC_PWM_DIRECTION_SIGN * drive_floor :
                    BC_PWM_DIRECTION_SIGN * drive_floor;
            }
        }

        pwm_delta = fclamp(pwm_delta,
                           -(float)bc->pwm_negative_delta_limit_us,
                           (float)bc->pwm_positive_delta_limit_us);
        pwm_float = bc->pwm_neutral + pwm_delta;

        /* 硬限幅 [500, 2500] — 最后一道安全防线! */
        if (pwm_float < (float)BC_PWM_MIN_US) {
            pwm_float = (float)BC_PWM_MIN_US;
        } else if (pwm_float > (float)BC_PWM_MAX_US) {
            pwm_float = (float)BC_PWM_MAX_US;
        }

        pwm_float = slew_limit_pwm(pwm_float, (float)bc->pwm_pulse, bc->pwm_neutral);

        bc->pwm_pulse = (uint16_t)(pwm_float + 0.5f);   /* 四舍五入 */
    }

    /* ---- Run 结束, 外部读取 bc->pwm_pulse 写入 TIMER CCR ---- */
}

/* ======================================================================== *
 *  调试辅助
 * ======================================================================== */

float BalanceControl_GetStateMagnitude(const BalanceControl_t *bc)
{
    if (bc_unlikely(bc == NULL)) return 0.0f;

    /*
     * 使用 |e| + |v·T| 的近似范数, 替代 sqrt(e² + (vT)²)
     * 避免 sqrtf() 的软件仿真开销.
     * 两者在量级上一致, 仅用于调试/自适应调参的定性判断.
     */
    float pos_err = bc->x_pos - bc->x_ref;
    return fabsf_(pos_err) + fabsf_(bc->x_vel * BC_DT_S);
}

/* ======================================================================== *
 *  FreeRTOS 任务封装
 *
 *  如果项目使用 FreeRTOS, 取消下方 #if 1 并适配硬件寄存器.
 *  否则直接在 10 ms 定时器中断回调 (或主循环) 中调用 BalanceControl_Run().
 *
 *  用法:
 *     #include "FreeRTOS.h"
 *     #include "task.h"
 *     static BalanceControl_t g_balance;
 *     xTaskCreate(BalanceControl_Task, "balance", 256, &g_balance, 5, NULL);
 *
 *  注意: 使用 vTaskDelayUntil() 保证 10 ms 硬实时周期,
 *        不受调度器 jitter 影响.
 *
 *  平台适配: 当前项目未使用 FreeRTOS, 此段被 #if 0 禁用.
 *            直接在 TIMER_0 的 10ms 中断中调用 BalanceControl_Run():
 * ======================================================================== */
#if 0  /* → 改为 1 启用 FreeRTOS 任务 */
#include "FreeRTOS.h"
#include "task.h"

void BalanceControl_Task(void *param)
{
    BalanceControl_t *bc = (BalanceControl_t *)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* 等待硬件稳定 */
    vTaskDelay(pdMS_TO_TICKS(200));

    for (;;) {
        BalanceControl_Run(bc);

        /*
         * 写入硬件 PWM 比较器:
         *   DL_Timer_setCaptureCompareValue(STEPPER_PWM_INST,
         *       (uint32_t)bc->pwm_pulse, DL_TIMER_CC_0_INDEX);
         */

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
#endif /* FreeRTOS 任务封装 */
