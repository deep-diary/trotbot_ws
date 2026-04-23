#ifndef _DEEP_DOG_DOG_CONFIG_H_
#define _DEEP_DOG_DOG_CONFIG_H_

/**
 * 整机步态/底盘策略相关配置（从 board `config.h` 拆分）。
 * - 依赖 motor 的 `DEEP_DOG_MIT_VDES_RAD_S` 等宏（由 motor_config 提供）
 */

/** 步态一个正弦周期的采样点数（越大越平滑、步频任务更密） */
#ifndef DEEP_DOG_GAIT_TOTAL_STEPS
#define DEEP_DOG_GAIT_TOTAL_STEPS 50
#endif

/**
 * 周期优先配置（推荐）：
 * - 以「完整步态周期（一个正弦周期）」时长定义快慢范围；
 * - 每个小步间隔 step_period_ms 由 周期/采样点数 自动推导。
 */
#ifndef DEEP_DOG_BIG_STEP_PERIOD_MS_MIN
#define DEEP_DOG_BIG_STEP_PERIOD_MS_MIN 500
#endif
#ifndef DEEP_DOG_BIG_STEP_PERIOD_MS_MAX
#define DEEP_DOG_BIG_STEP_PERIOD_MS_MAX 4000
#endif
#ifndef DEEP_DOG_BIG_STEP_PERIOD_MS_DEFAULT
#define DEEP_DOG_BIG_STEP_PERIOD_MS_DEFAULT 2000
#endif

/** 兼容旧宏名：这里直接按完整步态周期使用（单位 ms） */
#define DEEP_DOG_CYCLE_PERIOD_MS_MIN (DEEP_DOG_BIG_STEP_PERIOD_MS_MIN)
#define DEEP_DOG_CYCLE_PERIOD_MS_MAX (DEEP_DOG_BIG_STEP_PERIOD_MS_MAX)
#define DEEP_DOG_CYCLE_PERIOD_MS_DEFAULT (DEEP_DOG_BIG_STEP_PERIOD_MS_DEFAULT)

/** step_period_ms = 全周期 / 采样点数（取整）；并保证最小为 1ms */
#define DEEP_DOG_STEP_PERIOD_MS_MIN \
    (((DEEP_DOG_CYCLE_PERIOD_MS_MIN / DEEP_DOG_GAIT_TOTAL_STEPS) > 0) ? \
         (DEEP_DOG_CYCLE_PERIOD_MS_MIN / DEEP_DOG_GAIT_TOTAL_STEPS) : 1)
#define DEEP_DOG_STEP_PERIOD_MS_MAX \
    (((DEEP_DOG_CYCLE_PERIOD_MS_MAX / DEEP_DOG_GAIT_TOTAL_STEPS) > 0) ? \
         (DEEP_DOG_CYCLE_PERIOD_MS_MAX / DEEP_DOG_GAIT_TOTAL_STEPS) : 1)
#define DEEP_DOG_STEP_PERIOD_MS_DEFAULT \
    (((DEEP_DOG_CYCLE_PERIOD_MS_DEFAULT / DEEP_DOG_GAIT_TOTAL_STEPS) > 0) ? \
         (DEEP_DOG_CYCLE_PERIOD_MS_DEFAULT / DEEP_DOG_GAIT_TOTAL_STEPS) : 1)

/**
 * 底盘 MCP `speed` 整型：÷100 后写入 `setContinuousSpeed`
 * - 位置模式：影响限速
 * - MIT 下：v_des 仍为 DEEP_DOG_MIT_VDES_RAD_S
 */
#ifndef DEEP_DOG_CHASSIS_SPEED_X100_MIN
#define DEEP_DOG_CHASSIS_SPEED_X100_MIN 20
#endif
#ifndef DEEP_DOG_CHASSIS_SPEED_X100_MAX
#define DEEP_DOG_CHASSIS_SPEED_X100_MAX 500
#endif

#endif  // _DEEP_DOG_DOG_CONFIG_H_
