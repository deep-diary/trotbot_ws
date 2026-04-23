#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "dog/dog_config.h"
#include "motor/motor_config.h"

class DeepMotor;

/**
 * 腿类型：前左、前右、后左、后右
 */
enum class LegType {
    FL = 0,  // 前左
    FR = 1,
    RL = 2,
    RR = 3
};

/** 单腿关节数 */
#define LEG_JOINT_COUNT 3

/** 关节索引 */
#define LEG_JOINT_HIP_AA   0  // 髋侧摆
#define LEG_JOINT_HIP_FE   1  // 髋前后
#define LEG_JOINT_KNEE     2  // 膝

/** 默认步态总步数（一步周期内的采样点数） */
#define LEG_DEFAULT_TOTAL_STEPS DEEP_DOG_GAIT_TOTAL_STEPS

/** 默认站立位（相对零位，弧度） */
#ifndef DEEP_DOG_STANCE_HIP_AA_RAD
#define DEEP_DOG_STANCE_HIP_AA_RAD 0.0f
#endif
#ifndef DEEP_DOG_STANCE_HIP_FE_RAD
#define DEEP_DOG_STANCE_HIP_FE_RAD 0.2f
#endif
#ifndef DEEP_DOG_STANCE_KNEE_RAD
#define DEEP_DOG_STANCE_KNEE_RAD 1.4f
#endif

/** 默认迈步关节幅度（弧度），用于正弦偏移 */
// 依据 minidog.urdf 关节可动范围，采用更保守的步态幅度以降低横摆与俯仰抖动。
#define LEG_DEFAULT_HIP_FE_AMP   0.16f
#define LEG_DEFAULT_KNEE_AMP     0.18f
#define LEG_DEFAULT_HIP_AA_AMP   0.01f

/**
 * 摆动相抬脚：sin 半周内 sin² 包络；左膝站立为负、右膝为正，抬脚项为左加右减（见 leg_control.cc）。
 */
#ifndef LEG_KNEE_SWING_LIFT_RAD
#define LEG_KNEE_SWING_LIFT_RAD 0.30f
#endif
/**
 * 髋前后 cos 分量：打破纯 sin 对称；实机若「前进后退都向后漂」可维持负号或加大模长试参。
 */
#ifndef LEG_HIP_FE_COS_AMP
#define LEG_HIP_FE_COS_AMP (-0.10f)
#endif

/**
 * 单腿控制类：一条腿 3 个关节，对应 3 个电机。
 * 负责站立/卧倒/迈一步的目标位置计算，通过 DeepMotor 下发。
 */
class LegControl {
public:
    LegControl();
    ~LegControl() = default;

    /** 绑定电机管理器（必须在调用 init/stand/step 前设置） */
    void setDeepMotor(DeepMotor* motor) { deep_motor_ = motor; }
    DeepMotor* getDeepMotor() const { return deep_motor_; }

    /** 设置腿类型（用于默认电机 ID 与限位，见 dog/README） */
    void setLegType(LegType type);
    LegType getLegType() const { return leg_type_; }

    /** 设置本腿 3 个电机 ID（髋侧摆、髋前后、膝） */
    void setMotorIds(uint8_t hip_aa, uint8_t hip_fe, uint8_t knee);

    /** 设置 3 关节控制上下限（弧度） */
    void setLimits(const float limit_low[LEG_JOINT_COUNT], const float limit_high[LEG_JOINT_COUNT]);

    /** 设置站立位（弧度，3 关节） */
    void setStancePosition(const float stance[LEG_JOINT_COUNT]);

    /** 设置步态总步数、当前步数 */
    void setTotalSteps(uint16_t total) { total_steps_ = total; }
    void setCurrentStep(uint16_t step) { current_step_ = step; }
    uint16_t getCurrentStep() const { return current_step_; }
    uint16_t getTotalSteps() const { return total_steps_; }

    /** 腿初始化：3 电机；模式由 config.h `DEEP_DOG_USE_MIT_WALK` 决定（MIT 或位置模式） */
    bool init();

    /** 腿失能：3 电机 reset */
    bool disable();

    /** 回到零位（卧倒）：3 关节置 0 */
    bool goToZero(float max_speed_rad_s = DEEP_DOG_MIT_VDES_RAD_S);

    /** 回到站立位 */
    bool goToStance(float max_speed_rad_s = DEEP_DOG_MIT_VDES_RAD_S);

    /** 向前迈一步：current_step_ += 1，目标 = 站立位 + 正弦偏移，并限幅 */
    bool stepForward(float max_speed_rad_s = DEEP_DOG_MIT_VDES_RAD_S);

    /** 向后迈一步：current_step_ -= 1 */
    bool stepBackward(float max_speed_rad_s = DEEP_DOG_MIT_VDES_RAD_S);

    /** 将角度夹在控制上下限内 */
    float clampJoint(int joint_index, float value) const;

    /**
     * 机械限位（README 机械下/上极限）：最后一道安全边界，下发前必须再夹紧一次。
     * 与行走用的 limit_low_/high_ 独立；行走范围应窄于机械范围。
     */
    float clampJointMechanical(int joint_index, float value) const;

    /** 就地夹紧 3 关节到机械限位；若有裁剪打一条警告日志 */
    void clampJointPositionsMechanical(float pos[LEG_JOINT_COUNT]) const;

    /** 关节索引 0..LEG_JOINT_COUNT-1，对应电机 ID（0 表示未配置） */
    uint8_t getMotorId(int joint_index) const;

    /** 站立位目标（已按限幅），关节索引 0..2 */
    float getStanceTargetJoint(int joint_index) const;

    /** 在当前 current_step_ 下填充 3 关节目标（不修改步数） */
    void fillCurrentStepPositions(float out[LEG_JOINT_COUNT], bool forward) const;

    /**
     * 使用指定周期内步数索引计算正弦步态目标（不修改 current_step_）。
     * 供整机 GaitPlanner 按腿分配不同相位。
     */
    void fillStepPositionsAtStepIndex(uint16_t step_index, float out[LEG_JOINT_COUNT], bool forward) const;

    /** 步进相位 +1 / -1（单腿 MCP / 旧版同步步态用） */
    void advanceStepForward();
    void advanceStepBackward();

private:
    DeepMotor* deep_motor_ = nullptr;
    LegType leg_type_ = LegType::FL;
    uint8_t motor_ids_[LEG_JOINT_COUNT] = {0, 0, 0};
    float stance_position_[LEG_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
    float limit_low_[LEG_JOINT_COUNT] = {-0.2f, -0.5f, -1.0f};
    float limit_high_[LEG_JOINT_COUNT] = {0.2f, 0.5f, 1.0f};
    float mech_limit_low_[LEG_JOINT_COUNT] = {-0.3f, -0.6f, -2.26f};
    float mech_limit_high_[LEG_JOINT_COUNT] = {0.3f, 1.4f, 0.0f};
    uint16_t total_steps_ = LEG_DEFAULT_TOTAL_STEPS;
    uint16_t current_step_ = 0;

    /** 正弦步态幅度（弧度） */
    float hip_fe_amp_ = LEG_DEFAULT_HIP_FE_AMP;
    float knee_amp_ = LEG_DEFAULT_KNEE_AMP;
    float hip_aa_amp_ = LEG_DEFAULT_HIP_AA_AMP;
    float knee_swing_lift_ = LEG_KNEE_SWING_LIFT_RAD;
    float hip_fe_cos_amp_ = LEG_HIP_FE_COS_AMP;

    /** 根据步数索引计算 3 关节目标（弧度）；forward=false 时正弦项取反，表示后退摆动方向 */
    void computeStepPositionAt(uint16_t step_index, float out_position[LEG_JOINT_COUNT], bool forward) const;

    void computeStepPosition(float out_position[LEG_JOINT_COUNT], bool forward) const;

    /** 限速 + 下发（DEEP_DOG_USE_MIT_WALK 时运控帧，否则 PARAM_LOC_REF） */
    bool sendThreeJointsPositionOrMit(const float pos[LEG_JOINT_COUNT], float target_velocity_rad_s);
};

/**
 * 注册单腿 MCP 工具（初始化、站立、卧倒、迈一步等）。
 * legs 为 4 条腿指针，可为 nullptr 表示该腿未使用（如仅调试单腿时只建 1 个 LegControl）。
 */
void RegisterLegMcpTools(class McpServer& mcp_server, LegControl* legs[4]);

#endif // LEG_CONTROL_H
