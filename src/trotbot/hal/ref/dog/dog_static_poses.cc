#include "dog_static_poses.h"

static inline void FillStandFromLegs(const LegControl legs[4], float out[4][LEG_JOINT_COUNT]) {
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            out[leg][j] = legs[leg].getStanceTargetJoint(j);
        }
    }
}

static inline void FillZeros(float out[4][LEG_JOINT_COUNT]) {
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            out[leg][j] = 0.0f;
        }
    }
}

const char* DogStaticPoseName(DogStaticPoseId id) {
    switch (id) {
        case DogStaticPoseId::LieDownZero:     return "lie_down_zero";
        case DogStaticPoseId::Stand:           return "stand";
        case DogStaticPoseId::TiltLeft:        return "tilt_left";
        case DogStaticPoseId::TiltRight:       return "tilt_right";
        case DogStaticPoseId::FrontDownBackUp: return "front_down_back_up";
        case DogStaticPoseId::FrontUpBackDown: return "front_up_back_down";
        default:                               return "unknown_pose";
    }
}

void FillDogStaticPose(DogStaticPoseId id, const LegControl legs[4], float out[4][LEG_JOINT_COUNT]) {
    // 基准：站立位
    float base[4][LEG_JOINT_COUNT] = {};
    FillStandFromLegs(legs, base);

    // 默认输出为站立位
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            out[leg][j] = base[leg][j];
        }
    }

    if (id == DogStaticPoseId::Stand) {
        return;
    }
    if (id == DogStaticPoseId::LieDownZero) {
        FillZeros(out);
        return;
    }

    // 下面所有静态点都在“行走范围附近的小幅偏置”，避免一上来就撞机械限位。
    // 单位：rad
    // 经验偏置（rad）
    // 说明：
    // - TiltLeft/TiltRight 这里按“真实侧倾（roll，左右高低）”实现：通过左右两侧腿长差（主要膝）制造高低差，
    //   这样更接近“站在地上倾斜”的效果；HipAA 仅做很小辅助避免脚向外撇。
    const float aa = 0.03f;        // HipAA：轻微辅助（不再用大幅度镜像）
    const float fe = 0.10f;        // HipFE：俯仰
    const float knee = 0.30f;      // 膝：伸长/缩短腿长，控制前后俯仰体感
    const float fe_roll = 0.03f;   // roll 时 HipFE 配合幅度（小）
    const float knee_roll = 0.18f; // roll 时膝关节腿长差幅度（建议先小后大）

    // leg index: 0=FL,1=FR,2=RL,3=RR
    const bool is_front[4] = {true, true, false, false};
    const bool is_left[4] = {true, false, true, false};

    for (int leg = 0; leg < 4; leg++) {
        switch (id) {
            case DogStaticPoseId::TiltLeft: {
                // 左侧倾：左侧“压低”、右侧“抬高”
                // 左侧腿缩短（膝更弯，朝 0），右侧腿变长（膝更直，远离 0）。
                // 膝关节左右符号相反：用 base[knee] 的正负来决定“朝 0 / 远离 0”的符号方向。
                const float s = (base[leg][LEG_JOINT_KNEE] >= 0.0f) ? 1.0f : -1.0f;
                out[leg][LEG_JOINT_HIP_AA] = base[leg][LEG_JOINT_HIP_AA] + (is_left[leg] ? +aa : -aa);
                out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] + (is_left[leg] ? +fe_roll : -fe_roll);
                out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (is_left[leg] ? -s * knee_roll : +s * knee_roll);
            } break;
            case DogStaticPoseId::TiltRight: {
                // 右侧倾：右侧“压低”、左侧“抬高”
                const float s = (base[leg][LEG_JOINT_KNEE] >= 0.0f) ? 1.0f : -1.0f;
                out[leg][LEG_JOINT_HIP_AA] = base[leg][LEG_JOINT_HIP_AA] + (is_left[leg] ? -aa : +aa);
                out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] + (is_left[leg] ? -fe_roll : +fe_roll);
                out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (is_left[leg] ? +s * knee_roll : -s * knee_roll);
            } break;
            case DogStaticPoseId::FrontDownBackUp: {
                // 前趴后仰：前腿缩短（膝更弯，接近 0），后腿伸长（膝更直，远离 0）
                // 同时给一点髋前后偏置，增强观感
                if (is_front[leg]) {
                    out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] + fe;
                    out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (base[leg][LEG_JOINT_KNEE] >= 0.0f ? -knee : +knee);
                } else {
                    out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] - fe;
                    out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (base[leg][LEG_JOINT_KNEE] >= 0.0f ? +knee : -knee);
                }
            } break;
            case DogStaticPoseId::FrontUpBackDown: {
                // 前仰后趴：前腿伸长，后腿缩短
                if (is_front[leg]) {
                    out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] - fe;
                    out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (base[leg][LEG_JOINT_KNEE] >= 0.0f ? +knee : -knee);
                } else {
                    out[leg][LEG_JOINT_HIP_FE] = base[leg][LEG_JOINT_HIP_FE] + fe;
                    out[leg][LEG_JOINT_KNEE]   = base[leg][LEG_JOINT_KNEE] + (base[leg][LEG_JOINT_KNEE] >= 0.0f ? -knee : +knee);
                }
            } break;
            default:
                break;
        }
    }
}

