#ifndef DOG_STATIC_POSES_H
#define DOG_STATIC_POSES_H

#include "leg/leg_control.h"

// 静态姿态点（4腿×3关节）。用于站立/趴下/左右倾/前后俯仰等“关键帧”。
// 说明：
// - 站立位以各腿 LegControl 的 stance 为基准（左右腿符号对称已在 LegControl 内配置）。
// - 其它静态点以“站立位 + 关节偏置(rad)”生成，避免手写 12 个绝对值带来的符号混乱。
enum class DogStaticPoseId : int {
    LieDownZero = 0,      // 卧倒/趴下：全关节 0（即零点）
    Stand = 1,            // 站立：各腿 stance
    TiltLeft = 2,         // 左倾（身体向左侧倾）
    TiltRight = 3,        // 右倾
    FrontDownBackUp = 4,  // 前趴后仰：前腿更“趴”(缩短) / 后腿更“仰”(伸长)
    FrontUpBackDown = 5,  // 前仰后趴：前腿更“仰”(伸长) / 后腿更“趴”(缩短)
};

const char* DogStaticPoseName(DogStaticPoseId id);

// 生成目标关节角（rad）。out[leg][joint]，leg: 0=FL,1=FR,2=RL,3=RR
// 生成后仍建议再做一次整机机械限位裁剪（DogControl::clampLegsMechanical）。
void FillDogStaticPose(DogStaticPoseId id, const LegControl legs[4], float out[4][LEG_JOINT_COUNT]);

#endif // DOG_STATIC_POSES_H

