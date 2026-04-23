#ifndef DEEP_DOG_GAIT_PLANNER_H
#define DEEP_DOG_GAIT_PLANNER_H

#include <stdint.h>

/**
 * 四足步态类型（无 URDF，仅用正弦相位 + 周期内离散索引）。
 * - SyncAllLegs：四腿同一相位（与旧版「四腿同时迈一步」一致）。
 * - Trot：对角小跑，FL–RR 同相，FR–RL 相对半周期（常见稳定步态）。
 */
enum class QuadrupedGaitType : uint8_t {
    SyncAllLegs = 0,
    Trot = 1,
};

/**
 * 整机步态规划：维护 **全局周期索引** cycle_index ∈ [0, total_steps-1]，
 * 每条腿通过 `effectiveStepForLeg(leg)` 得到用于 sin(ω·step) 的 **有效步数**。
 * LegControl 侧仍用原有正弦幅度与 clamp，仅改变各腿相对相位。
 */
class GaitPlanner {
public:
    GaitPlanner() = default;

    void setGaitType(QuadrupedGaitType t) { gait_type_ = t; }
    QuadrupedGaitType getGaitType() const { return gait_type_; }

    /** 一个完整步态周期内的离散采样数（与 LegControl::total_steps_ 一致，建议为偶数以便 Trot 半周期） */
    void setTotalSteps(uint16_t n);
    uint16_t getTotalSteps() const { return total_steps_; }

    /** 当前全局周期位置（一次「前/后一步」只改变该索引） */
    uint16_t getCycleIndex() const { return cycle_index_; }

    void advanceCycleForward();
    void advanceCycleBackward();
    void resetCycle() { cycle_index_ = 0; }

    /**
     * leg_index：0=FL, 1=FR, 2=RL, 3=RR（与 DogControl::legs_ 顺序一致）
     */
    uint16_t effectiveStepForLeg(int leg_index) const;

private:
    uint16_t halfPeriodSteps() const;

    QuadrupedGaitType gait_type_ = QuadrupedGaitType::Trot;
    uint16_t total_steps_ = 20;
    uint16_t cycle_index_ = 0;
};

#endif