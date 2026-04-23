#include "gait_planner.h"
#include <esp_log.h>

static const char* TAG = "GaitPlanner";

void GaitPlanner::setTotalSteps(uint16_t n) {
    if (n < 4) {
        n = 4;
    }
    if ((n & 1u) != 0u) {
        ESP_LOGW(TAG, "total_steps=%u 为奇数，Trot 需要半周期；已调整为 %u", (unsigned)n, (unsigned)(n + 1));
        n = (uint16_t)(n + 1u);
    }
    total_steps_ = n;
}

uint16_t GaitPlanner::halfPeriodSteps() const {
    return total_steps_ / 2;
}

void GaitPlanner::advanceCycleForward() {
    if (total_steps_ == 0) {
        return;
    }
    cycle_index_ = (uint16_t)((cycle_index_ + 1) % total_steps_);
}

void GaitPlanner::advanceCycleBackward() {
    if (total_steps_ == 0) {
        return;
    }
    cycle_index_ = (uint16_t)((cycle_index_ + total_steps_ - 1) % total_steps_);
}

uint16_t GaitPlanner::effectiveStepForLeg(int leg_index) const {
    if (total_steps_ == 0) {
        return 0;
    }
    if (leg_index < 0 || leg_index > 3) {
        return (uint16_t)(cycle_index_ % total_steps_);
    }

    uint16_t off = 0;
    switch (gait_type_) {
        case QuadrupedGaitType::SyncAllLegs:
            off = 0;
            break;
        case QuadrupedGaitType::Trot:
            // 对角同相：FL(0)+RR(3) 与 FR(1)+RL(2) 错开半周期
            if (leg_index == 1 || leg_index == 2) {
                off = halfPeriodSteps();
            }
            break;
    }

    return (uint16_t)((cycle_index_ + off) % total_steps_);
}
