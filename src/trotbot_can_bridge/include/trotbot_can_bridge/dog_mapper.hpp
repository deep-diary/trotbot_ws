#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "trotbot_can_bridge/motor_model.hpp"

namespace trotbot_can_bridge
{

struct MotorRoute
{
  size_t trajectory_index{0};
  uint8_t motor_id{0};
  CanBus bus{CanBus::CAN0};
  const char * joint_hint{""};
};

class DogMapper
{
public:
  // CHAMP 关节命名（来自 config/champ/joints.yaml 对应 12 关节链）
  static constexpr std::array<const char *, 12> kChampJointNames{{
    "base_lf1", "lf1_lf2", "lf2_lf3",
    "base_rf1", "rf1_rf2", "rf2_rf3",
    "base_lb1", "lb1_lb2", "lb2_lb3",
    "base_rb1", "rb1_rb2", "rb2_rb3",
  }};

  // P1 临时映射：先按 JointTrajectory.positions 索引做固定映射
  // 0..11 -> 11,12,13,21,22,23,51,52,53,61,62,63
  // 前腿 can0，后腿 can1
  static constexpr std::array<MotorRoute, 12> kTemporaryIndexMap{{
    {0, 11, CanBus::CAN0, "fl_hip_aa"},
    {1, 12, CanBus::CAN0, "fl_hip_fe"},
    {2, 13, CanBus::CAN0, "fl_knee"},
    {3, 21, CanBus::CAN0, "fr_hip_aa"},
    {4, 22, CanBus::CAN0, "fr_hip_fe"},
    {5, 23, CanBus::CAN0, "fr_knee"},
    {6, 51, CanBus::CAN1, "rl_hip_aa"},
    {7, 52, CanBus::CAN1, "rl_hip_fe"},
    {8, 53, CanBus::CAN1, "rl_knee"},
    {9, 61, CanBus::CAN1, "rr_hip_aa"},
    {10, 62, CanBus::CAN1, "rr_hip_fe"},
    {11, 63, CanBus::CAN1, "rr_knee"},
  }};

  static std::optional<MotorRoute> GetRouteByTrajectoryIndex(size_t idx)
  {
    if (idx >= kTemporaryIndexMap.size()) {
      return std::nullopt;
    }
    return kTemporaryIndexMap[idx];
  }

  static std::optional<MotorRoute> GetRouteByJointName(const std::string & joint_name)
  {
    for (size_t i = 0; i < kChampJointNames.size(); ++i) {
      if (joint_name == kChampJointNames[i]) {
        return GetRouteByTrajectoryIndex(i);
      }
    }
    return std::nullopt;
  }
};

}  // namespace trotbot_can_bridge
