#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trotbot_can_bridge/dog_mapper.hpp"
#include "trotbot_can_bridge/frame_codec.hpp"
#include "trotbot_can_bridge/protocol_codec.hpp"

using trajectory_msgs::msg::JointTrajectory;
using sensor_msgs::msg::JointState;
using std_msgs::msg::String;
using std_msgs::msg::UInt8MultiArray;

namespace trotbot_can_bridge
{

/// CHAMP 顺序：LF | RF | LH | RH，每腿 hip / thigh / calf；四腿关于机体镜像，偏移可由三个幅值生成：
/// LF(+h,+t,-c) RF(-h,-t,+c) LH(-h,+t,-c) RH(+h,-t,+c)，与 DogMapper::kTemporaryIndexMap 索引一致。
static std::vector<double> BuildMirrorJointOffsetsRad(double hip, double thigh, double calf_mag)
{
  return {
    hip, thigh, -calf_mag,
    -hip, -thigh, calf_mag,
    -hip, thigh, -calf_mag,
    hip, -thigh, calf_mag};
}

class MotorProtocolNode : public rclcpp::Node
{
public:
  MotorProtocolNode()
  : Node("motor_protocol_node")
  {
    kp_ = this->declare_parameter<double>("kp", 30.0);
    kd_ = this->declare_parameter<double>("kd", 1.5);
    default_velocity_ = this->declare_parameter<double>("default_velocity", 0.0);
    default_tau_ff_ = this->declare_parameter<double>("default_tau_ff", 0.0);
    enable_rx_decode_log_ = this->declare_parameter<bool>("enable_rx_decode_log", true);
    prefer_joint_name_mapping_ = this->declare_parameter<bool>("prefer_joint_name_mapping", true);
    joint_signs_ = this->declare_parameter<std::vector<double>>(
      "joint_signs", std::vector<double>(12, 1.0));

    joint_offsets_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_offsets_rad", std::vector<double>(12, 0.0));

    const double off_hip = this->declare_parameter<double>("joint_offset_hip_rad", 0.0);
    const double off_thigh = this->declare_parameter<double>("joint_offset_thigh_rad", 0.0);
    const double off_calf = this->declare_parameter<double>("joint_offset_calf_rad", 0.0);

    const bool expand_mirror = this->declare_parameter<bool>(
      "expand_joint_offsets_from_mirror", false);
    if (expand_mirror) {
      joint_offsets_rad_ = BuildMirrorJointOffsetsRad(off_hip, off_thigh, off_calf);
      RCLCPP_INFO(
        this->get_logger(),
        "expand_joint_offsets_from_mirror: hip=%.5f thigh=%.5f calf_mag=%.5f (overrides joint_offsets_rad)",
        off_hip, off_thigh, off_calf);
    }

    if (joint_signs_.size() != 12 || joint_offsets_rad_.size() != 12) {
      RCLCPP_WARN(
        this->get_logger(),
        "joint_signs/joint_offsets_rad size mismatch, reset to 12 defaults.");
      joint_signs_ = std::vector<double>(12, 1.0);
      joint_offsets_rad_ = std::vector<double>(12, 0.0);
    }

    publish_mit_mapped_ = this->declare_parameter<bool>("publish_mit_mapped_positions", true);
    mit_mapped_topic_ = this->declare_parameter<std::string>(
      "mit_mapped_positions_topic", "/mit_motor_position_rad");

    traj_sub_ = this->create_subscription<JointTrajectory>(
      "/joint_group_effort_controller/joint_trajectory", rclcpp::SensorDataQoS(),
      std::bind(&MotorProtocolNode::OnTrajectory, this, std::placeholders::_1));
    rx_sub_ = this->create_subscription<UInt8MultiArray>(
      "/can_rx_frames", rclcpp::SensorDataQoS(),
      std::bind(&MotorProtocolNode::OnRxFrame, this, std::placeholders::_1));

    tx_pub_ = this->create_publisher<UInt8MultiArray>("/can_tx_frames", rclcpp::SensorDataQoS());
    feedback_pub_ = this->create_publisher<String>("/motor_feedback", 50);

    if (publish_mit_mapped_) {
      mapped_positions_pub_ = this->create_publisher<JointState>(mit_mapped_topic_, 10);
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing MIT-mapped angles (θ_mit=sign*θ_champ+offset) on '%s' (same joint order as trajectory)",
        mit_mapped_topic_.c_str());
    }

    RCLCPP_WARN(
      this->get_logger(),
      "P1 temporary mapping enabled: positions[0..11] -> IDs [11,12,13,21,22,23,51,52,53,61,62,63], "
      "front legs on can0, rear legs on can1. Zero offset and mechanical limits are intentionally skipped.");
  }

private:
  void OnTrajectory(const JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore trajectory without points");
      return;
    }

    const auto & positions = msg->points.front().positions;
    uint32_t front_count = 0;
    uint32_t rear_count = 0;
    size_t total_sent = 0;

    std::array<double, 12> mapped_rad{};
    mapped_rad.fill(std::numeric_limits<double>::quiet_NaN());

    // P2 前置：优先按 joint_names 名称映射；缺失时回退到索引映射
    if (
      prefer_joint_name_mapping_ &&
      !msg->joint_names.empty() &&
      msg->joint_names.size() == positions.size())
    {
      std::unordered_map<std::string, size_t> name_to_idx;
      name_to_idx.reserve(msg->joint_names.size());
      for (size_t i = 0; i < msg->joint_names.size(); ++i) {
        name_to_idx[msg->joint_names[i]] = i;
      }

      for (size_t i = 0; i < DogMapper::kChampJointNames.size(); ++i) {
        const auto it = name_to_idx.find(DogMapper::kChampJointNames[i]);
        if (it == name_to_idx.end()) {
          continue;
        }
        const auto route = DogMapper::GetRouteByTrajectoryIndex(i);
        if (!route.has_value()) {
          continue;
        }
        SendMitFrame(route.value(), positions[it->second], front_count, rear_count, &mapped_rad);
        ++total_sent;
      }
    } else {
      const size_t count = std::min(positions.size(), DogMapper::kTemporaryIndexMap.size());
      if (count < DogMapper::kTemporaryIndexMap.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Trajectory positions size=%zu (<12), partial output only", positions.size());
      }
      for (size_t i = 0; i < count; ++i) {
        const auto route = DogMapper::GetRouteByTrajectoryIndex(i);
        if (!route.has_value()) {
          continue;
        }
        SendMitFrame(route.value(), positions[i], front_count, rear_count, &mapped_rad);
        ++total_sent;
      }
    }

    if (publish_mit_mapped_ && mapped_positions_pub_) {
      JointState js;
      js.header.stamp = this->now();
      js.name.reserve(DogMapper::kChampJointNames.size());
      for (const char * jn : DogMapper::kChampJointNames) {
        js.name.emplace_back(jn);
      }
      js.position.assign(mapped_rad.begin(), mapped_rad.end());
      mapped_positions_pub_->publish(js);
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MIT publish: total=%zu front(can0)=%u rear(can1)=%u kp=%.2f kd=%.2f v=%.2f tau=%.2f",
      total_sent, front_count, rear_count, kp_, kd_, default_velocity_, default_tau_ff_);
  }

  void SendMitFrame(
    const MotorRoute & route, double position,
    uint32_t & front_count, uint32_t & rear_count,
    std::array<double, 12> * mapped_out = nullptr)
  {
    const size_t idx = std::min(route.trajectory_index, static_cast<size_t>(11));
    const double mapped_position = joint_signs_[idx] * position + joint_offsets_rad_[idx];
    if (mapped_out != nullptr) {
      (*mapped_out)[idx] = mapped_position;
    }

    const auto frame = ProtocolCodec::BuildMitControlFrame(
      route.bus,
      route.motor_id,
      static_cast<float>(mapped_position),
      static_cast<float>(default_velocity_),
      static_cast<float>(kp_),
      static_cast<float>(kd_),
      static_cast<float>(default_tau_ff_));

    auto packed = FrameCodec::Pack(frame);
    tx_pub_->publish(packed);
    if (route.bus == CanBus::CAN0) {
      ++front_count;
    } else {
      ++rear_count;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "MIT TX joint=%s motor=%u %s",
      route.joint_hint,
      route.motor_id,
      ProtocolCodec::FrameSummary(frame).c_str());
  }

  void OnRxFrame(const UInt8MultiArray::SharedPtr msg)
  {
    if (!enable_rx_decode_log_) {
      return;
    }

    const auto frame = FrameCodec::Unpack(*msg);
    if (!frame.has_value()) {
      return;
    }

    const auto feedback = ProtocolCodec::DecodeFeedback(frame.value());
    if (!feedback.has_value()) {
      return;
    }

    std::ostringstream oss;
    oss << "bus=" << feedback->source_bus
        << " motor=" << static_cast<int>(feedback->motor_id)
        << " mode=" << static_cast<int>(feedback->mode_status)
        << " angle=" << feedback->current_angle
        << " speed=" << feedback->current_speed
        << " torque=" << feedback->current_torque
        << " temp=" << feedback->current_temp
        << " err=" << (feedback->error_status ? "1" : "0");

    String out;
    out.data = oss.str();
    feedback_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", out.data.c_str());
  }

  double kp_{30.0};
  double kd_{1.5};
  double default_velocity_{0.0};
  double default_tau_ff_{0.0};
  bool enable_rx_decode_log_{true};
  bool prefer_joint_name_mapping_{true};
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_rad_;
  bool publish_mit_mapped_{false};
  std::string mit_mapped_topic_;
  rclcpp::Publisher<JointState>::SharedPtr mapped_positions_pub_;

  rclcpp::Subscription<JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<String>::SharedPtr feedback_pub_;
};

}  // namespace trotbot_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trotbot_can_bridge::MotorProtocolNode>());
  rclcpp::shutdown();
  return 0;
}
