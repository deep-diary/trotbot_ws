#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
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

enum class TuneScope
{
  ALL = 0,
  FRONT = 1,
  REAR = 2
};

static std::string ToLower(std::string v)
{
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return v;
}

static std::vector<std::string> SplitTokens(const std::string & text)
{
  std::istringstream iss(text);
  std::vector<std::string> out;
  std::string token;
  while (iss >> token) {
    out.push_back(token);
  }
  return out;
}

static bool ParseBool(const std::string & text)
{
  const std::string v = ToLower(text);
  return (v == "1" || v == "true" || v == "yes" || v == "on");
}

static TuneScope ParseScope(const std::string & text)
{
  const std::string v = ToLower(text);
  if (v == "front" || v == "can0") {
    return TuneScope::FRONT;
  }
  if (v == "rear" || v == "can1") {
    return TuneScope::REAR;
  }
  return TuneScope::ALL;
}

static std::optional<MotorRoute> GetRouteByMotorId(uint8_t motor_id)
{
  for (const auto & route : DogMapper::kTemporaryIndexMap) {
    if (route.motor_id == motor_id) {
      return route;
    }
  }
  return std::nullopt;
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
    runtime_tune_topic_ = this->declare_parameter<std::string>("runtime_tune_topic", "/mit_gains_cmd");
    publish_feedback_joint_states_ = this->declare_parameter<bool>("publish_feedback_joint_states", true);
    feedback_joint_states_topic_ = this->declare_parameter<std::string>(
      "feedback_joint_states_topic", "/joint_states_feedback");
    use_joint_cmd_limits_ = this->declare_parameter<bool>("use_joint_cmd_limits", true);
    joint_cmd_min_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_cmd_min_rad", std::vector<double>(12, -3.14));
    joint_cmd_max_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_cmd_max_rad", std::vector<double>(12, 3.14));
    enable_startup_smoothing_ = this->declare_parameter<bool>("enable_startup_smoothing", true);
    startup_smoothing_duration_s_ = this->declare_parameter<double>("startup_smoothing_duration_s", 1.5);
    command_max_velocity_rad_s_ = this->declare_parameter<double>("command_max_velocity_rad_s", 6.0);
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
    tune_sub_ = this->create_subscription<String>(
      runtime_tune_topic_, 10,
      std::bind(&MotorProtocolNode::OnTuneCommand, this, std::placeholders::_1));

    tx_pub_ = this->create_publisher<UInt8MultiArray>("/can_tx_frames", rclcpp::SensorDataQoS());
    feedback_pub_ = this->create_publisher<String>("/motor_feedback", 50);
    if (publish_feedback_joint_states_) {
      feedback_joint_states_pub_ = this->create_publisher<JointState>(feedback_joint_states_topic_, 10);
    }

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
      "front legs on can0, rear legs on can1.");
    RCLCPP_INFO(
      this->get_logger(),
      "Runtime MIT tuning topic: %s (example: 'scope=all kp=25 kd=1.8 tau=0.3' or 'scope=rear tau=0.6' or 'reset=1')",
      runtime_tune_topic_.c_str());
    ResetRuntimeTuning();
    last_commanded_mit_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    last_sent_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    boot_feedback_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    boot_feedback_captured_.fill(false);
    has_last_sent_.fill(false);

    if (joint_cmd_min_rad_.size() != 12 || joint_cmd_max_rad_.size() != 12) {
      RCLCPP_WARN(this->get_logger(), "joint_cmd_min_rad/max size mismatch, using default [-3.14, 3.14]x12");
      joint_cmd_min_rad_ = std::vector<double>(12, -3.14);
      joint_cmd_max_rad_ = std::vector<double>(12, 3.14);
    }
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
      total_sent, front_count, rear_count, runtime_kp_can0_, runtime_kd_can0_, default_velocity_, runtime_tau_can0_);
  }

  void SendMitFrame(
    const MotorRoute & route, double champ_position_raw,
    uint32_t & front_count, uint32_t & rear_count,
    std::array<double, 12> * mapped_out = nullptr)
  {
    const size_t idx = std::min(route.trajectory_index, static_cast<size_t>(11));
    const double champ_limited = ClampJointCommand(idx, champ_position_raw);
    const double champ_smoothed = SmoothJointCommand(idx, champ_limited);
    const double mapped_position = joint_signs_[idx] * champ_smoothed + joint_offsets_rad_[idx];
    if (mapped_out != nullptr) {
      (*mapped_out)[idx] = mapped_position;
    }

    const bool is_front = (route.bus == CanBus::CAN0);
    const double use_kp = is_front ? runtime_kp_can0_ : runtime_kp_can1_;
    const double use_kd = is_front ? runtime_kd_can0_ : runtime_kd_can1_;
    const double use_tau = is_front ? runtime_tau_can0_ : runtime_tau_can1_;

    const auto frame = ProtocolCodec::BuildMitControlFrame(
      route.bus,
      route.motor_id,
      static_cast<float>(mapped_position),
      static_cast<float>(default_velocity_),
      static_cast<float>(use_kp),
      static_cast<float>(use_kd),
      static_cast<float>(use_tau));

    auto packed = FrameCodec::Pack(frame);
    tx_pub_->publish(packed);
    if (route.bus == CanBus::CAN0) {
      ++front_count;
    } else {
      ++rear_count;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "MIT TX joint=%s motor=%u raw=%.4f lim=%.4f smooth=%.4f %s",
      route.joint_hint,
      route.motor_id,
      champ_position_raw,
      champ_limited,
      champ_smoothed,
      ProtocolCodec::FrameSummary(frame).c_str());

    if (route.motor_id < last_commanded_mit_rad_.size()) {
      last_commanded_mit_rad_[route.motor_id] = mapped_position;
    }
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
    const double expected_mit = feedback->motor_id < last_commanded_mit_rad_.size() ?
      last_commanded_mit_rad_[feedback->motor_id] :
      std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(expected_mit)) {
      const double abs_err = std::fabs(expected_mit - feedback->current_angle);
      ++error_sample_count_;
      error_abs_sum_ += abs_err;
      error_abs_max_ = std::max(error_abs_max_, abs_err);
    }

    if (const auto route = GetRouteByMotorId(feedback->motor_id); route.has_value()) {
      const size_t idx = std::min(route->trajectory_index, static_cast<size_t>(11));
      const double sign = std::fabs(joint_signs_[idx]) < 1e-6 ? 1.0 : joint_signs_[idx];
      const double champ_feedback = (feedback->current_angle - joint_offsets_rad_[idx]) / sign;
      last_feedback_champ_rad_[idx] = champ_feedback;
      if (!boot_feedback_captured_[idx]) {
        boot_feedback_champ_rad_[idx] = champ_feedback;
        boot_feedback_captured_[idx] = true;
      }
      if (publish_feedback_joint_states_ && feedback_joint_states_pub_) {
        PublishFeedbackJointStates();
      }
    }

    oss << "bus=" << feedback->source_bus
        << " motor=" << static_cast<int>(feedback->motor_id)
        << " mode=" << static_cast<int>(feedback->mode_status)
        << " angle=" << feedback->current_angle
        << " speed=" << feedback->current_speed
        << " torque=" << feedback->current_torque
        << " temp=" << feedback->current_temp
        << " err=" << (feedback->error_status ? "1" : "0");
    if (std::isfinite(expected_mit)) {
      oss << " cmd_angle=" << expected_mit
          << " abs_err=" << std::fabs(expected_mit - feedback->current_angle);
    }

    String out;
    out.data = oss.str();
    feedback_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", out.data.c_str());
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MIT tracking abs_err(rad): mean=%.4f max=%.4f samples=%zu | can0(kp=%.2f,kd=%.2f,tau=%.2f) can1(kp=%.2f,kd=%.2f,tau=%.2f)",
      error_sample_count_ > 0 ? (error_abs_sum_ / static_cast<double>(error_sample_count_)) : 0.0,
      error_abs_max_,
      error_sample_count_,
      runtime_kp_can0_, runtime_kd_can0_, runtime_tau_can0_,
      runtime_kp_can1_, runtime_kd_can1_, runtime_tau_can1_);
  }

  void PublishFeedbackJointStates()
  {
    JointState js;
    js.header.stamp = this->now();
    js.name.reserve(DogMapper::kChampJointNames.size());
    js.position.reserve(DogMapper::kChampJointNames.size());
    for (size_t i = 0; i < DogMapper::kChampJointNames.size(); ++i) {
      js.name.emplace_back(DogMapper::kChampJointNames[i]);
      const double pos = std::isfinite(last_feedback_champ_rad_[i]) ? last_feedback_champ_rad_[i] : 0.0;
      js.position.emplace_back(pos);
    }
    feedback_joint_states_pub_->publish(js);
  }

  void ResetRuntimeTuning()
  {
    runtime_kp_can0_ = Clamp(kp_, ProtocolCodec::kKpMin, ProtocolCodec::kKpMax);
    runtime_kp_can1_ = runtime_kp_can0_;
    runtime_kd_can0_ = Clamp(kd_, ProtocolCodec::kKdMin, ProtocolCodec::kKdMax);
    runtime_kd_can1_ = runtime_kd_can0_;
    runtime_tau_can0_ = Clamp(default_tau_ff_, ProtocolCodec::kTMin, ProtocolCodec::kTMax);
    runtime_tau_can1_ = runtime_tau_can0_;
  }

  static double Clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  double ClampJointCommand(size_t idx, double champ_position) const
  {
    if (!use_joint_cmd_limits_ || idx >= joint_cmd_min_rad_.size() || idx >= joint_cmd_max_rad_.size()) {
      return champ_position;
    }
    return Clamp(champ_position, joint_cmd_min_rad_[idx], joint_cmd_max_rad_[idx]);
  }

  double SmoothJointCommand(size_t idx, double champ_position)
  {
    const rclcpp::Time now = this->now();
    if (first_command_time_.nanoseconds() == 0) {
      first_command_time_ = now;
      last_command_time_ = now;
    }

    double target = champ_position;
    if (
      enable_startup_smoothing_ &&
      startup_smoothing_duration_s_ > 1e-3 &&
      idx < boot_feedback_captured_.size() &&
      boot_feedback_captured_[idx] &&
      std::isfinite(boot_feedback_champ_rad_[idx]))
    {
      const double elapsed = (now - first_command_time_).seconds();
      const double alpha = Clamp(elapsed / startup_smoothing_duration_s_, 0.0, 1.0);
      target = boot_feedback_champ_rad_[idx] + alpha * (champ_position - boot_feedback_champ_rad_[idx]);
    }

    const double dt = std::max((now - last_command_time_).seconds(), 1e-3);
    const double max_step = std::max(0.0, command_max_velocity_rad_s_) * dt;
    if (!has_last_sent_[idx]) {
      if (idx < boot_feedback_captured_.size() && boot_feedback_captured_[idx] && std::isfinite(boot_feedback_champ_rad_[idx])) {
        last_sent_champ_rad_[idx] = boot_feedback_champ_rad_[idx];
      } else {
        last_sent_champ_rad_[idx] = target;
      }
      has_last_sent_[idx] = true;
    }
    const double prev = last_sent_champ_rad_[idx];
    const double delta = Clamp(target - prev, -max_step, max_step);
    const double out = prev + delta;
    last_sent_champ_rad_[idx] = out;
    last_command_time_ = now;
    return out;
  }

  static bool ParseKV(const std::string & token, std::string & key, std::string & value)
  {
    const auto pos = token.find('=');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= token.size()) {
      return false;
    }
    key = ToLower(token.substr(0, pos));
    value = token.substr(pos + 1);
    return true;
  }

  void OnTuneCommand(const String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    auto scope = TuneScope::ALL;
    bool do_reset = false;
    bool has_kp = false;
    bool has_kd = false;
    bool has_tau = false;
    double new_kp = 0.0;
    double new_kd = 0.0;
    double new_tau = 0.0;

    const auto tokens = SplitTokens(msg->data);
    for (const auto & token : tokens) {
      std::string key;
      std::string value;
      if (!ParseKV(token, key, value)) {
        continue;
      }
      if (key == "scope") {
        scope = ParseScope(value);
        continue;
      }
      if (key == "reset") {
        do_reset = ParseBool(value);
        continue;
      }
      try {
        const double parsed = std::stod(value);
        if (key == "kp") {
          new_kp = Clamp(parsed, ProtocolCodec::kKpMin, ProtocolCodec::kKpMax);
          has_kp = true;
        } else if (key == "kd") {
          new_kd = Clamp(parsed, ProtocolCodec::kKdMin, ProtocolCodec::kKdMax);
          has_kd = true;
        } else if (key == "tau" || key == "tau_ff") {
          new_tau = Clamp(parsed, ProtocolCodec::kTMin, ProtocolCodec::kTMax);
          has_tau = true;
        }
      } catch (const std::exception &) {
        RCLCPP_WARN(this->get_logger(), "Ignore invalid tune token: %s", token.c_str());
      }
    }

    if (do_reset) {
      ResetRuntimeTuning();
      RCLCPP_WARN(this->get_logger(), "MIT tune reset to defaults from parameters.");
    }

    auto apply_scoped = [&](double & front, double & rear, double v) {
      if (scope == TuneScope::ALL) {
        front = v;
        rear = v;
      } else if (scope == TuneScope::FRONT) {
        front = v;
      } else {
        rear = v;
      }
    };

    if (has_kp) {
      apply_scoped(runtime_kp_can0_, runtime_kp_can1_, new_kp);
    }
    if (has_kd) {
      apply_scoped(runtime_kd_can0_, runtime_kd_can1_, new_kd);
    }
    if (has_tau) {
      apply_scoped(runtime_tau_can0_, runtime_tau_can1_, new_tau);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "MIT tune applied: raw='%s' | can0(kp=%.2f,kd=%.2f,tau=%.2f) can1(kp=%.2f,kd=%.2f,tau=%.2f)",
      msg->data.c_str(),
      runtime_kp_can0_, runtime_kd_can0_, runtime_tau_can0_,
      runtime_kp_can1_, runtime_kd_can1_, runtime_tau_can1_);
  }

  double kp_{30.0};
  double kd_{1.5};
  double default_velocity_{0.0};
  double default_tau_ff_{0.0};
  std::string runtime_tune_topic_;
  bool publish_feedback_joint_states_{true};
  std::string feedback_joint_states_topic_;
  bool use_joint_cmd_limits_{true};
  std::vector<double> joint_cmd_min_rad_;
  std::vector<double> joint_cmd_max_rad_;
  bool enable_startup_smoothing_{true};
  double startup_smoothing_duration_s_{1.5};
  double command_max_velocity_rad_s_{6.0};
  bool enable_rx_decode_log_{true};
  bool prefer_joint_name_mapping_{true};
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_rad_;
  bool publish_mit_mapped_{false};
  std::string mit_mapped_topic_;
  rclcpp::Publisher<JointState>::SharedPtr mapped_positions_pub_;
  std::array<double, 256> last_commanded_mit_rad_{};
  std::array<double, 12> last_feedback_champ_rad_{};
  std::array<double, 12> last_sent_champ_rad_{};
  std::array<double, 12> boot_feedback_champ_rad_{};
  std::array<bool, 12> boot_feedback_captured_{};
  std::array<bool, 12> has_last_sent_{};
  rclcpp::Time first_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  size_t error_sample_count_{0};
  double error_abs_sum_{0.0};
  double error_abs_max_{0.0};
  double runtime_kp_can0_{30.0};
  double runtime_kd_can0_{1.5};
  double runtime_tau_can0_{0.0};
  double runtime_kp_can1_{30.0};
  double runtime_kd_can1_{1.5};
  double runtime_tau_can1_{0.0};

  rclcpp::Subscription<JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<String>::SharedPtr tune_sub_;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<JointState>::SharedPtr feedback_joint_states_pub_;
};

}  // namespace trotbot_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trotbot_can_bridge::MotorProtocolNode>());
  rclcpp::shutdown();
  return 0;
}
