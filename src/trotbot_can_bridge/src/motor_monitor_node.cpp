#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trotbot_can_bridge/dog_mapper.hpp"

namespace trotbot_can_bridge
{

class MotorMonitorNode : public rclcpp::Node
{
public:
  MotorMonitorNode()
  : Node("motor_monitor_node")
  {
    feedback_topic_ = this->declare_parameter<std::string>("feedback_topic", "/motor_feedback");
    output_mode_ = this->declare_parameter<std::string>("output_mode", "torque");
    print_hz_ = this->declare_parameter<double>("print_hz", 1.0);
    show_max_torque_ = this->declare_parameter<bool>("show_max_torque", true);

    if (output_mode_ != "torque" && output_mode_ != "position" && output_mode_ != "both") {
      RCLCPP_WARN(
        this->get_logger(), "unknown output_mode='%s', fallback to 'torque'",
        output_mode_.c_str());
      output_mode_ = "torque";
    }

    for (size_t i = 0; i < DogMapper::kChampJointNames.size(); ++i) {
      joint_names_[i] = DogMapper::kChampJointNames[i];
    }
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      motor_to_index_[route.motor_id] = route.trajectory_index;
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    position_.fill(nan);
    torque_.fill(nan);
    peak_abs_torque_.fill(nan);
    peak_signed_torque_.fill(nan);
    seen_.fill(false);

    sub_ = this->create_subscription<std_msgs::msg::String>(
      feedback_topic_, 50,
      std::bind(&MotorMonitorNode::OnFeedback, this, std::placeholders::_1));

    start_time_ = this->now();
    const double hz = print_hz_ > 1e-3 ? print_hz_ : 1.0;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&MotorMonitorNode::PrintTable, this));

    RCLCPP_INFO(
      this->get_logger(),
      "motor_monitor: feedback=%s output_mode=%s print_hz=%.2f show_max_torque=%s",
      feedback_topic_.c_str(), output_mode_.c_str(), hz, show_max_torque_ ? "true" : "false");
  }

private:
  void OnFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    int motor_id = -1;
    double angle = std::numeric_limits<double>::quiet_NaN();
    double torque = std::numeric_limits<double>::quiet_NaN();

    std::istringstream iss(msg->data);
    std::string token;
    while (iss >> token) {
      const auto eq = token.find('=');
      if (eq == std::string::npos) {
        continue;
      }
      const std::string key = token.substr(0, eq);
      const std::string value = token.substr(eq + 1);
      if (key == "motor") {
        motor_id = std::atoi(value.c_str());
      } else if (key == "angle") {
        angle = std::atof(value.c_str());
      } else if (key == "torque") {
        torque = std::atof(value.c_str());
      }
    }

    const auto it = motor_to_index_.find(motor_id);
    if (it == motor_to_index_.end()) {
      return;
    }
    const size_t idx = std::min(it->second, static_cast<size_t>(11));

    if (std::isfinite(angle)) {
      position_[idx] = angle;
    }
    if (std::isfinite(torque)) {
      torque_[idx] = torque;
      const double abs_t = std::fabs(torque);
      if (!std::isfinite(peak_abs_torque_[idx]) || abs_t > peak_abs_torque_[idx]) {
        peak_abs_torque_[idx] = abs_t;
        peak_signed_torque_[idx] = torque;
      }
    }
    seen_[idx] = true;
  }

  static std::string FormatValue(double value, const char * fmt)
  {
    if (!std::isfinite(value)) {
      return "n/a";
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), fmt, value);
    return std::string(buf);
  }

  void PrintTable()
  {
    const bool show_position = output_mode_ == "position" || output_mode_ == "both";
    const bool show_torque = output_mode_ == "torque" || output_mode_ == "both";
    const size_t seen_count = static_cast<size_t>(std::count(seen_.begin(), seen_.end(), true));

    char name_buf[32];
    std::ostringstream os;
    os << "\n[motor_monitor] mode=" << output_mode_ << " topic=" << feedback_topic_
       << " seen=" << seen_count << "/12 elapsed=" << std::fixed << std::setprecision(1)
       << (this->now() - start_time_).seconds() << "s\n";
    os << std::setw(4) << "idx" << "  " << std::left << std::setw(12) << "joint"
       << std::right << std::setw(6) << "motor";
    if (show_position) {
      os << std::setw(15) << "position(rad)";
    }
    if (show_torque) {
      os << std::setw(14) << "torque(Nm)";
    }
    if (show_max_torque_) {
      os << std::setw(15) << "peak|tau|(Nm)";
    }
    os << "\n";

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      std::snprintf(name_buf, sizeof(name_buf), "%-12s", joint_names_[i].c_str());
      os << std::setw(4) << i << "  " << name_buf;
      os << std::setw(6) << static_cast<int>(DogMapper::kTemporaryIndexMap[i].motor_id);
      if (show_position) {
        os << std::setw(15) << FormatValue(position_[i], "%+.4f");
      }
      if (show_torque) {
        os << std::setw(14) << FormatValue(torque_[i], "%+.4f");
      }
      if (show_max_torque_) {
        os << std::setw(15) << FormatValue(peak_signed_torque_[i], "%+.4f");
      }
      os << "\n";
    }
    std::cout << os.str() << std::flush;
  }

  std::string feedback_topic_;
  std::string output_mode_;
  double print_hz_{1.0};
  bool show_max_torque_{true};

  std::array<std::string, 12> joint_names_{};
  std::unordered_map<int, size_t> motor_to_index_;
  std::array<double, 12> position_{};
  std::array<double, 12> torque_{};
  std::array<double, 12> peak_abs_torque_{};
  std::array<double, 12> peak_signed_torque_{};
  std::array<bool, 12> seen_{};

  rclcpp::Time start_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace trotbot_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trotbot_can_bridge::MotorMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
