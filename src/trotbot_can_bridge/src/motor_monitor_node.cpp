#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

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
    decimals_ = this->declare_parameter<int>("decimals", 4);
    motor_ids_ = this->declare_parameter<std::vector<int64_t>>(
      "motor_ids", std::vector<int64_t>{});

    if (output_mode_ != "torque" && output_mode_ != "position" &&
      output_mode_ != "both" && output_mode_ != "speed" &&
      output_mode_ != "temp" && output_mode_ != "speed_temp" &&
      output_mode_ != "all")
    {
      RCLCPP_WARN(
        this->get_logger(),
        "unknown output_mode='%s', fallback to 'torque' "
        "(torque|position|both|speed|temp|speed_temp|all)",
        output_mode_.c_str());
      output_mode_ = "torque";
    }
    decimals_ = std::max(0, std::min(6, decimals_));

    for (size_t i = 0; i < DogMapper::kChampJointNames.size(); ++i) {
      joint_names_[i] = DogMapper::kChampJointNames[i];
    }
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      motor_to_index_[route.motor_id] = route.trajectory_index;
    }
    BuildDisplayIndices();

    const double nan = std::numeric_limits<double>::quiet_NaN();
    position_.fill(nan);
    speed_.fill(nan);
    torque_.fill(nan);
    temp_.fill(nan);
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

    std::ostringstream ids;
    for (size_t k = 0; k < display_indices_.size(); ++k) {
      if (k) {
        ids << ",";
      }
      ids << static_cast<int>(DogMapper::kTemporaryIndexMap[display_indices_[k]].motor_id);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "motor_monitor: feedback=%s output_mode=%s print_hz=%.2f decimals=%d "
      "show_max_torque=%s motors=[%s]",
      feedback_topic_.c_str(), output_mode_.c_str(), hz, decimals_,
      show_max_torque_ ? "true" : "false", ids.str().c_str());
  }

private:
  void BuildDisplayIndices()
  {
    display_indices_.clear();
    if (motor_ids_.empty()) {
      for (size_t i = 0; i < 12; ++i) {
        display_indices_.push_back(i);
      }
      return;
    }

    std::array<bool, 12> used{};
    for (const int64_t id64 : motor_ids_) {
      const int id = static_cast<int>(id64);
      const auto it = motor_to_index_.find(id);
      if (it == motor_to_index_.end()) {
        RCLCPP_WARN(this->get_logger(), "ignore unknown motor_id=%d", id);
        continue;
      }
      if (!used[it->second]) {
        used[it->second] = true;
        display_indices_.push_back(it->second);
      }
    }
    std::sort(display_indices_.begin(), display_indices_.end());
    if (display_indices_.empty()) {
      RCLCPP_WARN(this->get_logger(), "motor_ids empty after validation, fallback to all 12");
      for (size_t i = 0; i < 12; ++i) {
        display_indices_.push_back(i);
      }
    }
  }

  void OnFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    int motor_id = -1;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double angle = nan;
    double speed = nan;
    double torque = nan;
    double temp = nan;

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
      } else if (key == "speed") {
        speed = std::atof(value.c_str());
      } else if (key == "torque") {
        torque = std::atof(value.c_str());
      } else if (key == "temp") {
        temp = std::atof(value.c_str());
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
    if (std::isfinite(speed)) {
      speed_[idx] = speed;
    }
    if (std::isfinite(temp)) {
      temp_[idx] = temp;
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

  std::string FormatValue(double value, bool show_sign = true) const
  {
    if (!std::isfinite(value)) {
      return "n/a";
    }
    if (std::fabs(value) < 0.5 * std::pow(10.0, -decimals_)) {
      value = 0.0;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), show_sign ? "%+.*f" : "%.*f", decimals_, value);
    return std::string(buf);
  }

  void PrintTable()
  {
    const bool show_position =
      output_mode_ == "position" || output_mode_ == "both" || output_mode_ == "all";
    const bool show_speed =
      output_mode_ == "speed" || output_mode_ == "speed_temp" || output_mode_ == "all";
    const bool show_torque =
      output_mode_ == "torque" || output_mode_ == "both" || output_mode_ == "all";
    const bool show_temp =
      output_mode_ == "temp" || output_mode_ == "speed_temp" || output_mode_ == "all";

    size_t seen_count = 0;
    for (const size_t i : display_indices_) {
      if (seen_[i]) {
        ++seen_count;
      }
    }

    char name_buf[32];
    std::ostringstream os;
    os << "\n[motor_monitor] mode=" << output_mode_ << " topic=" << feedback_topic_
       << " seen=" << seen_count << "/" << display_indices_.size()
       << " elapsed=" << std::fixed << std::setprecision(1)
       << (this->now() - start_time_).seconds() << "s\n";
    os << std::setw(4) << "idx" << "  " << std::left << std::setw(12) << "joint"
       << std::right << std::setw(6) << "motor";
    if (show_position) {
      os << std::setw(15) << "position(rad)";
    }
    if (show_speed) {
      os << std::setw(14) << "speed(rad/s)";
    }
    if (show_torque) {
      os << std::setw(14) << "torque(Nm)";
    }
    if (show_temp) {
      os << std::setw(10) << "temp(C)";
    }
    if (show_max_torque_ && show_torque) {
      os << std::setw(15) << "peak|tau|(Nm)";
    }
    os << "\n";

    for (const size_t i : display_indices_) {
      std::snprintf(name_buf, sizeof(name_buf), "%-12s", joint_names_[i].c_str());
      os << std::setw(4) << i << "  " << name_buf;
      os << std::setw(6) << static_cast<int>(DogMapper::kTemporaryIndexMap[i].motor_id);
      if (show_position) {
        os << std::setw(15) << FormatValue(position_[i]);
      }
      if (show_speed) {
        os << std::setw(14) << FormatValue(speed_[i]);
      }
      if (show_torque) {
        os << std::setw(14) << FormatValue(torque_[i]);
      }
      if (show_temp) {
        os << std::setw(10) << FormatValue(temp_[i], false);
      }
      if (show_max_torque_ && show_torque) {
        os << std::setw(15) << FormatValue(peak_signed_torque_[i]);
      }
      os << "\n";
    }
    std::cout << os.str() << std::flush;
  }

  std::string feedback_topic_;
  std::string output_mode_;
  double print_hz_{1.0};
  bool show_max_torque_{true};
  int decimals_{4};
  std::vector<int64_t> motor_ids_;

  std::array<std::string, 12> joint_names_{};
  std::unordered_map<int, size_t> motor_to_index_;
  std::vector<size_t> display_indices_;
  std::array<double, 12> position_{};
  std::array<double, 12> speed_{};
  std::array<double, 12> torque_{};
  std::array<double, 12> temp_{};
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
