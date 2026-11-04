#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trotbot_can_bridge/dog_mapper.hpp"

namespace trotbot_dynamics
{

class PinocchioTauFfNode : public rclcpp::Node
{
public:
  PinocchioTauFfNode()
  : Node("pinocchio_tau_ff_node")
  {
    robot_description_ = this->declare_parameter<std::string>("robot_description", "");
    urdf_file_ = this->declare_parameter<std::string>("urdf_file", "");
    joint_states_topic_ = this->declare_parameter<std::string>(
      "joint_states_topic", "/joint_states_feedback");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/model_tau_ff");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 50.0);
    input_timeout_s_ = this->declare_parameter<double>("input_timeout_s", 0.2);

    if (publish_hz_ <= 1e-3) {
      RCLCPP_WARN(this->get_logger(), "publish_hz %.3f invalid, fallback to 50 Hz", publish_hz_);
      publish_hz_ = 50.0;
    }
    if (input_timeout_s_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "input_timeout_s %.3f invalid, fallback to 0.2 s", input_timeout_s_);
      input_timeout_s_ = 0.2;
    }

    joint_names_.assign(
      trotbot_can_bridge::DogMapper::kChampJointNames.begin(),
      trotbot_can_bridge::DogMapper::kChampJointNames.end());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      name_to_slot_[joint_names_[i]] = i;
    }

    LoadModel();

    const double nan = std::numeric_limits<double>::quiet_NaN();
    latest_position_.fill(nan);

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PinocchioTauFfNode::OnJointState, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      output_topic_, rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = this->create_wall_timer(period, std::bind(&PinocchioTauFfNode::OnTimer, this));
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&PinocchioTauFfNode::LogStats, this));

    RCLCPP_INFO(
      this->get_logger(),
      "pinocchio_tau_ff: input=%s output=%s publish_hz=%.2f input_timeout=%.2fs",
      joint_states_topic_.c_str(), output_topic_.c_str(), publish_hz_, input_timeout_s_);
  }

private:
  void LoadModel()
  {
    bool loaded = false;
    if (!robot_description_.empty()) {
      try {
        pinocchio::urdf::buildModelFromXML(robot_description_, model_);
        loaded = true;
        RCLCPP_INFO(
          this->get_logger(), "Pinocchio model loaded from robot_description (%zu bytes)",
          robot_description_.size());
      } catch (const std::exception & e) {
        RCLCPP_FATAL(this->get_logger(), "buildModelFromXML failed: %s", e.what());
      }
    } else if (!urdf_file_.empty()) {
      try {
        pinocchio::urdf::buildModel(urdf_file_, model_);
        loaded = true;
        RCLCPP_INFO(
          this->get_logger(), "Pinocchio model loaded from urdf_file: %s", urdf_file_.c_str());
      } catch (const std::exception & e) {
        RCLCPP_FATAL(this->get_logger(), "buildModel('%s') failed: %s", urdf_file_.c_str(), e.what());
      }
    } else {
      RCLCPP_FATAL(
        this->get_logger(), "neither robot_description nor urdf_file is provided");
    }

    if (!loaded) {
      throw std::runtime_error("pinocchio_tau_ff: model load failed");
    }
    if (model_.nq != 12 || model_.nv != 12) {
      RCLCPP_FATAL(
        this->get_logger(),
        "unexpected model size: nq=%d nv=%d (expected fixed-base 12/12)", model_.nq, model_.nv);
      throw std::runtime_error("pinocchio_tau_ff: model is not a 12-DoF fixed-base model");
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string & name = joint_names_[i];
      if (!model_.existJointName(name)) {
        RCLCPP_FATAL(this->get_logger(), "joint '%s' not found in Pinocchio model", name.c_str());
        throw std::runtime_error("pinocchio_tau_ff: missing joint " + name);
      }
      const auto jid = model_.getJointId(name);
      const auto & joint = model_.joints[jid];
      if (joint.nq() != 1 || joint.nv() != 1) {
        RCLCPP_FATAL(
          this->get_logger(), "joint '%s' has nq=%d nv=%d, expected 1/1", name.c_str(),
          joint.nq(), joint.nv());
        throw std::runtime_error("pinocchio_tau_ff: unsupported joint " + name);
      }
      q_index_[i] = joint.idx_q();
      v_index_[i] = joint.idx_v();
    }

    data_ = std::make_unique<pinocchio::Data>(model_);
    q_ = Eigen::VectorXd::Zero(model_.nq);

    RCLCPP_INFO(
      this->get_logger(),
      "Pinocchio fixed-base model ready: nq=%d nv=%d njoints=%d; head joint q index for '%s'=%d",
      model_.nq, model_.nv, model_.njoints, joint_names_[0].c_str(), q_index_[0]);
  }

  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, 12> positions;
    positions.fill(std::numeric_limits<double>::quiet_NaN());
    std::array<bool, 12> got{};
    size_t matched = 0;

    const size_t count = std::min(msg->name.size(), msg->position.size());
    for (size_t k = 0; k < count; ++k) {
      const auto it = name_to_slot_.find(msg->name[k]);
      if (it == name_to_slot_.end()) {
        continue;
      }
      const double p = msg->position[k];
      if (!std::isfinite(p)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "non-finite position for joint '%s', ignore frame", msg->name[k].c_str());
        return;
      }
      const size_t slot = it->second;
      positions[slot] = p;
      if (!got[slot]) {
        got[slot] = true;
        ++matched;
      }
    }

    if (matched != 12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint_states_feedback missing joints (%zu/12), ignore frame", matched);
      return;
    }

    latest_position_ = positions;
    last_input_stamp_ = this->now();
    has_input_ = true;
  }

  void OnTimer()
  {
    if (!has_input_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for %s ...", joint_states_topic_.c_str());
      return;
    }

    const double age = (this->now() - last_input_stamp_).seconds();
    if (age > input_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint states stale (%.3fs > %.3fs), skip publishing %s",
        age, input_timeout_s_, output_topic_.c_str());
      return;
    }

    for (size_t i = 0; i < latest_position_.size(); ++i) {
      q_[q_index_[i]] = latest_position_[i];
    }

    const auto t0 = std::chrono::steady_clock::now();
    pinocchio::computeGeneralizedGravity(model_, *data_, q_);
    const auto t1 = std::chrono::steady_clock::now();
    last_compute_us_ =
      std::chrono::duration<double, std::micro>(t1 - t0).count();

    sensor_msgs::msg::JointState out;
    out.header.stamp = this->now();
    out.name = joint_names_;
    out.effort.resize(joint_names_.size());
    double max_abs = 0.0;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double tau = data_->g[static_cast<Eigen::Index>(v_index_[i])];
      out.effort[i] = tau;
      max_abs = std::max(max_abs, std::fabs(tau));
    }
    pub_->publish(out);
    ++publish_count_;
    last_max_abs_tau_ = max_abs;
    last_input_age_s_ = age;
  }

  void LogStats()
  {
    if (!has_input_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "pinocchio_tau_ff: no joint states received yet");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "pinocchio_tau_ff: published=%zu age=%.3fs max|g|=%.4f Nm compute=%.1fus",
      publish_count_, last_input_age_s_, last_max_abs_tau_, last_compute_us_);
  }

  std::string robot_description_;
  std::string urdf_file_;
  std::string joint_states_topic_;
  std::string output_topic_;
  double publish_hz_{50.0};
  double input_timeout_s_{0.2};

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> name_to_slot_;
  std::array<int, 12> q_index_{};
  std::array<int, 12> v_index_{};

  std::array<double, 12> latest_position_{};
  bool has_input_{false};
  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};

  size_t publish_count_{0};
  double last_max_abs_tau_{0.0};
  double last_input_age_s_{0.0};
  double last_compute_us_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace trotbot_dynamics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trotbot_dynamics::PinocchioTauFfNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pinocchio_tau_ff_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
