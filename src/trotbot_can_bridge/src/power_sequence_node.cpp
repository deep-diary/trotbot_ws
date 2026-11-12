#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "trotbot_can_bridge/dog_mapper.hpp"
#include "trotbot_can_bridge/frame_codec.hpp"
#include "trotbot_can_bridge/protocol_codec.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Joy;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using std_msgs::msg::UInt8MultiArray;

namespace trotbot_can_bridge
{
namespace
{
uint16_t HzToEpScanTime(double hz)
{
  if (hz <= 0.0) {
    hz = 5.0;
  }
  const double period_ms = 1000.0 / hz;
  int n = static_cast<int>(std::lround((period_ms - 10.0) / 5.0 + 1.0));
  n = std::max(1, std::min(n, 65535));
  return static_cast<uint16_t>(n);
}
}  // namespace

enum class SequenceState
{
  Idle = 0,
  Precheck = 1,
  EnableInit = 2,
  SoftStand = 3,
  Running = 4,
  SoftProne = 5,
  ProneHold = 6,
  Disable = 7,
  Fault = 8,
  PoseHold = 9,
  PoseMove = 10,
};

/// 命名姿态目标（PoseMove 的终点）
enum class NamedPose
{
  None = 0,
  Idle = 1,
  Home = 2,
};

const char * NamedPoseName(NamedPose p)
{
  switch (p) {
    case NamedPose::Idle: return "idle";
    case NamedPose::Home: return "home";
    default: return "none";
  }
}

class PowerSequenceNode : public rclcpp::Node
{
public:
  PowerSequenceNode()
  : Node("power_sequence_node")
  {
    gate_topic_ = this->declare_parameter<std::string>("gate_topic", "/power_sequence/gate_open");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/power_sequence/state");
    tx_topic_ = this->declare_parameter<std::string>("tx_topic", "/can_tx_frames");
    rx_topic_ = this->declare_parameter<std::string>("rx_topic", "/can_rx_frames");
    body_pose_topic_ = this->declare_parameter<std::string>("body_pose_topic", "body_pose");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/power_sequence/command");

    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 20);
    precheck_duration_s_ = this->declare_parameter<double>("precheck_duration_s", 0.20);
    init_hold_duration_s_ = this->declare_parameter<double>("init_hold_duration_s", 0.35);
    stand_duration_s_ = this->declare_parameter<double>("stand_duration_s", 2.50);
    prone_duration_s_ = this->declare_parameter<double>("prone_duration_s", 2.50);

    stand_z_ = this->declare_parameter<double>("stand_z", 0.0);
    prone_z_ = this->declare_parameter<double>("prone_z", -0.35);

    button_l1_ = this->declare_parameter<int>("button_l1", 4);
    button_r1_ = this->declare_parameter<int>("button_r1", 5);
    button_share_ = this->declare_parameter<int>("button_share", 8);
    button_circle_ = this->declare_parameter<int>("button_circle", 1);
    button_square_ = this->declare_parameter<int>("button_square", 3);
    button_l3_ = this->declare_parameter<int>("button_l3", 11);

    motor_master_id_ = this->declare_parameter<int>("motor_master_id", 253);
    init_kp_ = this->declare_parameter<double>("init_kp", 20.0);
    init_kd_ = this->declare_parameter<double>("init_kd", 1.5);
    init_velocity_ = this->declare_parameter<double>("init_velocity", 0.0);
    init_tau_ = this->declare_parameter<double>("init_tau", 0.0);
    send_enable_in_startup_ = this->declare_parameter<bool>("send_enable_in_startup", true);
    send_mit_hold_in_startup_ = this->declare_parameter<bool>("send_mit_hold_in_startup", true);
    require_feedback_before_enable_ = this->declare_parameter<bool>("require_feedback_before_enable", true);
    motor_feedback_fresh_timeout_s_ =
      this->declare_parameter<double>("motor_feedback_fresh_timeout_s", 1.0);

    enable_thermal_protection_ = this->declare_parameter<bool>("enable_thermal_protection", true);
    thermal_limit_c_ = this->declare_parameter<double>("thermal_limit_c", 100.0);
    thermal_recover_c_ = this->declare_parameter<double>("thermal_recover_c", 80.0);
    thermal_debounce_s_ = this->declare_parameter<double>("thermal_debounce_s", 0.5);
    thermal_fresh_timeout_s_ = this->declare_parameter<double>("thermal_fresh_timeout_s", 2.0);
    if (thermal_recover_c_ >= thermal_limit_c_) {
      RCLCPP_WARN(
        this->get_logger(), "thermal_recover_c=%.1f >= thermal_limit_c=%.1f invalid, fallback to %.1f",
        thermal_recover_c_, thermal_limit_c_, thermal_limit_c_ * 0.8);
      thermal_recover_c_ = thermal_limit_c_ * 0.8;
    }

    enable_active_report_in_startup_ = this->declare_parameter<bool>("enable_active_report_in_startup", true);
    active_report_hz_ = this->declare_parameter<double>("active_report_hz", 10.0);
    active_report_master_id_ = this->declare_parameter<int>("active_report_master_id", -1);
    enable_active_report_at_launch_ = this->declare_parameter<bool>("enable_active_report_at_launch", true);
    active_report_boot_delay_s_ = this->declare_parameter<double>("active_report_boot_delay_s", 0.25);

    enable_remote_set_zero_ = this->declare_parameter<bool>("enable_remote_set_zero", true);
    button_r3_ = this->declare_parameter<int>("button_r3", 12);
    set_zero_event_topic_ = this->declare_parameter<std::string>("set_zero_event_topic", "");
    set_zero_repeat_count_ = this->declare_parameter<int>("set_zero_repeat_count", 3);
    set_zero_repeat_interval_ms_ = this->declare_parameter<int>("set_zero_repeat_interval_ms", 40);

    enable_remote_motor_init_ = this->declare_parameter<bool>("enable_remote_motor_init", true);
    motor_init_longpress_s_ = this->declare_parameter<double>("motor_init_longpress_s", 3.0);
    motor_init_do_set_zero_ = this->declare_parameter<bool>("motor_init_do_set_zero", true);
    motor_init_zero_sta_ = this->declare_parameter<int>("motor_init_zero_sta", 1);
    motor_init_tick_ms_ = this->declare_parameter<int>("motor_init_tick_ms", 50);
    motor_init_set_zero_wait_ms_ = this->declare_parameter<int>("motor_init_set_zero_wait_ms", 400);
    motor_init_set_wait_ms_ = this->declare_parameter<int>("motor_init_set_wait_ms", 200);
    motor_init_save_wait_ms_ = this->declare_parameter<int>("motor_init_save_wait_ms", 500);
    motor_init_read_wait_ms_ = this->declare_parameter<int>("motor_init_read_wait_ms", 600);
    motor_init_event_topic_ = this->declare_parameter<std::string>("motor_init_event_topic", "");
    verify_zero_sta_at_launch_ = this->declare_parameter<bool>("verify_zero_sta_at_launch", true);
    zero_sta_verify_delay_s_ = this->declare_parameter<double>("zero_sta_verify_delay_s", 1.0);
    if (motor_init_zero_sta_ != 0 && motor_init_zero_sta_ != 1) {
      RCLCPP_WARN(
        this->get_logger(), "motor_init_zero_sta=%d invalid, fallback to 1", motor_init_zero_sta_);
      motor_init_zero_sta_ = 1;
    }
    motor_init_readback_.fill(-1);
    motor_init_has_readback_.fill(false);

    track_body_pose_height_ = this->declare_parameter<bool>("track_body_pose_height", true);
    body_pose_stale_s_ = this->declare_parameter<double>("body_pose_stale_s", 2.0);
    track_z_clamp_min_ = this->declare_parameter<double>("track_z_clamp_min", -0.5);
    track_z_clamp_max_ = this->declare_parameter<double>("track_z_clamp_max", 0.5);

    // --- 命名姿态（idle/home）与关节空间切换 ---
    enable_named_poses_ = this->declare_parameter<bool>("enable_named_poses", true);
    named_poses_file_ = this->declare_parameter<std::string>("named_poses_file", "");
    pose_move_duration_s_ = this->declare_parameter<double>("pose_move_duration_s", 2.5);
    pose_max_joint_speed_rad_s_ =
      this->declare_parameter<double>("pose_max_joint_speed_rad_s", 1.2);
    pose_servo_active_topic_ = this->declare_parameter<std::string>(
      "pose_servo_active_topic", "/pose_servo/active");
    pose_servo_joint_topic_ = this->declare_parameter<std::string>(
      "pose_servo_joint_topic", "/pose_servo/joint_targets");
    enable_stand_after_home_ = this->declare_parameter<bool>("enable_stand_after_home", true);
    pose_servo_blend_s_ = this->declare_parameter<double>("pose_servo_blend_s", 0.30);
    feedback_joint_states_topic_ = this->declare_parameter<std::string>(
      "feedback_joint_states_topic", "/joint_states_feedback");
    if (pose_servo_blend_s_ < 1e-3) {
      pose_servo_blend_s_ = 0.30;
    }
    if (pose_max_joint_speed_rad_s_ < 1e-3) {
      pose_max_joint_speed_rad_s_ = 1.2;
    }
    if (pose_move_duration_s_ < 1e-3) {
      pose_move_duration_s_ = 2.5;
    }
    idle_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    home_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hold_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    pose_q0_.fill(std::numeric_limits<double>::quiet_NaN());
    pose_q1_.fill(std::numeric_limits<double>::quiet_NaN());
    if (enable_named_poses_) {
      LoadNamedPoses();
      if (!poses_loaded_) {
        RCLCPP_WARN(
          this->get_logger(),
          "named poses unavailable, fallback to legacy SoftStand/SoftProne behavior");
        enable_named_poses_ = false;
      }
    }

    const int can_tx_depth = this->declare_parameter<int>("can_tx_frames_qos_depth", 4000);
    rclcpp::QoS tx_qos(static_cast<size_t>(std::max(1, can_tx_depth)));
    tx_qos.reliable();
    tx_pub_ = this->create_publisher<UInt8MultiArray>(tx_topic_, tx_qos);
    pose_pub_ = this->create_publisher<Pose>(body_pose_topic_, 10);
    cmd_vel_pub_ = this->create_publisher<Twist>(cmd_vel_topic_, 10);
    gate_pub_ = this->create_publisher<Bool>(gate_topic_, rclcpp::QoS(1).reliable().transient_local());
    if (enable_named_poses_) {
      pose_servo_active_pub_ = this->create_publisher<Bool>(
        pose_servo_active_topic_, rclcpp::QoS(1).reliable().transient_local());
      pose_servo_joint_pub_ = this->create_publisher<JointState>(pose_servo_joint_topic_, 10);
      PublishPoseServoActive(false);
    }
    // 与 status_led 等 transient_local 订阅对齐：晚启动节点仍可收到最后一帧状态（否则易卡在 Idle 灯语）。
    state_pub_ = this->create_publisher<String>(state_topic_, rclcpp::QoS(1).reliable().transient_local());
    if (!set_zero_event_topic_.empty()) {
      event_pub_ = this->create_publisher<String>(set_zero_event_topic_, 10);
    }
    if (!motor_init_event_topic_.empty()) {
      motor_init_event_pub_ = this->create_publisher<String>(motor_init_event_topic_, 10);
    }
    joy_sub_ = this->create_subscription<Joy>(joy_topic_, 10, std::bind(&PowerSequenceNode::OnJoy, this, std::placeholders::_1));
    command_sub_ = this->create_subscription<String>(
      command_topic_, 10, std::bind(&PowerSequenceNode::OnCommand, this, std::placeholders::_1));
    rx_sub_ = this->create_subscription<UInt8MultiArray>(
      rx_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PowerSequenceNode::OnCanRxFrame, this, std::placeholders::_1));
    if (track_body_pose_height_) {
      body_pose_sub_ = this->create_subscription<Pose>(
        body_pose_topic_, rclcpp::QoS(10),
        std::bind(&PowerSequenceNode::OnBodyPose, this, std::placeholders::_1));
    }
    if (enable_named_poses_) {
      feedback_js_sub_ = this->create_subscription<JointState>(
        feedback_joint_states_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PowerSequenceNode::OnFeedbackJointStates, this, std::placeholders::_1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(5, timer_period_ms_)),
      std::bind(&PowerSequenceNode::OnTimer, this));

    motor_init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(10, motor_init_tick_ms_)),
      std::bind(&PowerSequenceNode::TickMotorInit, this));

    if (verify_zero_sta_at_launch_) {
      const int verify_ms = std::max(
        200, static_cast<int>(std::lround(zero_sta_verify_delay_s_ * 1000.0)));
      verify_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(verify_ms),
        std::bind(&PowerSequenceNode::OnVerifyZeroStaTimer, this));
    }

    if (enable_active_report_in_startup_ && enable_active_report_at_launch_) {
      const int boot_ms = std::max(0, static_cast<int>(std::lround(active_report_boot_delay_s_ * 1000.0)));
      boot_report_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(boot_ms),
        std::bind(&PowerSequenceNode::OnBootActiveReportOnce, this));
    }

    PublishGate();
    PublishState();
    RCLCPP_INFO(this->get_logger(), "Power sequence node started: state=Idle gate=closed");
  }

private:
  /// 一次性电机初始化（只在 Idle + 门禁关时执行）：set_zero → 写 zero_sta → 保存 flash → 回读校验
  enum class MotorInitPhase { Idle, SetZero, SetZeroSta, SaveParams, ReadBack };

  static constexpr uint8_t kCmdEnable = 0x03;
  static constexpr uint8_t kCmdReset = 0x04;
  static constexpr uint8_t kCmdSetZero = 0x06;
  static constexpr uint8_t kCmdSetParam = 18;
  /// 手册通信类型 24，CAN ID 高 5 位为 0x18
  static constexpr uint8_t kCommActiveReport = 24;
  static constexpr uint16_t kParamEpScanTime = 0x7026;

  void OnBootActiveReportOnce()
  {
    if (boot_active_report_done_) {
      return;
    }
    boot_active_report_done_ = true;
    if (boot_report_timer_) {
      boot_report_timer_->cancel();
    }
    const uint16_t scan = HzToEpScanTime(active_report_hz_);
    SendEpScanTimeAll(scan);
    SendActiveReportAll(true);
    RCLCPP_INFO(
      this->get_logger(),
      "active report at launch (Idle): EPScan_time=%u (~%.2f Hz), COMM 0x18 ON — before start, for RViz /joint_states_feedback",
      static_cast<unsigned>(scan), active_report_hz_);
  }

  void OnJoy(const Joy::SharedPtr msg)
  {
    latest_joy_ = *msg;
    has_joy_ = true;
  }

  void OnCanRxFrame(const UInt8MultiArray::SharedPtr msg)
  {
    const auto frame = FrameCodec::Unpack(*msg);
    if (!frame.has_value()) {
      return;
    }

    // 1) 缓存电机反馈角（MIT 域 rad）与温度：供上电「按当前反馈保持」、反馈新鲜度校验与过温保护
    if (const auto fb = ProtocolCodec::DecodeFeedback(frame.value()); fb.has_value()) {
      if (static_cast<size_t>(fb->motor_id) < last_feedback_angle_rad_.size()) {
        last_feedback_angle_rad_[fb->motor_id] = fb->current_angle;
        last_feedback_temp_c_[fb->motor_id] = fb->current_temp;
        last_feedback_stamp_ns_[fb->motor_id] = this->now().nanoseconds();
      }
    }

    // 2) 初始化/自检的回读阶段：收集 0x7029 应答
    if (motor_init_active_ && motor_init_phase_ == MotorInitPhase::ReadBack) {
      if (const auto rsp = ProtocolCodec::DecodeGetParamResponse(
          frame.value(), static_cast<uint8_t>(motor_master_id_));
        rsp.has_value() && rsp->param_id == ProtocolCodec::kParamZeroSta &&
        static_cast<size_t>(rsp->motor_id) < motor_init_readback_.size())
      {
        motor_init_readback_[rsp->motor_id] = rsp->value_u8;
        motor_init_has_readback_[rsp->motor_id] = true;
      }
    }
  }

  static std::string ToLower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return value;
  }

  void OnCommand(const String::SharedPtr msg)
  {
    const std::string command = ToLower(msg->data);
    if (command == "motor_init") {
      (void)BeginMotorInit("command", false);
      return;
    }
    if (
      motor_init_active_ &&
      (command == "start" || command == "prone" || command == "shutdown" ||
      command == "home" || command == "idle"))
    {
      RCLCPP_WARN(
        this->get_logger(), "ignore command=%s: motor_init in progress", command.c_str());
      return;
    }
    if (command == "reload_poses") {
      if (!enable_named_poses_) {
        RCLCPP_WARN(this->get_logger(), "ignore command=reload_poses: enable_named_poses=false");
        return;
      }
      poses_loaded_ = false;
      LoadNamedPoses();
      RCLCPP_WARN(
        this->get_logger(), "command=reload_poses -> %s",
        poses_loaded_ ? "OK" : "FAILED (keep previous poses)");
      return;
    }
    if (command == "start") {
      if (thermal_latched_) {
        RCLCPP_WARN(
          this->get_logger(),
          "ignore command=start: thermal protection latched (motor=%d was %.1f C, need <= %.1f C)",
          thermal_hot_motor_id_, thermal_hot_temp_c_, thermal_recover_c_);
        return;
      }
      if (state_ == SequenceState::Idle) {
        use_stand_resume_for_softstand_ = false;
        EnterState(SequenceState::Precheck);
      } else if (state_ == SequenceState::ProneHold) {
        if (enable_named_poses_) {
          StartPoseMove(NamedPose::Home);
        } else {
          EnterState(SequenceState::SoftStand);
        }
      } else if (state_ == SequenceState::PoseHold) {
        StartPoseMove(NamedPose::Home);
      } else if (state_ == SequenceState::SoftProne && !soft_prone_then_disable_) {
        // 趴下动画未结束就发 start：原先会被忽略，导致“要发两次才站起来”
        pending_start_after_prone_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "command=start latched during SoftProne(prone path): will enter SoftStand when prone motion completes");
      } else {
        RCLCPP_WARN(this->get_logger(), "ignore command=start in state=%s", StateName(state_));
      }
      return;
    }
    if (command == "home") {
      if (thermal_latched_) {
        RCLCPP_WARN(this->get_logger(), "ignore command=home: thermal protection latched");
        return;
      }
      if (!enable_named_poses_) {
        RCLCPP_WARN(this->get_logger(), "ignore command=home: enable_named_poses=false");
        return;
      }
      if (
        state_ == SequenceState::PoseHold || state_ == SequenceState::ProneHold ||
        state_ == SequenceState::Running)
      {
        StartPoseMove(NamedPose::Home);
      } else {
        RCLCPP_WARN(this->get_logger(), "ignore command=home in state=%s", StateName(state_));
      }
      return;
    }
    if (command == "idle" || command == "prone") {
      if (!enable_named_poses_) {
        if (command == "prone" && state_ == SequenceState::Running) {
          BeginSoftProne(false);
        } else {
          RCLCPP_WARN(this->get_logger(), "ignore command=%s in state=%s",
            command.c_str(), StateName(state_));
        }
        return;
      }
      if (state_ == SequenceState::Running || state_ == SequenceState::PoseHold) {
        StartPoseMove(NamedPose::Idle);
      } else {
        RCLCPP_WARN(this->get_logger(), "ignore command=%s in state=%s",
          command.c_str(), StateName(state_));
      }
      return;
    }
    if (command == "shutdown") {
      if (state_ == SequenceState::ProneHold) {
        EnterState(SequenceState::Disable);
        return;
      }
      if (enable_named_poses_) {
        if (
          state_ == SequenceState::PoseHold || state_ == SequenceState::PoseMove ||
          state_ == SequenceState::Running)
        {
          pose_move_then_disable_ = true;
          if (state_ == SequenceState::PoseMove && pose_target_ == NamedPose::Idle) {
            return;  // 已在去 idle 的路上，到点后失能
          }
          if (!StartPoseMove(NamedPose::Idle)) {
            pose_move_then_disable_ = false;
            EnterState(SequenceState::Disable);
          }
          return;
        }
        if (
          state_ == SequenceState::Idle || state_ == SequenceState::Precheck ||
          state_ == SequenceState::EnableInit)
        {
          PublishPoseServoActive(false);
          EnterState(SequenceState::Disable);
          return;
        }
      }
      if (
        state_ == SequenceState::Idle || state_ == SequenceState::Precheck ||
        state_ == SequenceState::EnableInit || state_ == SequenceState::SoftStand ||
        state_ == SequenceState::Running || state_ == SequenceState::SoftProne)
      {
        BeginSoftProne(true);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "ignore command=shutdown in state=%s", StateName(state_));
      return;
    }
    if (command == "set_zero") {
      (void)TryRemoteSetZero("command");
      return;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "unknown command=%s (supported: start/home/idle/prone/shutdown/reload_poses/set_zero/motor_init)",
      command.c_str());
  }

  bool ButtonOn(int idx) const
  {
    if (!has_joy_ || idx < 0 || static_cast<size_t>(idx) >= latest_joy_.buttons.size()) {
      return false;
    }
    return latest_joy_.buttons[static_cast<size_t>(idx)] != 0;
  }

  void OnTimer()
  {
    const double dt = static_cast<double>(std::max(5, timer_period_ms_)) / 1000.0;

    TickMotorInit();

    const bool share = ButtonOn(button_share_);
    const bool l1 = ButtonOn(button_l1_);
    const bool r1 = ButtonOn(button_r1_);
    const bool circle = (button_circle_ >= 0) && ButtonOn(button_circle_);
    const bool square = (button_square_ >= 0) && ButtonOn(button_square_);
    const bool l3 = (button_l3_ >= 0) && ButtonOn(button_l3_);
    const bool r3 = (button_r3_ >= 0) && ButtonOn(button_r3_);

    // 短按 = 按下沿；先更新 prev，避免后续早退丢失边沿
    const bool l3_edge = l3 && !l3_prev_;
    const bool square_edge = square && !square_prev_;
    const bool circle_edge = circle && !circle_prev_;
    const bool r3_edge = r3 && !r3_prev_;
    l3_prev_ = l3;
    square_prev_ = square;
    circle_prev_ = circle;
    r3_prev_ = r3;

    // 设零唯一入口：未上电（Idle + 门禁关）时 L1+R1 长按 → set_zero + zero_sta=1 + 保存 + 回读
    const bool init_combo =
      l1 && r1 && !share && state_ == SequenceState::Idle && !gate_open_;
    init_hold_s_ = init_combo ? (init_hold_s_ + dt) : 0.0;
    if (!init_combo) {
      init_latch_ = false;
    }
    if (init_combo && !init_latch_ && init_hold_s_ >= motor_init_longpress_s_) {
      init_latch_ = true;
      (void)BeginMotorInit("joy", false);
    }

    // 初始化期间屏蔽一切运动触发（start/shutdown/prone），避免中途开门禁或使能
    if (motor_init_active_) {
      return;
    }

    // 过温保护：等效 R3，最高优先级；触发当 tick 直接接管
    if (UpdateThermalProtection(dt)) {
      return;
    }

    if (state_ == SequenceState::Idle) {
      // 上电唯一入口：L3 短按 → Precheck → EnableInit → SoftStand → Running
      if (l3_edge) {
        if (thermal_latched_) {
          RCLCPP_WARN(
            this->get_logger(),
            "joy L3 ignored: thermal protection latched (motor=%d was %.1f C, need <= %.1f C)",
            thermal_hot_motor_id_, thermal_hot_temp_c_, thermal_recover_c_);
          return;
        }
        RCLCPP_WARN(this->get_logger(), "joy L3 short press: power on / enable");
        use_stand_resume_for_softstand_ = false;
        EnterState(SequenceState::Precheck);
        return;
      }
      return;
    }

    if (state_ == SequenceState::Running) {
      if (circle_edge) {
        if (enable_named_poses_) {
          RCLCPP_WARN(this->get_logger(), "joy Circle: go to named pose idle");
          StartPoseMove(NamedPose::Idle);
        } else {
          RCLCPP_WARN(this->get_logger(), "joy Circle short press: go to idle (SoftProne)");
          BeginSoftProne(false);
        }
      } else if (r3_edge) {
        if (enable_named_poses_) {
          RCLCPP_WARN(this->get_logger(), "joy R3: go to named pose idle, then disable");
          pose_move_then_disable_ = true;
          if (!StartPoseMove(NamedPose::Idle)) {
            pose_move_then_disable_ = false;
            EnterState(SequenceState::Disable);
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(), "joy R3 short press: safe-park (SoftProne) then disable");
          BeginSoftProne(true);
        }
      }
      return;
    }

    if (state_ == SequenceState::PoseHold) {
      if (square_edge) {
        if (thermal_latched_) {
          RCLCPP_WARN(
            this->get_logger(),
            "joy Square ignored: thermal protection latched (motor=%d was %.1f C, need <= %.1f C)",
            thermal_hot_motor_id_, thermal_hot_temp_c_, thermal_recover_c_);
          TickState(dt);
          return;
        }
        RCLCPP_WARN(this->get_logger(), "joy Square: go to named pose home");
        StartPoseMove(NamedPose::Home);
      } else if (circle_edge) {
        RCLCPP_WARN(this->get_logger(), "joy Circle: go to named pose idle");
        StartPoseMove(NamedPose::Idle);
      } else if (r3_edge) {
        RCLCPP_WARN(this->get_logger(), "joy R3: go to named pose idle, then disable");
        pose_move_then_disable_ = true;
        if (!StartPoseMove(NamedPose::Idle)) {
          pose_move_then_disable_ = false;
          EnterState(SequenceState::Disable);
        }
      }
      TickState(dt);
      return;
    }

    if (state_ == SequenceState::PoseMove) {
      if (r3_edge && !pose_move_then_disable_) {
        RCLCPP_WARN(this->get_logger(), "joy R3 during PoseMove: retarget idle, then disable");
        pose_move_then_disable_ = true;
        if (pose_target_ != NamedPose::Idle) {
          StartPoseMove(NamedPose::Idle);
        }
      }
      TickState(dt);
      return;
    }

    if (state_ == SequenceState::ProneHold) {
      if (square_edge) {
        if (thermal_latched_) {
          RCLCPP_WARN(
            this->get_logger(),
            "joy Square ignored: thermal protection latched (motor=%d was %.1f C, need <= %.1f C)",
            thermal_hot_motor_id_, thermal_hot_temp_c_, thermal_recover_c_);
          TickState(dt);
          return;
        }
        if (enable_named_poses_) {
          RCLCPP_WARN(this->get_logger(), "joy Square: go to named pose home");
          StartPoseMove(NamedPose::Home);
        } else {
          RCLCPP_WARN(this->get_logger(), "joy Square short press: stand up (SoftStand)");
          use_stand_resume_for_softstand_ = false;
          EnterState(SequenceState::SoftStand);
        }
      } else if (r3_edge) {
        RCLCPP_WARN(this->get_logger(), "joy R3 short press: disable from ProneHold");
        EnterState(SequenceState::Disable);
      }
      TickState(dt);
      return;
    }

    if (
      state_ == SequenceState::SoftStand || state_ == SequenceState::EnableInit ||
      state_ == SequenceState::Precheck)
    {
      // R3 在启动过程中同样可中止
      if (r3_edge) {
        if (enable_named_poses_) {
          RCLCPP_WARN(
            this->get_logger(), "joy R3: abort startup (gate stays closed), disable");
          PublishPoseServoActive(false);
          EnterState(SequenceState::Disable);
        } else {
          RCLCPP_WARN(
            this->get_logger(), "joy R3 short press: abort startup (SoftProne) then disable");
          BeginSoftProne(true);
        }
        return;
      }
    }

    TickState(dt);
  }

  void TickState(double dt)
  {
    elapsed_in_state_s_ += dt;
    switch (state_) {
      case SequenceState::Precheck:
        if (elapsed_in_state_s_ >= precheck_duration_s_) {
          EnterState(SequenceState::EnableInit);
        }
        break;
      case SequenceState::EnableInit:
        if (!init_sent_) {
          init_sent_ = true;
          if (!DoEnableInitFrames()) {
            break;
          }
        }
        if (enable_named_poses_) {
          // 原地保持：不写 prone_z、不进 SoftStand，等 □ / ○ / R3
          PublishTwistZero();
          if (elapsed_in_state_s_ >= init_hold_duration_s_) {
            BeginPoseHold();
          }
        } else {
          PublishHoldPose(prone_z_);
          if (elapsed_in_state_s_ >= init_hold_duration_s_) {
            EnterState(SequenceState::SoftStand);
          }
        }
        break;
      case SequenceState::SoftStand: {
          SetGateOpen(true);
          const double alpha = stand_duration_s_ > 1e-6 ? std::min(1.0, elapsed_in_state_s_ / stand_duration_s_) : 1.0;
          const double z_end = use_stand_resume_for_softstand_ ? stand_resume_z_ : stand_z_;
          const double z = prone_z_ + SmoothStep(alpha) * (z_end - prone_z_);
          PublishHoldPose(z);
          if (alpha >= 0.999) {
            EnterState(SequenceState::Running);
          }
        }
        break;
      case SequenceState::SoftProne: {
          SetGateOpen(true);
          const double alpha = prone_duration_s_ > 1e-6 ? std::min(1.0, elapsed_in_state_s_ / prone_duration_s_) : 1.0;
          const double z = soft_prone_z0_ + SmoothStep(alpha) * (prone_z_ - soft_prone_z0_);
          PublishHoldPose(z);
          if (alpha >= 0.999) {
            if (soft_prone_then_disable_) {
              EnterState(SequenceState::Disable);
            } else if (pending_start_after_prone_) {
              pending_start_after_prone_ = false;
              EnterState(SequenceState::SoftStand);
            } else {
              EnterState(SequenceState::ProneHold);
            }
          }
        }
        break;
      case SequenceState::ProneHold:
        SetGateOpen(true);
        if (enable_named_poses_ && pose_servo_active_) {
          PublishPoseServoTargets(hold_pose_);
          PublishTwistZero();
        } else {
          PublishHoldPose(prone_z_);
        }
        break;
      case SequenceState::PoseHold:
        // 必须等 pose servo 的交叉淡入完成（blend=1）再开门禁，
        // 否则开门禁瞬间下发的是 CHAMP 的站姿目标，会产生大幅跳变。
        if (elapsed_in_state_s_ >= pose_servo_blend_s_) {
          SetGateOpen(true);
        }
        PublishPoseServoTargets(hold_pose_);
        PublishTwistZero();
        break;
      case SequenceState::PoseMove:
        TickPoseMove(dt);
        break;
      case SequenceState::Disable:
        // 必须先关门禁、再撤 pose servo：否则 0.3s 淡出期间会把 CHAMP 站姿目标
        // 混进输出，表现为「失能前突然抬高」。
        SetGateOpen(false);
        if (!disable_sent_) {
          // 不下发「上报 OFF」：shutdown 后 RViz/观测仍依赖 0x18 刷新关节态；需要关上报请用手动脚本 reset
          SendCmdAll(kCmdReset);
          PublishPoseServoActive(false);
          disable_sent_ = true;
        }
        PublishHoldPose(prone_z_);
        if (elapsed_in_state_s_ >= 0.25) {
          EnterState(SequenceState::Idle);
        }
        break;
      case SequenceState::Fault:
      case SequenceState::Idle:
      case SequenceState::Running:
      default:
        break;
    }
  }

  /// 扫描反馈缓存，返回最高有效温度（°C）；无有效数据返回 -1.0
  double MaxFreshMotorTempC(int * motor_id_out)
  {
    double max_temp = -1.0;
    int max_id = -1;
    const int64_t now_ns = this->now().nanoseconds();
    for (size_t id = 0; id < last_feedback_temp_c_.size(); ++id) {
      if (last_feedback_stamp_ns_[id] <= 0) {
        continue;
      }
      const double age_s = static_cast<double>(now_ns - last_feedback_stamp_ns_[id]) / 1e9;
      if (age_s > thermal_fresh_timeout_s_) {
        continue;
      }
      if (last_feedback_temp_c_[id] > max_temp) {
        max_temp = last_feedback_temp_c_[id];
        max_id = static_cast<int>(id);
      }
    }
    if (motor_id_out) {
      *motor_id_out = max_id;
    }
    return max_temp;
  }

  /// 与 R3 完全等价的下电路径：软趴 -> 失能 -> Idle
  void TriggerThermalShutdown()
  {
    switch (state_) {
      case SequenceState::Running:
      case SequenceState::Precheck:
      case SequenceState::EnableInit:
      case SequenceState::SoftStand:
        BeginSoftProne(true);
        break;
      case SequenceState::SoftProne:
        // 已在趴下途中：不重新锚定 z0（避免高度跳变），仅升级为「趴完再失能」
        pending_start_after_prone_ = false;
        use_stand_resume_for_softstand_ = false;
        soft_prone_then_disable_ = true;
        break;
      case SequenceState::ProneHold:
        EnterState(SequenceState::Disable);
        break;
      case SequenceState::PoseHold:
      case SequenceState::PoseMove:
        // 已被 pose servo 接管：直接失能，不再走 CHAMP 趴下路径
        RCLCPP_WARN(
          this->get_logger(), "thermal shutdown in %s: pose servo off, go Disable", StateName(state_));
        SetGateOpen(false);
        PublishPoseServoActive(false);
        EnterState(SequenceState::Disable);
        break;
      case SequenceState::Idle:
      case SequenceState::Disable:
      case SequenceState::Fault:
      default:
        break;
    }
  }

  /// 过温判定 + 锁存/滞回。返回 true 表示本 tick 已触发（调用方应立即 return）
  bool UpdateThermalProtection(double dt)
  {
    if (!enable_thermal_protection_) {
      return false;
    }
    int max_id = -1;
    const double max_temp = MaxFreshMotorTempC(&max_id);

    if (thermal_latched_) {
      // 所有有效温度都回落到恢复阈值以下才解锁；无有效数据时保持锁存（保守）
      if (max_temp >= 0.0 && max_temp <= thermal_recover_c_) {
        thermal_latched_ = false;
        thermal_over_s_ = 0.0;
        RCLCPP_WARN(
          this->get_logger(),
          "thermal protection cleared: max temp %.1f C <= %.1f C; press L3 to power on again",
          max_temp, thermal_recover_c_);
      }
      return false;
    }

    if (max_temp >= thermal_limit_c_) {
      thermal_over_s_ += dt;
      if (thermal_over_s_ < thermal_debounce_s_) {
        return false;
      }
      thermal_latched_ = true;
      thermal_hot_motor_id_ = max_id;
      thermal_hot_temp_c_ = max_temp;
      RCLCPP_ERROR(
        this->get_logger(),
        "thermal protection TRIGGERED: motor=%d temp=%.1f C >= %.1f C; "
        "perform R3-equivalent safe-park -> disable (state=%s)",
        max_id, max_temp, thermal_limit_c_, StateName(state_));
      TriggerThermalShutdown();
      return true;
    }

    thermal_over_s_ = 0.0;
    return false;
  }

  void BeginSoftProne(bool then_disable)
  {
    pending_start_after_prone_ = false;
    soft_prone_then_disable_ = then_disable;
    const double raw = CurrentHeightCmdZ();
    soft_prone_z0_ = ClampTrackedZ0(raw);
    if (!then_disable) {
      stand_resume_z_ = soft_prone_z0_;
      use_stand_resume_for_softstand_ = true;
    } else {
      use_stand_resume_for_softstand_ = false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "SoftProne anchor z0=%.4f (tracked, then_disable=%d)", soft_prone_z0_, then_disable ? 1 : 0);
    EnterState(SequenceState::SoftProne);
  }

  void OnBodyPose(const Pose::SharedPtr msg)
  {
    if (!std::isfinite(msg->position.z)) {
      return;
    }
    last_body_pose_cmd_z_ = msg->position.z;
    last_body_pose_receive_time_ = this->now();
    has_body_pose_sample_ = true;
  }

  double CurrentHeightCmdZ()
  {
    if (!track_body_pose_height_) {
      return stand_z_;
    }
    if (has_body_pose_sample_) {
      const double age = (this->now() - last_body_pose_receive_time_).seconds();
      if (age <= body_pose_stale_s_) {
        return last_body_pose_cmd_z_;
      }
    }
    if (has_last_published_cmd_z_) {
      return last_published_cmd_z_;
    }
    return stand_z_;
  }

  double ClampTrackedZ0(double raw) const
  {
    return std::clamp(raw, track_z_clamp_min_, track_z_clamp_max_);
  }

  /// S 曲线：s(0)=0, s(1)=1, s'(0)=s'(1)=0，避免起/止时刻高度速度阶跃
  /// （速度前馈会把这个阶跃放大成 kd 力矩冲击）。
  static double SmoothStep(double alpha)
  {
    const double a = std::clamp(alpha, 0.0, 1.0);
    return a * a * (3.0 - 2.0 * a);
  }

  // --- 命名姿态（idle/home）---

  /// 最小 jerk（五次多项式）：s(0)=0, s(1)=1, 一阶/二阶导在两端均为 0
  static double MinimumJerk(double alpha)
  {
    const double a = std::clamp(alpha, 0.0, 1.0);
    return a * a * a * (10.0 - 15.0 * a + 6.0 * a * a);
  }

  /// 从 YAML 读取 idle_joints / home_joints；失败返回 false 并保持 poses_loaded_=false
  void LoadNamedPoses()
  {
    std::string path = named_poses_file_;
    if (path.empty()) {
      path = ResolveDefaultPoseFile();
    }
    if (path.empty()) {
      RCLCPP_WARN(this->get_logger(), "named_poses_file empty and default not found");
      return;
    }
    try {
      YAML::Node root = YAML::LoadFile(path);
      YAML::Node params = root["/**"] && root["/**"]["ros__parameters"] ?
        root["/**"]["ros__parameters"] : root;
      auto read = [&](const char * key, std::array<double, 12> & out) -> bool {
          if (!params[key]) {
            RCLCPP_WARN(this->get_logger(), "%s: missing key '%s'", path.c_str(), key);
            return false;
          }
          const auto seq = params[key];
          if (!seq.IsSequence() || seq.size() != out.size()) {
            RCLCPP_WARN(
              this->get_logger(), "%s: '%s' must be a 12-element sequence", path.c_str(), key);
            return false;
          }
          for (size_t i = 0; i < out.size(); ++i) {
            const double v = seq[i].as<double>();
            if (!std::isfinite(v)) {
              RCLCPP_WARN(
                this->get_logger(), "%s: '%s'[%zu] is not finite", path.c_str(), key, i);
              return false;
            }
            out[i] = v;
          }
          return true;
        };
      auto read_bool = [&](const char * key) -> bool {
          if (!params[key]) {
            return false;
          }
          try {
            return params[key].as<bool>();
          } catch (const std::exception &) {
            return false;
          }
        };
      const bool idle_ok = read("idle_joints", idle_pose_);
      const bool home_ok = read("home_joints", home_pose_);
      if (!idle_ok || !home_ok) {
        return;
      }
      idle_recorded_ = read_bool("idle_recorded");
      home_recorded_ = read_bool("home_recorded");
      poses_loaded_ = true;
      named_poses_file_ = path;
      LogNamedPoses(path);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "load named poses failed (%s): %s", path.c_str(), e.what());
    }
  }

  std::string ResolveDefaultPoseFile() const
  {
    // 优先安装目录（本工作区为 symlink-install，等价于源码 config）
    const char * share_hint = "/home/cat/trotbot_ws/install/trotbot_can_bridge/share/"
      "trotbot_can_bridge/config/named_poses.yaml";
    if (std::filesystem::exists(share_hint)) {
      return share_hint;
    }
    const char * src_hint = "/home/cat/trotbot_ws/src/trotbot_can_bridge/config/named_poses.yaml";
    if (std::filesystem::exists(src_hint)) {
      return src_hint;
    }
    return {};
  }

  void LogNamedPoses(const std::string & path)
  {
    auto fmt = [](const std::array<double, 12> & q) {
        std::ostringstream os;
        for (size_t i = 0; i < q.size(); ++i) {
          os << (i == 0 ? "" : ", ") << std::fixed << std::setprecision(1)
             << (q[i] * 57.29577951308232);
        }
        return os.str();
      };
    RCLCPP_WARN(
      this->get_logger(),
      "named poses loaded from %s (idle_recorded=%d, home_recorded=%d)\n"
      "  idle(deg)=[%s]\n  home(deg)=[%s]",
      path.c_str(), idle_recorded_ ? 1 : 0, home_recorded_ ? 1 : 0,
      fmt(idle_pose_).c_str(), fmt(home_pose_).c_str());
    if (!idle_recorded_ || !home_recorded_) {
      RCLCPP_WARN(
        this->get_logger(),
        "unrecorded pose(s) will REFUSE motion: %s%s%s",
        idle_recorded_ ? "" : "idle ",
        home_recorded_ ? "" : "home ",
        "(run record_named_pose.py <pose>)");
    }
  }

  void PublishPoseServoActive(bool on)
  {
    pose_servo_active_ = on;
    if (!pose_servo_active_pub_) {
      return;
    }
    Bool msg;
    msg.data = on;
    pose_servo_active_pub_->publish(msg);
  }

  void PublishPoseServoTargets(const std::array<double, 12> & q)
  {
    if (!pose_servo_joint_pub_) {
      return;
    }
    JointState msg;
    msg.header.stamp = this->now();
    msg.name.reserve(DogMapper::kChampJointNames.size());
    for (const char * n : DogMapper::kChampJointNames) {
      msg.name.emplace_back(n);
    }
    msg.position.assign(q.begin(), q.end());
    pose_servo_joint_pub_->publish(msg);
  }

  void PublishTwistZero()
  {
    Twist tw;
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;
    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = 0.0;
    cmd_vel_pub_->publish(tw);
  }

  void OnFeedbackJointStates(const JointState::SharedPtr msg)
  {
    const size_t n = std::min(msg->position.size(), static_cast<size_t>(12));
    if (n < 12) {
      return;
    }
    bool mapped_by_name = false;
    if (msg->name.size() >= 12) {
      std::array<double, 12> tmp{};
      bool all_found = true;
      for (size_t i = 0; i < 12; ++i) {
        bool found = false;
        for (size_t k = 0; k < msg->name.size(); ++k) {
          if (msg->name[k] == DogMapper::kChampJointNames[i]) {
            tmp[i] = msg->position[k];
            found = true;
            break;
          }
        }
        if (!found) {
          all_found = false;
          break;
        }
      }
      if (all_found) {
        champ_feedback_ = tmp;
        mapped_by_name = true;
      }
    }
    if (!mapped_by_name) {
      for (size_t i = 0; i < 12; ++i) {
        champ_feedback_[i] = msg->position[i];
      }
    }
    champ_feedback_stamp_ns_ = this->now().nanoseconds();
    has_champ_feedback_ = true;
  }

  /// 当前 CHAMP 域 12 关节角；反馈超时则 ok=false
  std::array<double, 12> CurrentChampPose(bool * ok) const
  {
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t fresh_ns = static_cast<int64_t>(motor_feedback_fresh_timeout_s_ * 1e9);
    const bool fresh = has_champ_feedback_ && champ_feedback_stamp_ns_ > 0 &&
      (now_ns - champ_feedback_stamp_ns_) <= fresh_ns;
    if (ok != nullptr) {
      *ok = fresh;
    }
    return champ_feedback_;
  }

  /// L3 上电收尾：原地保持当前姿态（不写 prone_z，不进 SoftStand）
  void BeginPoseHold()
  {
    bool ok = false;
    const auto q = CurrentChampPose(&ok);
    if (!ok) {
      RCLCPP_ERROR(
        this->get_logger(),
        "pose hold aborted: /joint_states_feedback stale (>%.2fs)", motor_feedback_fresh_timeout_s_);
      EnterState(SequenceState::Idle);
      return;
    }
    hold_pose_ = q;
    PublishPoseServoTargets(hold_pose_);
    PublishPoseServoActive(true);
    // 不在这里开门禁：等 PoseHold 里 blend 完成后再开，避免首帧走 CHAMP 站姿目标
    PublishTwistZero();
    RCLCPP_WARN(this->get_logger(), "pose hold: enabled in place, holding current pose");
    EnterState(SequenceState::PoseHold);
  }

  bool StartPoseMove(NamedPose target)
  {
    if (!enable_named_poses_ || !poses_loaded_) {
      RCLCPP_WARN(this->get_logger(), "pose move ignored: named poses unavailable");
      return false;
    }
    // 未录制的姿态拒绝运动：占位全 0 会把腿拉直，比不动更危险
    const bool recorded = (target == NamedPose::Idle) ? idle_recorded_ : home_recorded_;
    if (!recorded) {
      RCLCPP_ERROR(
        this->get_logger(),
        "pose move -> %s REFUSED: not recorded yet (run: record_named_pose.py %s)",
        NamedPoseName(target), NamedPoseName(target));
      return false;
    }
    const std::array<double, 12> & q1 =
      (target == NamedPose::Idle) ? idle_pose_ : home_pose_;
    bool ok = false;
    const auto q0 = CurrentChampPose(&ok);
    if (!ok) {
      RCLCPP_ERROR(
        this->get_logger(),
        "pose move ignored: /joint_states_feedback stale (>%.2fs)",
        motor_feedback_fresh_timeout_s_);
      return false;
    }
    double max_delta = 0.0;
    for (size_t i = 0; i < q0.size(); ++i) {
      max_delta = std::max(max_delta, std::fabs(q1[i] - q0[i]));
    }
    pose_q0_ = q0;
    pose_q1_ = q1;
    pose_target_ = target;
    pose_move_dur_ = std::max(pose_move_duration_s_, max_delta / pose_max_joint_speed_rad_s_);
    pose_move_t_ = -pose_servo_blend_s_;

    PublishPoseServoTargets(pose_q0_);
    PublishPoseServoActive(true);
    // 门禁由 TickPoseMove 在淡入完成后打开
    PublishTwistZero();
    RCLCPP_WARN(
      this->get_logger(), "pose move -> %s: max_delta=%.3f rad dur=%.2fs",
      NamedPoseName(target), max_delta, pose_move_dur_);
    EnterState(SequenceState::PoseMove);
    return true;
  }

  void TickPoseMove(double dt)
  {
    pose_move_t_ += dt;
    PublishTwistZero();

    if (pose_move_t_ < 0.0) {
      // 淡入阶段：先锁住起点，门禁保持关闭，避免开门禁瞬间走 CHAMP 站姿目标
      PublishPoseServoTargets(pose_q0_);
      return;
    }
    SetGateOpen(true);
    const double alpha = pose_move_dur_ > 1e-6 ? pose_move_t_ / pose_move_dur_ : 1.0;
    const double s = MinimumJerk(alpha);
    std::array<double, 12> q{};
    for (size_t i = 0; i < q.size(); ++i) {
      q[i] = pose_q0_[i] + s * (pose_q1_[i] - pose_q0_[i]);
    }
    PublishPoseServoTargets(q);

    if (alpha >= 0.999) {
      FinishPoseMove();
    }
  }

  void FinishPoseMove()
  {
    const NamedPose target = pose_target_;
    pose_target_ = NamedPose::None;
    hold_pose_ = pose_q1_;
    if (target == NamedPose::Idle) {
      if (pose_move_then_disable_) {
        pose_move_then_disable_ = false;
        RCLCPP_WARN(this->get_logger(), "named pose idle reached; disable");
        // 先关门禁再撤 pose servo，避免淡出期间混入 CHAMP 站姿目标
        SetGateOpen(false);
        PublishPoseServoActive(false);
        EnterState(SequenceState::Disable);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "named pose idle reached; holding (ProneHold)");
      EnterState(SequenceState::ProneHold);
      return;
    }
    if (target == NamedPose::Home) {
      if (enable_stand_after_home_) {
        RCLCPP_WARN(
          this->get_logger(), "named pose home reached; hand over to CHAMP (Running)");
        PublishPoseServoActive(false);
        PublishHoldPose(stand_z_);
        EnterState(SequenceState::Running);
      } else {
        RCLCPP_WARN(this->get_logger(), "named pose home reached; holding (PoseHold)");
        EnterState(SequenceState::PoseHold);
      }
    }
  }

  void EnterState(SequenceState next)
  {
    state_ = next;
    elapsed_in_state_s_ = 0.0;
    init_sent_ = false;
    disable_sent_ = false;
    PublishState();
    RCLCPP_WARN(this->get_logger(), "power_sequence state -> %s", StateName(state_));
  }

  const char * StateName(SequenceState s) const
  {
    switch (s) {
      case SequenceState::Idle: return "Idle";
      case SequenceState::Precheck: return "Precheck";
      case SequenceState::EnableInit: return "EnableInit";
      case SequenceState::SoftStand: return "SoftStand";
      case SequenceState::Running: return "Running";
      case SequenceState::SoftProne: return "SoftProne";
      case SequenceState::ProneHold: return "ProneHold";
      case SequenceState::Disable: return "Disable";
      case SequenceState::Fault: return "Fault";
      case SequenceState::PoseHold: return "PoseHold";
      case SequenceState::PoseMove: return "PoseMove";
      default: return "Unknown";
    }
  }

  void PublishState()
  {
    String msg;
    msg.data = StateName(state_);
    state_pub_->publish(msg);
  }

  void PublishGate()
  {
    Bool msg;
    msg.data = gate_open_;
    gate_pub_->publish(msg);
  }

  /** 仅在实际跳变时发布，避免 TickState 每周期 PublishGate 触发下游（如 status_led）狂刷 WS2812。 */
  void SetGateOpen(bool open)
  {
    if (gate_open_ == open) {
      return;
    }
    gate_open_ = open;
    PublishGate();
  }

  void PublishHoldPose(double z)
  {
    last_published_cmd_z_ = z;
    has_last_published_cmd_z_ = true;

    Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose_pub_->publish(pose);

    Twist tw;
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;
    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = 0.0;
    cmd_vel_pub_->publish(tw);
  }

  bool DoEnableInitFrames()
  {
    // 上电不再 set_zero / 不再发 MIT p=0：先要求 12/12 反馈新鲜，否则一个使能帧都不发
    if (require_feedback_before_enable_) {
      const size_t fresh = CountFreshFeedback();
      if (fresh < DogMapper::kTemporaryIndexMap.size()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "abort start: feedback %zu/%zu fresh (timeout=%.2fs); no enable frames sent",
          fresh, DogMapper::kTemporaryIndexMap.size(), motor_feedback_fresh_timeout_s_);
        EnterState(SequenceState::Idle);
        return false;
      }
    }
    if (enable_active_report_in_startup_ && send_enable_in_startup_) {
      const uint16_t scan = HzToEpScanTime(active_report_hz_);
      SendEpScanTimeAll(scan);
      RCLCPP_INFO(
        this->get_logger(),
        "active report: EPScan_time=%u (~%.2f Hz requested) before enable",
        static_cast<unsigned>(scan), active_report_hz_);
    }
    if (send_enable_in_startup_) {
      SendCmdAll(kCmdEnable);
    }
    if (send_mit_hold_in_startup_) {
      SendMitHoldAll();
    }
    if (enable_active_report_in_startup_ && send_enable_in_startup_) {
      SendActiveReportAll(true);
      RCLCPP_INFO(this->get_logger(), "active report: COMM 0x18 ON (all motors)");
    }
    return true;
  }

  int ReportMasterForCanId() const
  {
    return active_report_master_id_ >= 0 ? active_report_master_id_ : motor_master_id_;
  }

  void SendEpScanTimeAll(uint16_t scan_time)
  {
    const uint8_t p_lo = static_cast<uint8_t>(kParamEpScanTime & 0xFF);
    const uint8_t p_hi = static_cast<uint8_t>((kParamEpScanTime >> 8) & 0xFF);
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(route.bus, route.motor_id, kCmdSetParam, motor_master_id_);
      frame.data[0] = p_lo;
      frame.data[1] = p_hi;
      frame.data[2] = 0;
      frame.data[3] = 0;
      frame.data[4] = static_cast<uint8_t>(scan_time & 0xFF);
      frame.data[5] = static_cast<uint8_t>((scan_time >> 8) & 0xFF);
      frame.data[6] = 0;
      frame.data[7] = 0;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void SendActiveReportAll(bool on)
  {
    static constexpr std::array<uint8_t, 6> kReportPrefix{0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    const int rmaster = ReportMasterForCanId();
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(route.bus, route.motor_id, kCommActiveReport, rmaster);
      for (size_t i = 0; i < kReportPrefix.size(); ++i) {
        frame.data[i] = kReportPrefix[i];
      }
      frame.data[6] = on ? 0x01 : 0x00;
      frame.data[7] = 0x00;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void PublishSetZeroEvent(const std::string & payload)
  {
    if (!event_pub_) {
      return;
    }
    String msg;
    msg.data = payload;
    event_pub_->publish(msg);
  }

  void PublishMotorInitEvent(const std::string & payload)
  {
    if (!motor_init_event_pub_) {
      return;
    }
    String msg;
    msg.data = payload;
    motor_init_event_pub_->publish(msg);
  }

  bool TryRemoteSetZero(const char * trigger)
  {
    if (!enable_remote_set_zero_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "reject set_zero (%s): enable_remote_set_zero=false", trigger);
      return false;
    }
    if (state_ != SequenceState::Idle || gate_open_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "reject set_zero (%s): require state=Idle and gate_open=false (now state=%s gate_open=%d)",
        trigger, StateName(state_), gate_open_ ? 1 : 0);
      std::ostringstream oss;
      oss << "set_zero_rejected:" << StateName(state_) << ":gate=" << (gate_open_ ? 1 : 0) << ":" << trigger;
      PublishSetZeroEvent(oss.str());
      return false;
    }
    const int rounds = std::clamp(set_zero_repeat_count_, 1, 10);
    const int gap_ms = std::max(0, set_zero_repeat_interval_ms_);
    for (int r = 0; r < rounds; ++r) {
      SendSetZeroAll();
      if (r + 1 < rounds && gap_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
      }
    }
    RCLCPP_WARN(
      this->get_logger(),
      "set_zero: sent CMD 0x06 to all 12 motors %d round(s), gap_ms=%d (trigger=%s). Mechanical zero redefined — verify URDF/joint_offsets vs EL05 layer.",
      rounds, gap_ms, trigger);
    PublishSetZeroEvent(std::string("set_zero_ok:") + trigger);
    return true;
  }

  void SendSetZeroAll()
  {
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(route.bus, route.motor_id, kCmdSetZero, motor_master_id_);
      frame.data[0] = 0x01;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void SendCmdAll(uint8_t cmd)
  {
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      tx_pub_->publish(FrameCodec::Pack(BuildCmdFrame(route.bus, route.motor_id, cmd, motor_master_id_)));
    }
  }

  size_t CountFreshFeedback() const
  {
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t fresh_ns = static_cast<int64_t>(motor_feedback_fresh_timeout_s_ * 1e9);
    size_t fresh = 0;
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      const int64_t stamp = last_feedback_stamp_ns_[route.motor_id];
      if (stamp > 0 && (now_ns - stamp) <= fresh_ns) {
        ++fresh;
      }
    }
    return fresh;
  }

  /// 使能首帧：按各电机当前反馈角保持（绝不再用 p=0），避免绕圈回零
  void SendMitHoldAll()
  {
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t fresh_ns = static_cast<int64_t>(motor_feedback_fresh_timeout_s_ * 1e9);
    int missing = 0;
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      const int64_t stamp = last_feedback_stamp_ns_[route.motor_id];
      if (stamp <= 0 || (now_ns - stamp) > fresh_ns) {
        ++missing;
        continue;
      }
      const auto frame = ProtocolCodec::BuildMitControlFrame(
        route.bus,
        route.motor_id,
        static_cast<float>(last_feedback_angle_rad_[route.motor_id]),
        static_cast<float>(init_velocity_),
        static_cast<float>(init_kp_),
        static_cast<float>(init_kd_),
        static_cast<float>(init_tau_));
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
    if (missing > 0) {
      RCLCPP_WARN(
        this->get_logger(), "mit hold: %d motor(s) skipped (no fresh feedback)", missing);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "mit hold: 12 motors anchored at current feedback (kp=%.1f kd=%.2f)", init_kp_, init_kd_);
    }
  }

  bool BeginMotorInit(const char * trigger, bool verify_only)
  {
    if (motor_init_active_) {
      RCLCPP_WARN(this->get_logger(), "motor_init ignored (%s): already running", trigger);
      return false;
    }
    if (!verify_only && !enable_remote_motor_init_) {
      RCLCPP_WARN(
        this->get_logger(), "motor_init rejected (%s): enable_remote_motor_init=false", trigger);
      PublishMotorInitEvent(std::string("motor_init_rejected:disabled:") + trigger);
      return false;
    }
    if (state_ != SequenceState::Idle || gate_open_) {
      RCLCPP_WARN(
        this->get_logger(),
        "motor_init rejected (%s): require state=Idle and gate_open=false (now state=%s gate_open=%d)",
        trigger, StateName(state_), gate_open_ ? 1 : 0);
      std::ostringstream oss;
      oss << "motor_init_rejected:" << StateName(state_) << ":gate=" << (gate_open_ ? 1 : 0)
          << ":" << trigger;
      PublishMotorInitEvent(oss.str());
      return false;
    }

    motor_init_readback_.fill(-1);
    motor_init_has_readback_.fill(false);
    motor_init_verify_only_ = verify_only;
    motor_init_active_ = true;
    motor_init_phase_armed_ = false;
    motor_init_set_zero_round_ = 0;
    motor_init_read_round_ = 0;
    motor_init_phase_time_ = this->now();
    motor_init_last_round_time_ = motor_init_phase_time_;
    if (verify_only) {
      motor_init_phase_ = MotorInitPhase::ReadBack;
      RCLCPP_INFO(
        this->get_logger(), "zero_sta self-check (%s): read-only probe of 0x%04X",
        trigger, ProtocolCodec::kParamZeroSta);
    } else if (motor_init_do_set_zero_) {
      motor_init_phase_ = MotorInitPhase::SetZero;
      RCLCPP_WARN(
        this->get_logger(),
        "motor_init start (%s): set_zero -> zero_sta(0x%04X)=%d -> save -> verify (12 motors); "
        "current pose becomes mechanical zero, effective only after power cycle",
        trigger, ProtocolCodec::kParamZeroSta, motor_init_zero_sta_);
    } else {
      motor_init_phase_ = MotorInitPhase::SetZeroSta;
      RCLCPP_WARN(
        this->get_logger(),
        "motor_init start (%s): write zero_sta(0x%04X)=%d -> save -> verify (12 motors); "
        "effective only after power cycle",
        trigger, ProtocolCodec::kParamZeroSta, motor_init_zero_sta_);
    }
    return true;
  }

  void TickMotorInit()
  {
    if (!motor_init_active_) {
      return;
    }
    const double elapsed_ms = (this->now() - motor_init_phase_time_).seconds() * 1000.0;
    const uint8_t master = static_cast<uint8_t>(motor_master_id_);

    switch (motor_init_phase_) {
      case MotorInitPhase::SetZero: {
          const int rounds = std::clamp(set_zero_repeat_count_, 1, 10);
          if (!motor_init_phase_armed_) {
            SendSetZeroAll();
            motor_init_set_zero_round_ = 1;
            motor_init_last_round_time_ = this->now();
            motor_init_phase_armed_ = true;
            RCLCPP_WARN(
              this->get_logger(), "motor_init: set_zero round 1/%d (current pose = mechanical zero)",
              rounds);
          } else if (motor_init_set_zero_round_ < static_cast<uint32_t>(rounds)) {
            const auto now = this->now();
            const double since_round_ms = (now - motor_init_last_round_time_).seconds() * 1000.0;
            if (since_round_ms >= static_cast<double>(std::max(0, set_zero_repeat_interval_ms_))) {
              SendSetZeroAll();
              ++motor_init_set_zero_round_;
              motor_init_last_round_time_ = now;
              RCLCPP_WARN(
                this->get_logger(), "motor_init: set_zero round %u/%d",
                static_cast<unsigned>(motor_init_set_zero_round_), rounds);
            }
          }
          if (
            motor_init_set_zero_round_ >= static_cast<uint32_t>(rounds) &&
            elapsed_ms >= static_cast<double>(motor_init_set_zero_wait_ms_))
          {
            EnterMotorInitPhase(MotorInitPhase::SetZeroSta);
          }
        }
        break;
      case MotorInitPhase::SetZeroSta:
        if (!motor_init_phase_armed_) {
          for (const auto & route : DogMapper::kTemporaryIndexMap) {
            tx_pub_->publish(FrameCodec::Pack(ProtocolCodec::BuildSetParamU8Frame(
              route.bus, route.motor_id, ProtocolCodec::kParamZeroSta,
              static_cast<uint8_t>(motor_init_zero_sta_), master)));
          }
          motor_init_phase_armed_ = true;
          RCLCPP_WARN(
            this->get_logger(), "motor_init: wrote 0x%04X=%d to 12 motors",
            ProtocolCodec::kParamZeroSta, motor_init_zero_sta_);
        }
        if (elapsed_ms >= static_cast<double>(motor_init_set_wait_ms_)) {
          EnterMotorInitPhase(MotorInitPhase::SaveParams);
        }
        break;
      case MotorInitPhase::SaveParams:
        if (!motor_init_phase_armed_) {
          for (const auto & route : DogMapper::kTemporaryIndexMap) {
            tx_pub_->publish(
              FrameCodec::Pack(ProtocolCodec::BuildSaveParamFrame(route.bus, route.motor_id, master)));
          }
          motor_init_phase_armed_ = true;
          RCLCPP_WARN(
            this->get_logger(), "motor_init: save params (CMD 0x16, data 01..08) to 12 motors");
        }
        if (elapsed_ms >= static_cast<double>(motor_init_save_wait_ms_)) {
          EnterMotorInitPhase(MotorInitPhase::ReadBack);
        }
        break;
      case MotorInitPhase::ReadBack: {
          // 读参数应答可能因总线忙/上报冲突丢帧：窗口内每 150ms 重发一次 GET，最多 3~4 轮
          const auto now = this->now();
          const double since_read_ms = (now - motor_init_last_round_time_).seconds() * 1000.0;
          if (!motor_init_phase_armed_ || since_read_ms >= 150.0) {
            for (const auto & route : DogMapper::kTemporaryIndexMap) {
              tx_pub_->publish(FrameCodec::Pack(ProtocolCodec::BuildGetParamFrame(
                route.bus, route.motor_id, ProtocolCodec::kParamZeroSta, master)));
            }
            motor_init_phase_armed_ = true;
            motor_init_last_round_time_ = now;
            ++motor_init_read_round_;
          }
          if (elapsed_ms >= static_cast<double>(motor_init_read_wait_ms_)) {
            FinishMotorInit();
          }
        }
        break;
      case MotorInitPhase::Idle:
      default:
        break;
    }
  }

  void EnterMotorInitPhase(MotorInitPhase next)
  {
    motor_init_phase_ = next;
    motor_init_phase_time_ = this->now();
    motor_init_phase_armed_ = false;
  }

  void FinishMotorInit()
  {
    const int total = static_cast<int>(DogMapper::kTemporaryIndexMap.size());
    int ok = 0;
    std::ostringstream missing;
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      const uint8_t mid = route.motor_id;
      if (motor_init_has_readback_[mid] && motor_init_readback_[mid] == motor_init_zero_sta_) {
        ++ok;
      } else {
        if (missing.tellp() > 0) {
          missing << ",";
        }
        missing << static_cast<int>(mid);
      }
    }

    std::ostringstream oss;
    if (motor_init_verify_only_) {
      if (ok == total) {
        oss << "zero_sta_verify_ok:" << motor_init_zero_sta_;
        RCLCPP_INFO(this->get_logger(), "%s (%d/%d)", oss.str().c_str(), ok, total);
      } else {
        oss << "zero_sta_verify_fail:" << motor_init_zero_sta_ << ":ok=" << ok << "/" << total
            << ":missing=" << missing.str();
        RCLCPP_WARN(
          this->get_logger(),
          "%s — 未完成一次性电机初始化，请 L1+R1 长按 %.1fs（写后需断电重启生效）",
          oss.str().c_str(), motor_init_longpress_s_);
      }
    } else {
      const std::string zero_tag = motor_init_do_set_zero_ ? ":set_zero=1" : ":set_zero=0";
      if (motor_init_do_set_zero_) {
        const int64_t now_ns = this->now().nanoseconds();
        const int64_t fresh_ns = static_cast<int64_t>(motor_feedback_fresh_timeout_s_ * 1e9);
        double max_abs = 0.0;
        size_t fresh_count = 0;
        for (const auto & route : DogMapper::kTemporaryIndexMap) {
          const int64_t stamp = last_feedback_stamp_ns_[route.motor_id];
          if (stamp > 0 && (now_ns - stamp) <= fresh_ns) {
            max_abs = std::max(max_abs, std::fabs(last_feedback_angle_rad_[route.motor_id]));
            ++fresh_count;
          }
        }
        RCLCPP_INFO(
          this->get_logger(),
          "motor_init: post set_zero fresh feedback=%zu/%d, |motor angle| max=%.4f rad (0 ≈ 已回零)",
          fresh_count, total, max_abs);
      }
      if (ok == total) {
        oss << "motor_init_ok:zero_sta=" << motor_init_zero_sta_ << zero_tag << ":" << ok << "/" << total;
        RCLCPP_WARN(
          this->get_logger(), "%s — 已保存到电机 flash；请断电重启后生效", oss.str().c_str());
      } else {
        oss << "motor_init_fail:zero_sta=" << motor_init_zero_sta_ << zero_tag << ":ok=" << ok << "/"
            << total << ":missing=" << missing.str();
        RCLCPP_ERROR(
          this->get_logger(),
          "%s — 参数可能已写入但回读不全，请检查总线/固件或用脚本复查", oss.str().c_str());
      }
    }
    PublishMotorInitEvent(oss.str());

    motor_init_active_ = false;
    motor_init_phase_ = MotorInitPhase::Idle;
    motor_init_phase_armed_ = false;
    motor_init_verify_only_ = false;
  }

  void OnVerifyZeroStaTimer()
  {
    if (verify_timer_) {
      verify_timer_->cancel();
    }
    if (state_ != SequenceState::Idle || gate_open_ || motor_init_active_) {
      RCLCPP_INFO(
        this->get_logger(), "skip zero_sta self-check: state=%s gate_open=%d",
        StateName(state_), gate_open_ ? 1 : 0);
      return;
    }
    (void)BeginMotorInit("launch", true);
  }

  CanFrameMessage BuildCmdFrame(CanBus bus, uint8_t motor_id, uint8_t cmd, int master_id) const
  {
    CanFrameMessage frame;
    frame.bus = bus;
    frame.is_extended = true;
    frame.dlc = 8;
    frame.can_id = ((static_cast<uint32_t>(cmd) << 24) |
      (static_cast<uint32_t>(master_id & 0xFF) << 8) |
      static_cast<uint32_t>(motor_id));
    frame.data.fill(0);
    return frame;
  }

  std::string gate_topic_;
  std::string state_topic_;
  std::string tx_topic_;
  std::string rx_topic_;
  std::string body_pose_topic_;
  std::string cmd_vel_topic_;
  std::string joy_topic_;
  std::string command_topic_;

  int timer_period_ms_{20};
  double precheck_duration_s_{0.2};
  double init_hold_duration_s_{0.35};
  double stand_duration_s_{2.5};
  double prone_duration_s_{2.5};
  double stand_z_{0.0};
  double prone_z_{-0.35};

  int button_l1_{4};
  int button_r1_{5};
  int button_share_{8};
  int button_circle_{1};
  int button_square_{3};
  int button_l3_{11};
  int button_r3_{12};

  int motor_master_id_{253};
  double init_kp_{20.0};
  double init_kd_{1.5};
  double init_velocity_{0.0};
  double init_tau_{0.0};
  bool send_enable_in_startup_{true};
  bool send_mit_hold_in_startup_{true};
  bool require_feedback_before_enable_{true};
  double motor_feedback_fresh_timeout_s_{1.0};
  bool enable_thermal_protection_{true};
  double thermal_limit_c_{100.0};
  double thermal_recover_c_{80.0};
  double thermal_debounce_s_{0.5};
  double thermal_fresh_timeout_s_{2.0};
  double thermal_over_s_{0.0};      // 连续超限累计时间
  bool thermal_latched_{false};     // 锁存：置位后禁止上电
  int thermal_hot_motor_id_{-1};    // 最近一次触发电机（日志用）
  double thermal_hot_temp_c_{0.0};
  bool enable_active_report_in_startup_{true};
  double active_report_hz_{10.0};
  int active_report_master_id_{-1};
  bool enable_active_report_at_launch_{true};
  double active_report_boot_delay_s_{0.25};

  bool enable_remote_set_zero_{true};
  std::string set_zero_event_topic_;
  int set_zero_repeat_count_{3};
  int set_zero_repeat_interval_ms_{40};
  bool l3_prev_{false};
  bool square_prev_{false};
  bool circle_prev_{false};
  bool r3_prev_{false};

  bool enable_remote_motor_init_{true};
  double motor_init_longpress_s_{3.0};
  bool motor_init_do_set_zero_{true};
  int motor_init_zero_sta_{1};
  int motor_init_tick_ms_{50};
  int motor_init_set_zero_wait_ms_{400};
  int motor_init_set_wait_ms_{200};
  int motor_init_save_wait_ms_{500};
  int motor_init_read_wait_ms_{600};
  std::string motor_init_event_topic_;
  bool verify_zero_sta_at_launch_{true};
  double zero_sta_verify_delay_s_{1.0};
  MotorInitPhase motor_init_phase_{MotorInitPhase::Idle};
  bool motor_init_active_{false};
  bool motor_init_verify_only_{false};
  bool motor_init_phase_armed_{false};
  uint32_t motor_init_set_zero_round_{0};
  uint32_t motor_init_read_round_{0};
  rclcpp::Time motor_init_last_round_time_;
  rclcpp::Time motor_init_phase_time_;
  std::array<int, 64> motor_init_readback_{};
  std::array<bool, 64> motor_init_has_readback_{};
  double init_hold_s_{0.0};
  bool init_latch_{false};

  // 电机反馈缓存（按 motor_id 索引，ID ≤ 63）：MIT 域 rad + 温度 °C
  std::array<double, 64> last_feedback_angle_rad_{};
  std::array<double, 64> last_feedback_temp_c_{};
  std::array<int64_t, 64> last_feedback_stamp_ns_{};

  bool track_body_pose_height_{true};
  double body_pose_stale_s_{2.0};
  double track_z_clamp_min_{-0.5};
  double track_z_clamp_max_{0.5};
  double last_body_pose_cmd_z_{0.0};
  rclcpp::Time last_body_pose_receive_time_;
  bool has_body_pose_sample_{false};
  double last_published_cmd_z_{0.0};
  bool has_last_published_cmd_z_{false};
  double soft_prone_z0_{0.0};
  double stand_resume_z_{0.0};
  bool use_stand_resume_for_softstand_{false};

  bool has_joy_{false};
  Joy latest_joy_;
  bool soft_prone_then_disable_{true};
  bool pending_start_after_prone_{false};
  SequenceState state_{SequenceState::Idle};
  double elapsed_in_state_s_{0.0};
  bool gate_open_{false};
  bool init_sent_{false};
  bool disable_sent_{false};

  rclcpp::Publisher<UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<Bool>::SharedPtr gate_pub_;
  rclcpp::Publisher<String>::SharedPtr state_pub_;
  rclcpp::Publisher<String>::SharedPtr event_pub_;
  rclcpp::Publisher<String>::SharedPtr motor_init_event_pub_;
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<String>::SharedPtr command_sub_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<Pose>::SharedPtr body_pose_sub_;

  // --- 命名姿态（idle/home）---
  bool enable_named_poses_{true};
  std::string named_poses_file_;
  std::string feedback_joint_states_topic_;
  std::string pose_servo_active_topic_;
  std::string pose_servo_joint_topic_;
  double pose_move_duration_s_{2.5};
  double pose_max_joint_speed_rad_s_{1.2};
  double pose_servo_blend_s_{0.30};
  bool enable_stand_after_home_{true};
  bool poses_loaded_{false};
  bool idle_recorded_{false};
  bool home_recorded_{false};
  bool pose_servo_active_{false};
  bool pose_move_then_disable_{false};
  NamedPose pose_target_{NamedPose::None};
  std::array<double, 12> idle_pose_{};
  std::array<double, 12> home_pose_{};
  std::array<double, 12> hold_pose_{};
  std::array<double, 12> pose_q0_{};
  std::array<double, 12> pose_q1_{};
  double pose_move_dur_{2.5};
  double pose_move_t_{0.0};
  std::array<double, 12> champ_feedback_{};
  int64_t champ_feedback_stamp_ns_{0};
  bool has_champ_feedback_{false};
  rclcpp::Publisher<Bool>::SharedPtr pose_servo_active_pub_;
  rclcpp::Publisher<JointState>::SharedPtr pose_servo_joint_pub_;
  rclcpp::Subscription<JointState>::SharedPtr feedback_js_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr motor_init_timer_;
  rclcpp::TimerBase::SharedPtr verify_timer_;
  rclcpp::TimerBase::SharedPtr boot_report_timer_;
  bool boot_active_report_done_{false};
};

}  // namespace trotbot_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trotbot_can_bridge::PowerSequenceNode>());
  rclcpp::shutdown();
  return 0;
}

