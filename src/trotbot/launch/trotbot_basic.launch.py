#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Basic Launch File
# Minimal launch for core TrotBot functionality: quadruped controller, state estimator, and CAN bridge
# This launch file provides the essential components needed for TrotBot operation without teleop interfaces
#
# CORE COMPONENTS:
# - quadruped_controller: Champ-based quadruped walking controller
# - state_estimator: Robot state estimation and odometry
# - can_bridge: MIT-only dual-node CAN bridge
#
# CONTROL OPTIONS:
# - Direct ROS 2 topics: Publish to /cmd_vel for velocity control
# - Separate teleop launch: Use trotbot_joystick.launch.py for gamepad control
# - Manual champ_teleop: Launch champ_teleop separately for keyboard control
#
# USAGE:
#   ros2 launch trotbot trotbot_basic.launch.py
#   # 启用 MIT-only CAN 双节点桥接：
#   ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true
#   # 同时打开 RViz（需 DISPLAY；加载 share/trotbot/rviz/trotbot.rviz；RobotModel 用 /robot_description 话题）：
#   ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true rviz:=true
#   ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true rviz:=true rviz_config:=/path/to/custom.rviz
#   ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true rviz:=true description_file:=minidog_champ.urdf.xacro gait_config_file:=gait_minidog.yaml
#   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trotbot_package = FindPackageShare("trotbot")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    has_imu = LaunchConfiguration("has_imu")
    has_imu_launch_arg = DeclareLaunchArgument(
        name="has_imu",
        default_value="false",
        description="Enable IMU sensor support (default: false for TrotBot)"
    )

    use_can_bridge = LaunchConfiguration("use_can_bridge")
    use_can_bridge_launch_arg = DeclareLaunchArgument(
        name="use_can_bridge",
        default_value="false",
        description="是否启动 MIT-only 双节点 CAN bridge",
    )

    rviz = LaunchConfiguration("rviz")
    rviz_launch_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="false",
        description="为 true 时启动 robot_state_publisher 与 rviz2（需图形界面与已安装 ros-humble-rviz2）",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution(
        [trotbot_package, "rviz", "trotbot.rviz"]
    )
    rviz_config_launch_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config,
        description="传给 rviz2 -d 的配置文件路径（默认本包 rviz/trotbot.rviz）",
    )

    description_file = LaunchConfiguration("description_file")
    description_file_launch_arg = DeclareLaunchArgument(
        name="description_file",
        default_value="trotbot.urdf.xacro",
        description="URDF/xacro 文件名，位于 share/trotbot/urdf/，与 champ 控制器共用（如 minidog_champ.urdf.xacro）",
    )

    gait_config_file = LaunchConfiguration("gait_config_file")
    gait_config_file_launch_arg = DeclareLaunchArgument(
        name="gait_config_file",
        default_value="gait.yaml",
        description="Champ 步态 yaml 文件名，位于 share/trotbot/config/champ/（Minidog 建议 gait_minidog.yaml）",
    )

    description_path = PathJoinSubstitution(
        [trotbot_package, "urdf", description_file]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([
                        "xacro ", description_path,
                        " model_r:=0.15 model_g:=0.95 model_b:=0.25 model_a:=1.0"
                    ]),
                    value_type=str,
                ),
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(rviz),
    )

    # 反馈模型：订阅 /joint_states_feedback，发布带前缀 fb/ 的 TF，便于 RViz 与期望模型并行对比
    robot_state_publisher_feedback_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_feedback",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([
                        "xacro ", description_path,
                        " model_r:=0.95 model_g:=0.2 model_b:=0.85 model_a:=0.35"
                    ]),
                    value_type=str,
                ),
                "frame_prefix": "fb/",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("/joint_states", "/joint_states_feedback"),
            ("/robot_description", "/robot_description_feedback"),
        ],
        condition=IfCondition(rviz),
    )

    feedback_tf_bridge_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="feedback_tf_bridge",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "fb/base_link"],
        condition=IfCondition(rviz),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    # Include Champ controllers launch
    champ_controllers_launch_path = PathJoinSubstitution(
        [trotbot_package, "launch", "champ_controllers.launch.py"]
    )
    champ_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(champ_controllers_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "has_imu": has_imu,
            "description_file": description_file,
            "gait_config_file": gait_config_file,
        }.items(),
    )

    # Include CAN bridge launch
    can_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("trotbot_can_bridge"), "launch", "can_bridge.launch.py"])
        ),
        condition=IfCondition(use_can_bridge),
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_launch_arg,
        has_imu_launch_arg,
        use_can_bridge_launch_arg,
        rviz_launch_arg,
        rviz_config_launch_arg,
        description_file_launch_arg,
        gait_config_file_launch_arg,

        # Core TrotBot functionality
        champ_controllers_launch,    # Champ quadruped controller + state estimator
        can_bridge_launch,           # MIT-only dual-node CAN bridge
        robot_state_publisher_node,
        robot_state_publisher_feedback_node,
        feedback_tf_bridge_node,
        rviz_node,
    ])
