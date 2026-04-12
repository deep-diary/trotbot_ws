#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Basic Launch File
# Minimal launch for core TrotBot functionality: quadruped controller, state estimator, and servo interface
# This launch file provides the essential components needed for TrotBot operation without teleop interfaces
#
# CORE COMPONENTS:
# - quadruped_controller: Champ-based quadruped walking controller
# - state_estimator: Robot state estimation and odometry
# - servo_interface: Hardware abstraction layer for Pi 5 + PCA9685 servo control
#
# CONTROL OPTIONS:
# - Direct ROS 2 topics: Publish to /cmd_vel for velocity control
# - Separate teleop launch: Use trotbot_joystick.launch.py for gamepad control
# - Manual champ_teleop: Launch champ_teleop separately for keyboard control
#
# USAGE:
#   ros2 launch trotbot trotbot_basic.launch.py
#   # 无 PCA9685 / sysfs PWM 时避免刷屏，可关闭舵机节点（仅步态+状态估计+发关节指令到“空”）：
#   ros2 launch trotbot trotbot_basic.launch.py use_servo_interface:=false
#   # 同时打开 RViz（需 DISPLAY；加载 share/trotbot/rviz/trotbot.rviz；RobotModel 用 /robot_description 话题）：
#   ros2 launch trotbot trotbot_basic.launch.py use_servo_interface:=false rviz:=true
#   ros2 launch trotbot trotbot_basic.launch.py use_servo_interface:=false rviz:=true rviz_config:=/path/to/custom.rviz
#   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    use_servo_interface = LaunchConfiguration("use_servo_interface")
    use_servo_interface_launch_arg = DeclareLaunchArgument(
        name="use_servo_interface",
        default_value="true",
        description="是否启动 servo_interface（无硬件时请设 false，避免 sysfs PWM 报错刷屏）",
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

    description_path = PathJoinSubstitution(
        [trotbot_package, "urdf", "trotbot.urdf.xacro"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", description_path]),
                "use_sim_time": use_sim_time,
            }
        ],
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
            "has_imu": has_imu
        }.items(),
    )

    # Include servo interface launch
    servo_interface_launch_path = PathJoinSubstitution(
        [trotbot_package, "launch", "servo_interface.launch.py"]
    )
    servo_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_interface_launch_path),
        condition=IfCondition(use_servo_interface),
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_launch_arg,
        has_imu_launch_arg,
        use_servo_interface_launch_arg,
        rviz_launch_arg,
        rviz_config_launch_arg,

        # Core TrotBot functionality
        champ_controllers_launch,    # Champ quadruped controller + state estimator
        servo_interface_launch,      # TrotBot servo interface (Pi 5 + PCA9685)
        robot_state_publisher_node,
        rviz_node,
    ])
