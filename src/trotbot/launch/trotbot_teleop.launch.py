#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Teleop Launch File
# Dedicated launch file for keyboard and joystick teleoperation of TrotBot
# This launch file provides the champ_teleop interface separately from the basic launch
#
# CONTROL INTERFACES:
# - Keyboard: Terminal-based keyboard control via champ_teleop
# - Joystick: Optional joystick integration with advanced pose control
# - Both interfaces can run simultaneously for flexibility
#
# USAGE:
#   Keyboard only:
#     ros2 launch trotbot trotbot_teleop.launch.py
#
#   Keyboard + Joystick:
#     ros2 launch trotbot trotbot_teleop.launch.py use_joystick:=true
#
#   Custom joystick device (/dev/input/jsN — N 会传给 ROS 2 joy_node 的 device_id):
#     ros2 launch trotbot trotbot_teleop.launch.py use_joystick:=true joystick_dev:=/dev/input/js1
#
# NOTE: This launch file only provides the teleop interface. You must also run:
#   ros2 launch trotbot trotbot_basic.launch.py
# to start the core TrotBot functionality (quadruped_controller, servo_interface, etc.)

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include_champ_teleop(context, *args, **kwargs):
    """从 joystick_dev 路径解析 js 编号，传给 champ_teleop 的 joy_device_id。"""
    joystick_dev = context.launch_configurations.get("joystick_dev", "/dev/input/js0")
    m = re.search(r"js(\d+)\s*$", joystick_dev)
    joy_id = m.group(1) if m else "0"

    champ_teleop_launch_path = PathJoinSubstitution(
        [FindPackageShare("champ_teleop"), "launch", "teleop.launch.py"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_teleop_launch_path),
            launch_arguments={
                "use_sim_time": context.launch_configurations.get("use_sim_time", "false"),
                "use_joy": context.launch_configurations.get("use_joystick", "false"),
                "joy_device_id": joy_id,
                "use_xterm": context.launch_configurations.get("use_xterm", "true"),
            }.items(),
        )
    ]


def generate_launch_description():
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    use_joystick_launch_arg = DeclareLaunchArgument(
        name="use_joystick",
        default_value="false",
        description="Enable joystick/gamepad control (default: false, keyboard only)",
    )

    joystick_dev_launch_arg = DeclareLaunchArgument(
        name="joystick_dev",
        default_value="/dev/input/js0",
        description="Joystick device path; 末尾 jsN 中的 N 会作为 joy_node 的 device_id",
    )

    use_xterm_launch_arg = DeclareLaunchArgument(
        name="use_xterm",
        default_value="true",
        description="是否用 xterm 跑键盘遥控（桌面默认 true）；无 DISPLAY 时设 false 并配合 ssh -t 或改用 /cmd_vel",
    )

    return LaunchDescription(
        [
            use_sim_time_launch_arg,
            use_joystick_launch_arg,
            joystick_dev_launch_arg,
            use_xterm_launch_arg,
            OpaqueFunction(function=_include_champ_teleop),
        ]
    )
