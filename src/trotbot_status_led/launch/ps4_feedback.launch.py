#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# PS4（DualShock 4）灯条 + 振动反馈。可单独启动，也可由 trotbot_teleop 引入。
# 无手柄时节点不退出，只发布 /trotbot/ps4/feedback（JSON）。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("trotbot_status_led")
    # 注意：不要叫 params_file，避免与 status_led.launch.py 的同名参数串值
    ps4_params_file = LaunchConfiguration("ps4_params_file")
    use_ps4_feedback = LaunchConfiguration("use_ps4_feedback")

    declare_params = DeclareLaunchArgument(
        "ps4_params_file",
        default_value=PathJoinSubstitution([pkg, "config", "ps4_feedback_params.yaml"]),
        description="ds4_feedback_node 参数 YAML",
    )
    declare_use = DeclareLaunchArgument(
        "use_ps4_feedback",
        default_value="true",
        description="是否启动 PS4 灯条/振动反馈节点",
    )

    feedback_node = Node(
        package="trotbot_status_led",
        executable="ds4_feedback_node",
        name="ds4_feedback_node",
        output="screen",
        parameters=[ps4_params_file],
        condition=IfCondition(use_ps4_feedback),
    )

    return LaunchDescription(
        [
            declare_params,
            declare_use,
            feedback_node,
        ]
    )
