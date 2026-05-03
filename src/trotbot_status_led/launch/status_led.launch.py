#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# WS2812 状态灯（RQ-023）。可与整机 launch 组合；默认本文件内 use_status_led=true。
# 从 trotbot_basic 引入时通常传 use_status_led:=false 作为默认，避免无硬件开发机失败。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("trotbot_status_led")
    params_file = LaunchConfiguration("params_file")
    use_status_led = LaunchConfiguration("use_status_led")

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg, "config", "status_led_params.yaml"]),
        description="status_led_node 参数 YAML（含 gpiochip、gpio_line_offset、state_map_file 等）",
    )
    declare_use = DeclareLaunchArgument(
        "use_status_led",
        default_value="true",
        description="为 true 时启动 status_led_node；整机场景可由上层 launch 默认 false",
    )

    status_led_node = Node(
        package="trotbot_status_led",
        executable="status_led_node",
        name="status_led_node",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(use_status_led),
    )

    return LaunchDescription(
        [
            declare_params,
            declare_use,
            status_led_node,
        ]
    )
