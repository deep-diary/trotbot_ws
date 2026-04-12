#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Servo Interface Launch File
# Launches the servo interface bridge between ROS 2 and hardware

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trotbot',
            executable='servo_interface',
            name='servo_interface',
            output='screen'
        )
    ])
