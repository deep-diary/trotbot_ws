#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_package = FindPackageShare("trotbot_can_bridge")

    can0_name = LaunchConfiguration("can0_name")
    can1_name = LaunchConfiguration("can1_name")
    gains_file = LaunchConfiguration("gains_file")
    bridge_file = LaunchConfiguration("bridge_file")

    declare_can0 = DeclareLaunchArgument(
        "can0_name", default_value="can0", description="SocketCAN interface name for front legs"
    )
    declare_can1 = DeclareLaunchArgument(
        "can1_name", default_value="can1", description="SocketCAN interface name for rear legs"
    )
    declare_gains = DeclareLaunchArgument(
        "gains_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "control_gains.yaml"]),
        description="MIT gain parameters for motor_protocol_node",
    )
    declare_bridge = DeclareLaunchArgument(
        "bridge_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "bridge.yaml"]),
        description="SocketCAN transport parameters",
    )

    can_transport = Node(
        package="trotbot_can_bridge",
        executable="can_transport_node",
        name="can_transport_node",
        output="screen",
        parameters=[
            bridge_file,
            {"can0_name": can0_name},
            {"can1_name": can1_name},
        ],
    )

    motor_protocol = Node(
        package="trotbot_can_bridge",
        executable="motor_protocol_node",
        name="motor_protocol_node",
        output="screen",
        parameters=[gains_file],
    )

    return LaunchDescription(
        [
            declare_can0,
            declare_can1,
            declare_gains,
            declare_bridge,
            can_transport,
            motor_protocol,
        ]
    )
