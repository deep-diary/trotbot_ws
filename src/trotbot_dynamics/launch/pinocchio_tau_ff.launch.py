#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    publish_hz = LaunchConfiguration("publish_hz")
    output_topic = LaunchConfiguration("output_topic")

    declare_description_file = DeclareLaunchArgument(
        "description_file",
        default_value="minidog_champ.urdf.xacro",
        description="URDF/xacro 文件名，位于 share/trotbot/urdf/",
    )
    declare_publish_hz = DeclareLaunchArgument(
        "publish_hz",
        default_value="50.0",
        description="模型前馈发布频率（Hz）",
    )
    declare_output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/model_tau_ff",
        description="模型重力前馈输出话题（sensor_msgs/JointState，仅 name+effort）",
    )

    description_path = PathJoinSubstitution(
        [FindPackageShare("trotbot"), "urdf", description_file]
    )

    pinocchio_tau_ff_node = Node(
        package="trotbot_dynamics",
        executable="pinocchio_tau_ff_node",
        name="pinocchio_tau_ff_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("trotbot_dynamics"), "config", "pinocchio_tau_ff.yaml"]
            ),
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", description_path]),
                    value_type=str,
                ),
                "publish_hz": ParameterValue(publish_hz, value_type=float),
                "output_topic": output_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            declare_description_file,
            declare_publish_hz,
            declare_output_topic,
            pinocchio_tau_ff_node,
        ]
    )
