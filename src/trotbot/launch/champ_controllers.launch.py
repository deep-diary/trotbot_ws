#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Champ Controllers Launch File
# Launches the Champ quadruped controller and state estimator

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_package = FindPackageShare("trotbot")

    description_file = LaunchConfiguration("description_file")
    description_path = PathJoinSubstitution(
        [description_package, "urdf", description_file]
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", "joints.yaml"]
    )
    links_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", "links.yaml"]
    )
    gait_config_file = LaunchConfiguration("gait_config_file")
    gait_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", gait_config_file]
    )

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
        description="if the robot has imu sensor"
    )

    description_file_launch_arg = DeclareLaunchArgument(
        name="description_file",
        default_value="minidog_champ.urdf.xacro",
        description="URDF/xacro 文件名，位于 share/trotbot/urdf/；默认 minidog（旧款整机可传 trotbot.urdf.xacro）",
    )

    gait_config_file_launch_arg = DeclareLaunchArgument(
        name="gait_config_file",
        default_value="gait_minidog.yaml",
        description="Champ 步态参数文件名，位于 share/trotbot/config/champ/；默认与 minidog 配套（旧配置可传 gait.yaml）",
    )

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": use_sim_time},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": True},
            {
                "joint_controller_topic": "joint_group_effort_controller/joint_trajectory"
            },
            {
                "urdf": ParameterValue(
                    Command(["xacro ", description_path]),
                    value_type=str,
                )
            },
            joints_config_path,
            links_config_path,
            gait_config_path,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": has_imu},
            {
                "urdf": ParameterValue(
                    Command(["xacro ", description_path]),
                    value_type=str,
                )
            },
            joints_config_path,
            links_config_path,
            gait_config_path,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_launch_arg,
            has_imu_launch_arg,
            description_file_launch_arg,
            gait_config_file_launch_arg,
            quadruped_controller,
            state_estimator,
        ]
    )
