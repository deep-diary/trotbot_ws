#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# TrotBot Champ Controllers Launch File
# Launches the Champ quadruped controller and state estimator

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_package = FindPackageShare("trotbot")

    description_path = PathJoinSubstitution(
        [description_package, "urdf", "trotbot.urdf.xacro"]
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", "joints.yaml"]
    )
    links_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", "links.yaml"]
    )
    gait_config_path = PathJoinSubstitution(
        [description_package, "config", "champ", "gait.yaml"]
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
            {"urdf": Command(["xacro ", description_path])},
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
            {"urdf": Command(["xacro ", description_path])},
            joints_config_path,
            links_config_path,
            gait_config_path,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_launch_arg,
            has_imu_launch_arg,
            quadruped_controller,
            state_estimator,
        ]
    )
