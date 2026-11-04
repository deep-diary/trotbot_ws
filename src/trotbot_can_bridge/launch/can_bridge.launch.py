#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_package = FindPackageShare("trotbot_can_bridge")

    can0_name = LaunchConfiguration("can0_name")
    can1_name = LaunchConfiguration("can1_name")
    gains_file = LaunchConfiguration("gains_file")
    bridge_file = LaunchConfiguration("bridge_file")
    power_sequence_file = LaunchConfiguration("power_sequence_file")
    use_power_sequence = LaunchConfiguration("use_power_sequence")
    use_pinocchio_tau_ff = LaunchConfiguration("use_pinocchio_tau_ff")
    description_file = LaunchConfiguration("description_file")
    pinocchio_tau_ff_scale = LaunchConfiguration("pinocchio_tau_ff_scale")
    # 必须用 ParameterValue(..., value_type=str) 包一层：
    # 直接传字符串时 launch 会按 YAML 解析，'off' 会被解析成 bool False，
    # 而节点声明的是 std::string，启动即抛 InvalidParameterTypeException 退出。
    pinocchio_tau_ff_mode = ParameterValue(
        PythonExpression(["'replace' if '", use_pinocchio_tau_ff, "' == 'true' else 'off'"]),
        value_type=str,
    )

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
    declare_power_sequence_file = DeclareLaunchArgument(
        "power_sequence_file",
        default_value=PathJoinSubstitution([bridge_package, "config", "power_sequence.yaml"]),
        description="Power sequence state machine parameters",
    )
    declare_use_power_sequence = DeclareLaunchArgument(
        "use_power_sequence",
        default_value="true",
        description="是否启用遥控上下电状态机节点 power_sequence_node",
    )
    declare_use_pinocchio_tau_ff = DeclareLaunchArgument(
        "use_pinocchio_tau_ff",
        default_value="false",
        description="是否启动 trotbot_dynamics 的 Pinocchio 定基重力前馈（RQ-025）",
    )
    declare_description_file = DeclareLaunchArgument(
        "description_file",
        default_value="minidog_champ.urdf.xacro",
        description="Pinocchio 模型使用的 URDF/xacro（位于 share/trotbot/urdf/）",
    )
    declare_pinocchio_tau_ff_scale = DeclareLaunchArgument(
        "pinocchio_tau_ff_scale",
        default_value="0.5",
        description="模型重力前馈总增益（建议 < 1）",
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
        parameters=[
            gains_file,
            {"enable_power_sequence_gate": use_power_sequence},
            {
                "pinocchio_tau_ff_mode": pinocchio_tau_ff_mode,
                "pinocchio_tau_ff_scale": ParameterValue(
                    pinocchio_tau_ff_scale, value_type=float
                ),
            },
        ],
    )

    power_sequence = Node(
        package="trotbot_can_bridge",
        executable="power_sequence_node",
        name="power_sequence_node",
        output="screen",
        parameters=[power_sequence_file],
        condition=IfCondition(use_power_sequence),
    )

    pinocchio_tau_ff_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("trotbot_dynamics"),
                    "launch",
                    "pinocchio_tau_ff.launch.py",
                ]
            )
        ),
        launch_arguments={"description_file": description_file}.items(),
        condition=IfCondition(use_pinocchio_tau_ff),
    )

    return LaunchDescription(
        [
            declare_can0,
            declare_can1,
            declare_gains,
            declare_bridge,
            declare_power_sequence_file,
            declare_use_power_sequence,
            declare_use_pinocchio_tau_ff,
            declare_description_file,
            declare_pinocchio_tau_ff_scale,
            can_transport,
            motor_protocol,
            power_sequence,
            pinocchio_tau_ff_launch,
        ]
    )
