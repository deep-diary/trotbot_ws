import launch_ros
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joy = LaunchConfiguration("use_joy")
    joy_device_id = LaunchConfiguration("joy_device_id")

    declare_use_joy = DeclareLaunchArgument(
        "use_joy", default_value="false", description="Use joy or keyboard"
    )
    declare_joy_device_id = DeclareLaunchArgument(
        "joy_device_id",
        default_value="0",
        description="ROS 2 joy_node device_id (0=/dev/input/js0, 1=js1, ...)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    use_xterm = LaunchConfiguration("use_xterm")
    declare_use_xterm = DeclareLaunchArgument(
        "use_xterm",
        default_value="true",
        description="若 true，在 xterm 中运行键盘遥控（需 DISPLAY）；无桌面可设 false 并在真实 TTY（如 ssh -t）下运行",
    )

    block_body_pose = LaunchConfiguration("block_body_pose_until_running")
    declare_block_body_pose = DeclareLaunchArgument(
        "block_body_pose_until_running",
        default_value="true",
        description="真机默认 true：未进入 Running 前屏蔽 body_pose；无电源序列的仿真/台架可设 false",
    )

    joy = Node(
        package="joy",
        output="screen",
        executable="joy_node",
        condition=IfCondition(use_joy),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"device_id": ParameterValue(joy_device_id, value_type=int)},
            {"autorepeat_rate": 5.0},
        ],
        # 蓝牙 DS4 在 SDL2 HIDAPI 下会变成「PS4 Controller」6 轴/按键重排，
        # 与 power_sequence/champ_teleop 使用的 evdev 布局（□=3、Options=9 等）不符。
        # 强制走 evdev，保证本仓库的按键映射生效。
        additional_env={"SDL_JOYSTICK_HIDAPI": "0"},
    )

    champ_teleop_xterm = Node(
        package="champ_teleop",
        executable="champ_teleop.py",
        prefix="xterm -e",
        emulate_tty=True,
        condition=IfCondition(use_xterm),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"joy": use_joy},
            {"block_body_pose_until_running": ParameterValue(block_body_pose, value_type=bool)},
        ],
    )
    champ_teleop_inline = Node(
        package="champ_teleop",
        executable="champ_teleop.py",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_xterm),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"joy": use_joy},
            {"block_body_pose_until_running": ParameterValue(block_body_pose, value_type=bool)},
        ],
    )
    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_use_joy,
            declare_joy_device_id,
            declare_use_xterm,
            declare_block_body_pose,
            joy,
            champ_teleop_xterm,
            champ_teleop_inline,
        ]
    )
