# trotbot_ws

## 重新编译（工作空间根目录执行）

先加载 ROS 2 环境（以 Humble 为例，按你本机发行版修改）：

```bash
source /opt/ros/humble/setup.bash
cd /home/cat/trotbot_ws
```

只编译 `trotbot` 功能包（改 URDF、launch、meshes 等后常用）：

```bash
colcon build --packages-select trotbot trotbot_can_bridge --symlink-install
```

编译整个工作空间：

```bash
colcon build --symlink-install
```

编译完成后加载覆盖层：

```bash
source install/setup.bash
```

## Minidog 模型：RViz + 键盘遥控

无舵机硬件时关闭 `servo_interface`，并使用 Champ 兼容的 Minidog URDF（`minidog_champ.urdf.xacro`）。`gait_minidog.yaml` 略提高 `nominal_height`，站姿更舒展（与默认 `trotbot` 的 `gait.yaml` 分离）。

**终端 1：Minidog 模型 + RViz**

```bash
ros2 launch trotbot trotbot_basic.launch.py use_servo_interface:=false rviz:=true description_file:=minidog_champ.urdf.xacro gait_config_file:=gait_minidog.yaml
```

**终端 2：键盘遥控**

```bash
ros2 launch trotbot trotbot_teleop.launch.py
```

使用前请在工作空间根目录执行 `source install/setup.bash`（或你本机的 ROS 2 环境配置）。
