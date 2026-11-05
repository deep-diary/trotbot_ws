# trotbot_ws

四足机器狗工作空间：ROS 2 Humble + CHAMP 步态 + 12× EL05 MIT 电机（LubanCat-4-V1 双 CAN）。本 README 只保留高频部署 / 操作命令，细节见下方文档导航。

## 文档导航

| 文档 | 内容 |
|------|------|
| [docs/trotbot/README.md](docs/trotbot/README.md) | 协作型文档总入口（需求 / 架构 / 问题跟踪 / 路线） |
| [docs/trotbot/CAN配置与报文调试.md](docs/trotbot/CAN配置与报文调试.md) | 设备树 overlay、`can-up.service`、1Mbps 配置、`cansend` / `candump` |
| [docs/trotbot/遥控与电源启停.md](docs/trotbot/遥控与电源启停.md) | `power_sequence` 话题与状态语义、PS4 / 键盘遥控、轴映射 |
| [docs/trotbot/CAN桥架构与话题细节.md](docs/trotbot/CAN桥架构与话题细节.md) | 双节点桥接架构、话题消息定义、发送统计日志 |
| [docs/trotbot/CAN频率与负载.md](docs/trotbot/CAN频率与负载.md) | 轨迹频率 → CAN 帧率 / 带宽估算与调参 |
| [docs/trotbot/模型重力前馈.md](docs/trotbot/模型重力前馈.md) | Pinocchio 重力前馈（RQ-025，默认关闭） |
| [docs/trotbot/部署与开机自启.md](docs/trotbot/部署与开机自启.md) | systemd 重启 / 开机自启、灯带 sudo 说明 |
| [src/trotbot/本地运行指南.md](src/trotbot/本地运行指南.md) | 安装、编译、CHAMP 补丁、排障 |
| [src/trotbot_can_bridge/README.md](src/trotbot_can_bridge/README.md) | 电机脚本、MIT 调参、协议细节 |

## 编译（工作空间根目录执行）

先加载 ROS 2 环境（以 Humble 为例，按你本机发行版修改）：

```bash
source /opt/ros/humble/setup.bash
cd /home/cat/trotbot_ws
```

只编译常用功能包（`trotbot`、`trotbot_can_bridge`、`trotbot_dynamics`）：

```bash
colcon build --packages-select trotbot trotbot_can_bridge trotbot_dynamics --symlink-install
```

编译整个工作空间：

```bash
colcon build --symlink-install
```

编译完成后加载覆盖层：

```bash
source install/setup.bash
```

## 启动与停止（推荐 `main.sh`）

工作空间根目录的 `main.sh` 是**统一入口**，支持 `start / stop / restart / status`（**默认 `restart`**）以及 `motor / calib / led-test`。

```bash
cd ~/trotbot_ws

./main.sh                 # 默认真机 restart
./main.sh start --sim     # 无硬件台架仿真 start
./main.sh --sim           # 仿真 restart（不启动 CAN/灯带/PS4 反馈，带 RViz）
./main.sh stop
./main.sh status
./main.sh check           # 启动自检：CAN 桥节点 + 12 个电机是否都在线（真机 start 后自动执行）

# 额外参数原样追加给 ros2 launch（后出现的同名参数覆盖默认值）
./main.sh restart use_status_led:=false
```

- 默认真机参数：`use_can_bridge:=true use_teleop:=true use_joystick:=true use_xterm:=false rviz:=false use_status_led:=true block_body_pose_until_running:=true`。
- `--sim` 参数：`use_can_bridge:=false use_teleop:=true use_joystick:=true use_xterm:=false rviz:=true use_status_led:=false use_ps4_feedback:=false block_body_pose_until_running:=false`。
- 日志：`<workspace>/log/trotbot_ctl/stack.log`；PID：`<workspace>/log/trotbot_ctl/stack.pid`。
- `stop/restart` 会清理本脚本启动的进程组，并兜底结束残留的 `trotbot_basic`、`champ_teleop`、`joy_node`、`ds4_feedback_node`、`status_led_node`、`power_sequence_node` 等相关进程（**包括手动启动的**）。
- 工作空间可用 `TROTBOT_WS` 指定。

> 仿真模式不启动 `power_sequence_node`，因此 **L3（上电）/ □（站起）/ ○（趴下）/ R3（下电）均不会响应**；摇杆步态、△/✕ 高度、右摇杆姿态可用。△/✕ 高度增量限制为 `body_z_max=+0.015 m`、`body_z_min=-0.08 m`。

`main.sh` 只是转发到 `src/trotbot/scripts/trotbot_ctl.sh`；也可直接调用后者，或安装后调用：

```bash
bash "$(ros2 pkg prefix trotbot)/share/trotbot/scripts/trotbot_ctl.sh" restart
```

### 直接 `ros2 launch`（不用 main.sh）

```bash
source /opt/ros/humble/setup.bash
cd /home/cat/trotbot_ws
colcon build --packages-select trotbot_can_bridge trotbot --symlink-install
source install/setup.bash
ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true rviz:=true
```

默认已使用 **`minidog_champ.urdf.xacro`** 与 **`gait_minidog.yaml`**（与 `trotbot_basic.launch.py` / `champ_controllers.launch.py` 一致）。若需旧款 `trotbot.urdf.xacro` 整机，请显式传入 `description_file:=trotbot.urdf.xacro gait_config_file:=gait.yaml`。

一条命令带起 `can_bridge + teleop`（单终端联调）：

```bash
ros2 launch trotbot trotbot_basic.launch.py \
  use_can_bridge:=true \
  use_teleop:=true \
  use_joystick:=true \
  use_xterm:=false \
  rviz:=false \
  use_status_led:=true
```

仅启动 CAN 桥节点（不跑 CHAMP、不测机器狗）：

```bash
ros2 launch trotbot_can_bridge can_bridge.launch.py
# 接口名不一致时覆盖
ros2 launch trotbot_can_bridge can_bridge.launch.py can0_name:=can0 can1_name:=can1
```

## 改代码后：编译 + systemd 重启

```bash
cd ~/trotbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding champ champ_msgs --packages-up-to trotbot trotbot_status_led trotbot_can_bridge trotbot_dynamics
sudo systemctl restart trotbot-basic
sudo systemctl status trotbot-basic --no-pager
```

服务名以 `/etc/systemd/system/` 下实际 `.service` 文件名为准；开机自启与灯带 `sudo` 注意事项见 [部署与开机自启](docs/trotbot/部署与开机自启.md)。

## 查看话题与状态

```bash
ros2 topic list
ros2 node list
ros2 topic info /joint_states_feedback
ros2 topic hz /joint_states_feedback
```

**12 个电机角度：`/joint_states_feedback`**（`sensor_msgs/msg/JointState`，默认约 50Hz）。`position` 为 12 个关节角，单位 **rad**，顺序为 CHAMP：
`base_lf1, lf1_lf2, lf2_lf3, base_rf1, rf1_rf2, rf2_rf3, base_lb1, lb1_lb2, lb2_lb3, base_rb1, rb1_rb2, rb2_rb3`。

```bash
ros2 topic echo /joint_states_feedback --once
ros2 topic echo /joint_states_feedback --field position   # 只看 12 个角度
```

- 前提：`motor_protocol_node` 在运行，且参数 `publish_feedback_joint_states` 为 `true`（默认）。
- 尚未收到某关节反馈时该位会填 `0.0`，不要把 `0` 当成真实角度。

**单电机原始反馈：`/motor_feedback`**（逐帧一行）：

```bash
ros2 topic echo /motor_feedback
# 例：bus=1 motor=53 mode=1 angle=0.123 speed=-0.045 torque=0.210 temp=36 err=0
```

**反馈监控节点**：订阅 `/motor_feedback`，按关节打印位置、速度、力矩、温度，并记录本次订阅以来每个关节的最大力矩。

```bash
# output_mode: torque（默认）| position | both | speed | temp | speed_temp | all
ros2 run trotbot_can_bridge motor_monitor_node --ros-args \
  -p output_mode:=all \
  -p motor_ids:="[13,23,53,63]" \
  -p decimals:=2 \
  -p print_hz:=2.0
```

- 只看速度：`-p output_mode:=speed`；只看温度：`-p output_mode:=temp`；两者一起：`speed_temp`。
- `motor_ids` 为空时显示全部 12 个电机；`decimals` 控制小数位（0–6）；`print_hz` 控制汇总显示频率。
- 依赖 `/motor_feedback`，即 `motor_protocol_node` 的 `enable_rx_decode_log` 需为 `true`（默认）。

常用组合（只看 13/23/53/63）：

```bash
# 位置
ros2 run trotbot_can_bridge motor_monitor_node --ros-args \
  -p output_mode:=position -p motor_ids:="[13,23,53,63]" -p decimals:=2 -p print_hz:=2.0

# 速度
ros2 run trotbot_can_bridge motor_monitor_node --ros-args \
  -p output_mode:=speed -p motor_ids:="[13,23,53,63]" -p decimals:=3 -p print_hz:=2.0

# 温度
ros2 run trotbot_can_bridge motor_monitor_node --ros-args \
  -p output_mode:=temp -p motor_ids:="[13,23,53,63]" -p decimals:=1 -p print_hz:=2.0
```

**电源状态机**：

```bash
ros2 topic echo /power_sequence/state --once
ros2 topic echo /power_sequence/gate_open --once
```

**CAN 报文**：

```bash
candump -tz can0          # 前腿
candump -tz can1          # 后腿
candump -tz can0 can1     # 两路同时

ros2 topic echo /can_tx_frames     # MIT 下发（15 字节打包）
ros2 topic echo /can_rx_frames     # 电机反馈 / 主动上报
```

`/can_tx_frames`、`/can_rx_frames` 是 15 字节打包帧，只适合排查链路，不便于读角度。

**看频率 / 负载**：

```bash
ros2 topic hz /joint_group_effort_controller/joint_trajectory
ros2 topic hz /can_tx_frames
ros2 topic hz /can_rx_frames
ip -details -statistics link show can0
ip -details -statistics link show can1
```

话题字段、单行十六进制输出（`/can_frames_tx_line`）与发送统计日志见 [CAN桥架构与话题细节](docs/trotbot/CAN桥架构与话题细节.md)；频率关系与推荐值见 [CAN频率与负载](docs/trotbot/CAN频率与负载.md)。

## CAN 快速检查

```bash
ip -br link
ip -details link show can0
ip -details link show can1
```

若接口已存在但未 `up`，临时配置 1Mbps：

```bash
sudo ip link set can0 down || true
sudo ip link set can1 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 up
sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
ip -details -statistics link show can0
```

看不到 `can0/can1` 时，需先修复设备树 overlay（`/boot/firmware/ubuntuEnv.txt`）；开机自动 `up` 与 `cansend` 报文见 [CAN配置与报文调试](docs/trotbot/CAN配置与报文调试.md)。

## 遥控（简表）

```bash
ros2 launch trotbot trotbot_teleop.launch.py                           # 默认手柄 + 键盘（xterm）
ros2 launch trotbot trotbot_teleop.launch.py use_joystick:=false       # 纯键盘
ros2 launch trotbot trotbot_teleop.launch.py use_xterm:=false          # 无桌面 / SSH
ros2 launch trotbot trotbot_teleop.launch.py joystick_dev:=/dev/input/js0
```

PS4 默认按键（短按 = 按下沿）：

| 按键 | 作用 |
|------|------|
| `L3` 短按 | 上电 / 使能 `start`（唯一上电入口） |
| `□` 短按 | `ProneHold` → 站起 |
| `○` 短按 | `Running` → 趴下（保持使能） |
| `R3` 短按 | 下电 `shutdown`（唯一下电入口） |
| `L1+R1` 长按 3s | 未上电且门禁关时 `motor_init`（设零 + 保存，写后断电重启生效） |

无手柄时可用 `/power_sequence/command` 发 `start / prone / shutdown / set_zero / motor_init`，状态语义、键盘 / 手柄轴映射与 `/joy` 排查见 [遥控与电源启停](docs/trotbot/遥控与电源启停.md)。

## 常用维护

```bash
./main.sh motor ping
./main.sh motor zero_sta_read
./main.sh motor --help

./main.sh calib           # set_zero（当前姿态=机械零位）+ zero_sta=1 + 保存 flash，需交互确认
./main.sh calib -y        # 跳过确认（仍要求 state=Idle、门禁关闭）
./main.sh led-test        # 单独测试 WS2812 灯带
```

> 涉及真实电机写参数 / 设零 / 保存 flash：必须 **Idle + 门禁关闭 + 零位工装姿态**，写后需断电重启生效；不要与 `cansend` 脚本并发抢总线。
