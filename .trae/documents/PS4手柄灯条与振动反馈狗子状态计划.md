# PS4 手柄灯条 + 振动反馈狗子状态（计划）

## 1. Summary

在 `trotbot_ws` 中新增一个 Python ROS 2 节点 `ds4_feedback_node`，参考 `/home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py` 的 hidraw 实现，把手柄（DualShock 4）的**灯条颜色/闪烁/呼吸**与**左右马达振动**作为 TrotBot 的状态反馈通道：

- 颜色：复用现有 `trotbot_status_led/config/status_led_map.yaml` 的灯语（state + gate_open → RGB + effect），保证 WS2812 与手柄灯条语义一致。
- 振动：状态切换边沿触发（故障双震、下电强震、就绪/趴下轻震、起步轻震）。
- 行走/静止：`Running + gate_open` 时按 `/cmd_vel` 是否非零调节绿灯亮度。
- 电机故障：订阅 `/diagnostics` 中 `trotbot_can_bridge:motor_feedback:*` 的故障状态，触发红灯双闪 + 双震。
- 现有 WS2812 灯带（`trotbot_status_led` C++ 节点）**保留并可与手柄反馈并行运行**（用户已确认）。
- 无手柄/hidraw 不可用时节点不崩溃：只发调试 JSON 话题，便于无设备验收。

## 2. Current State Analysis

### 2.1 现有状态来源

- `power_sequence_node` 发布：
  - `/power_sequence/state`：`std_msgs/String`，取值 `Idle / Precheck / EnableInit / SoftStand / Running / SoftProne / ProneHold / Disable / Fault / Unknown`（`Fault` 枚举存在但当前无进入路径）。
  - `/power_sequence/gate_open`：`std_msgs/Bool`，门禁/使能透传标志。
  - 两个话题均为 `transient_local + reliable`（见 [power_sequence_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp#L147)）。
- `motor_protocol_node` 可选发布 `/diagnostics`（`publish_motor_diagnostics` 当前为 `false`），每条电机状态：
  - `st.name = "trotbot_can_bridge:motor_feedback:<joint>"`
  - 故障位置位时 `st.level = ERROR`，`message = "motor fault bits set"`，`values` 含 `fault_mask` / `fault_temp` 等（见 [motor_protocol_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/motor_protocol_node.cpp#L712-L765)）。
- `/cmd_vel`：`champ_teleop` 与 `power_sequence_node` 均会发布（后者在站/趴阶段发零速）。
- 手柄输入由 `joy_node` 读 `/dev/input/jsN` 发布 `/joy`；当前实机已连接 DS4（蓝牙，VID `0x054C` PID `0x05C4`，`/dev/hidraw2`，用户已在 `dialout` 组）。

### 2.2 现有灯语

- [status_led_map.yaml](file:///home/cat/trotbot_ws/src/trotbot_status_led/config/status_led_map.yaml) 已定义 `default_rgb` 与 `rules`（`state` / `gate` / `rgb` / `effect` / `breath_period_s`），C++ 节点按“先匹配先用”渲染。
- 现有 C++ 节点只驱动 WS2812，**没有任何手柄 hidraw 输出**；`/joy/set_feedback` 未被使用。

### 2.3 参考实现（a3_arm_ws）

- [ds4_feedback_node.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py) 已实机验证：
  - 自动扫描 `/dev/hidraw*`，按 `HID_ID` 识别 DS4（VID `054C`，PID `05C4/09CC/0BA0`），跳过 touchpad 接口。
  - USB 输出报告 `0x05` / 32 字节；蓝牙 `0x11` / 78 字节 + `0xC0` + CRC32（seed `0xA2`）。
  - 15 Hz 效果定时器、2 s 重扫设备、写入失败自动关闭重连、退出时熄灯停震。
  - `_Effect` 派生态（fault/offline/powering/ready/disabled/idle…）+ 灯效（solid/blink/double/breathe）+ 振动相位。
  - 无设备时仅发布 JSON 调试话题，节点存活。
- 本项目直接复用这套 HID 报告构造与设备扫描逻辑，去掉 a3 专属消息（`ArmStatus` / `MonitorStatus`），改为订阅本项目的电源状态、门禁、`/cmd_vel` 与 `/diagnostics`。

## 3. Proposed Changes

### 3.1 新增 `src/trotbot_status_led/scripts/ds4_feedback_node.py`

**为什么放这里**：与现有灯语同包，便于直接复用 `config/status_led_map.yaml`，避免新建包与重复配置；C++ WS2812 节点不受影响。

**节点结构**（单文件，约 400 行）：

1. **DS4 HID 层（照搬参考实现）**
   - 常量：`DS4_VID=0x054C`、`DS4_PIDS={0x05C4,0x09CC,0x0BA0}`、USB/BT 报告 ID 与长度、CRC seed `0xA2`。
   - `find_ds4_hidraw()`：扫描 `/dev/hidraw*`，读 `/sys/class/hidraw/<n>/device/uevent` 的 `HID_ID`，识别 DS4 并排除 Touchpad 接口。
   - `build_output_report(weak, strong, rgb, hid_bus)`：USB/BT 两种报告。
   - 打开/写入/断线重扫/退出熄灯逻辑同参考实现。

2. **灯语映射（复用 YAML）**
   - `state_map_file` 参数默认 `"config/status_led_map.yaml"`；相对路径通过 `ament_index_python.get_package_share_directory("trotbot_status_led")` 解析。
   - 解析 `default_rgb` 与 `rules`：`state`、`gate`（any/true/false）、`rgb`、`effect`（solid/breath/chase）、`breath_period_s`。
   - 匹配规则与 C++ 节点保持一致：按 YAML 顺序首条命中；`breath` 用正弦缩放；`chase` 当前按 solid 处理（与 C++ 一致）。
   - 最终 RGB = 映射色 × `global_brightness`（默认 `1.0`，手柄灯条亮度足够）。

3. **订阅与状态派生**
   - `/power_sequence/state`（`std_msgs/String`，`transient_local`）→ `power_state`。
   - `/power_sequence/gate_open`（`std_msgs/Bool`，`transient_local`）→ `gate_open`。
   - `/cmd_vel`（`geometry_msgs/Twist`）→ 行走判定：
     - `moving = |linear.x| > walk_linear_threshold || |linear.y| > walk_linear_threshold || |angular.z| > walk_angular_threshold`
     - 阈值默认：线速度 `0.02 m/s`、角速度 `0.05 rad/s`。
   - `/diagnostics`（`diagnostic_msgs/DiagnosticArray`）→ 电机故障：
     - 仅统计 `name` 以 `motor_diag_name_prefix`（默认 `trotbot_can_bridge:motor_feedback:`）开头的条目；
     - `level >= ERROR(2)` 或 `values` 中 `fault_mask != "0"` → `motor_fault = True`；
     - 最新一帧无故障则清除；从未收到诊断时保持 `False`。

4. **派生态（class）与灯色**
   - `fault`：`power_state=="Fault"` 或 `motor_fault` → 用 YAML `Fault` 规则（红）+ `double` 双闪。
   - 其余状态：直接按 `power_state + gate_open` 查 YAML；`Running + gate` 为绿。
   - **行走/静止亮度**：仅当 `power_state=="Running" && gate_open` 时，`moving` 用 1.0 亮度，`!moving` 用 `walk_stand_brightness`（默认 `0.35`）压暗；其它状态不受影响。
   - 启动时 `power_state` 为空、规则不命中 → 使用 `default_rgb`（暗灰），避免上电瞬间闪洋红。

5. **振动（边沿触发）**
   - 维护 `prev_cls`，仅进入新派生态时启动对应振动相位；参数化强度/时长：

     | 进入派生态 | 条件 | 振动 |
     |---|---|---|
     | `fault` | 电源 Fault 或电机诊断故障 | 强震 0.9 × 0.2 s，间隔 0.35 s 再来一次（双震） |
     | `offline` | `Idle/Disable/SoftProne` 且 `gate_open==false` | 强震 0.9 × 0.5 s |
     | `prone` | `ProneHold` | 弱震 0.5 × 0.12 s |
     | `ready` | `Running && gate_open` | 弱震 0.6 × 0.12 s（就绪确认） |
     | `powering` | `Precheck/EnableInit/SoftStand` | 不震动 |
     | `walk_start` | `ready` 且 `moving` 由 false→true | 弱震 0.35 × 0.08 s，冷却 1.0 s（`walk_start_rumble` 可关） |
   - 振动相位不依赖灯语定时器，统一由 15 Hz 效果定时器推进。

6. **输出与调试**
   - 15 Hz 定时器计算 RGB + 振动，`build_output_report()` 后写入 hidraw；报告不变则不写。
   - 调试话题 `/trotbot/ps4/feedback`（`std_msgs/String`，JSON），字段：
     `state, gate, cmd_linear_x, cmd_angular_z, moving, class, color, rgb, pattern, rumble_weak, rumble_strong, motor_fault, map_match, device`。
   - 无设备时每 `scan_period_s` 重扫，只 WARN 一次。

7. **参数**（全部有内置默认值，YAML 仅做显式配置）：
   `rate_hz=15.0`、`scan_period_s=2.0`、`hidraw_path=""`、`state_topic`、`gate_topic`、`cmd_vel_topic`、`diagnostics_topic`、`enable_motor_fault_feedback=true`、`motor_diag_name_prefix`、`feedback_topic`、`state_map_file`、`global_brightness=1.0`、`walk_linear_threshold`、`walk_angular_threshold`、`walk_stand_brightness`、`walk_start_rumble=true`、`walk_start_rumble_weak/duration_s/cooldown_s`、`rumble_enable_*`、`rumble_shutdown_*`、`rumble_fault_*`、`rumble_prone_*`。

### 3.2 新增 `src/trotbot_status_led/config/ps4_feedback_params.yaml`

节点名 `ds4_feedback_node`，写入 3.1 中全部参数，注释说明：
- `state_map_file` 复用现有灯语；
- `hidraw_path` 留空自动识别；
- 振动强度 0.0–1.0，可单独关掉某类振动；
- `walk_start_rumble` 可关闭。

### 3.3 修改 `src/trotbot_status_led/CMakeLists.txt`

新增：

```cmake
install(PROGRAMS scripts/ds4_feedback_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`config/` 与 `launch/` 已被现有 `install(DIRECTORY config launch docs ...)` 覆盖，无需另加。

### 3.4 修改 `src/trotbot_status_led/package.xml`

新增运行时依赖：

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>diagnostic_msgs</exec_depend>
<exec_depend>ament_index_python</exec_depend>
<exec_depend>python3-yaml</exec_depend>
```

### 3.5 新增 `src/trotbot_status_led/launch/ps4_feedback.launch.py`

- 参数 `params_file`，默认 `share/trotbot_status_led/config/ps4_feedback_params.yaml`。
- 启动 `Node(package="trotbot_status_led", executable="ds4_feedback_node", name="ds4_feedback_node", output="screen")`。
- 支持 `use_ps4_feedback` 开关，便于上层 include。

### 3.6 修改 `src/trotbot/launch/trotbot_teleop.launch.py`

- 新增 `use_ps4_feedback` 参数，默认 `"true"`。
- Include `trotbot_status_led/launch/ps4_feedback.launch.py`，条件 `IfCondition(use_ps4_feedback)`。
- 这样 `use_joystick:=true` 时手柄灯条/振动反馈随遥控一起启动；键盘-only 用户可传 `use_ps4_feedback:=false`（无手柄时节点本身也只 WARN 一次，不影响遥控）。

### 3.7 修改 `src/trotbot/launch/trotbot_basic.launch.py`

- 新增 `use_ps4_feedback` 启动参数（默认 `"true"`），传入 `trotbot_teleop.launch.py` 的 include 参数。
- 保持 `use_status_led` 不变，二者可同时为 true（用户确认 WS2812 保留并行）。

### 3.8 修改 `src/trotbot_can_bridge/config/control_gains.yaml`

- `publish_motor_diagnostics: false` → `true`，使电机故障能真实发布到 `/diagnostics`，供手柄红灯双闪/双震使用。
- 发布频率跟随现有反馈回调（约 10 Hz），仅多一组 `DiagnosticArray`，开销可忽略。
- 保留 `motor_diagnostics_topic: /diagnostics`。

### 3.9 文档更新

1. `src/trotbot_status_led/README.md`
   - 新增「PS4 手柄灯条 / 振动反馈」章节：
     - 状态 → 颜色（引用 `status_led_map.yaml`）；
     - 振动事件表；
     - 行走/静止亮度；
     - 电机故障诊断（说明 `publish_motor_diagnostics: true`）；
     - hidraw 权限（`dialout` 组或 udev 规则）；
     - 启动命令、独立 launch、调试话题；
     - 与 WS2812 并行说明（两节点同订阅电源话题，互不冲突）。
2. 根 `README.md`
   - §4.2 PS4 手柄：补充「手柄灯条/振动反馈默认随 teleop 启动」，以及 `use_ps4_feedback:=false` 关闭方式、编译命令加入 `trotbot_status_led`。
3. `docs/trotbot/PS4 按键说明.md`
   - 新增一节「手柄灯条/振动反馈」：颜色/振动对应电源状态与故障，链接到 `trotbot_status_led/README.md`。

## 4. Assumptions & Decisions

1. **保留 WS2812**：现有 C++ 节点、启动参数、灯语文件不改；PS4 反馈是新增并行通道（用户已确认）。
2. **灯语单一来源**：手柄灯条直接读取 `status_led_map.yaml`，不复制颜色表；后续改灯色只改一处。
3. **HID 实现复用 a3 已验证代码**：USB `0x05/32B`、蓝牙 `0x11/78B + CRC32`，不引入 `hidapi`/`evdev` 新系统依赖。
4. **状态反馈范围**：电源状态 + gate、电机故障诊断、行走/静止（用户三项全选）。
5. **振动默认值**：故障双震、下电强震、就绪/趴下轻震、起步轻震（`walk_start_rumble` 可关）；不做持续振动。
6. **设备缺失降级**：无 DS4/hidraw 时节点只发 `/trotbot/ps4/feedback`，不阻塞遥控与整机栈。
7. **不修改 a3_arm_ws**：仅参考其实现。
8. **Fault 派生态**：即使 `power_sequence` 当前不会进入 `Fault`，仍保留映射；实际故障主要由 `/diagnostics` 触发。

## 5. Verification

### 5.1 构建与静态检查

```bash
cd /home/cat/trotbot_ws
source /opt/ros/humble/setup.bash
python3 -m py_compile src/trotbot_status_led/scripts/ds4_feedback_node.py
colcon build --symlink-install --packages-select trotbot_status_led trotbot_can_bridge trotbot
source install/setup.bash
```

### 5.2 无设备降级验证

```bash
ros2 launch trotbot_status_led ps4_feedback.launch.py
# 预期：WARN 一次 "no DualShock 4 hidraw"，节点不退出
ros2 topic echo /trotbot/ps4/feedback
```

### 5.3 实机 DS4（当前为蓝牙 `/dev/hidraw2`）

```bash
# 终端 1
ros2 launch trotbot_status_led ps4_feedback.launch.py
# 终端 2
ros2 topic pub --once /power_sequence/gate_open std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /power_sequence/state std_msgs/msg/String "{data: Running}"
# 预期：灯条变绿；进入 Running 时一次轻震
ros2 topic pub --once /power_sequence/state std_msgs/msg/String "{data: Idle}"
# 预期：灯条变蓝；一次强震（0.5 s）

# 行走/静止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# 预期：绿灯高亮 + 起步轻震
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
# 预期：绿灯压暗至 35%

# 电机故障
ros2 topic pub --once /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  "{status: [{level: 2, name: 'trotbot_can_bridge:motor_feedback:base_lf1', message: 'motor fault bits set', values: [{key: 'fault_mask', value: '1'}]}]}"
# 预期：红灯双闪 + 双强震
ros2 topic echo /trotbot/ps4/feedback
```

### 5.4 集成启动验证

```bash
ros2 launch trotbot trotbot_basic.launch.py \
  use_can_bridge:=true use_teleop:=true use_joystick:=true \
  use_xterm:=false use_status_led:=true
ros2 node list | grep -E 'ds4_feedback|status_led'
```

预期：`ds4_feedback_node` 与 `status_led_node` 同时运行且互不影响；PS4 灯条随电源状态变化，WS2812 仍按原灯语工作。

### 5.5 回归检查

- 不启动 `use_ps4_feedback` 时，原遥控/整机启动行为不变。
- `publish_motor_diagnostics: true` 后 `/diagnostics` 有 12 条电机状态，`/joint_states_feedback` 等原有话题正常。
- 退出节点时灯条熄灭、振动停止（`destroy_node` 写零报告）。
