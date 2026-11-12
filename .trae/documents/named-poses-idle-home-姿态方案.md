# named_poses（idle/home）姿态录制与平滑切换方案

## 1. Summary

给狗子增加 `named_poses`（命名姿态）能力，并重做上/下电与站立/趴下的按键语义：

- 新增 `named_poses.yaml`，保存 `idle`（趴地）与 `home`（站立）两组 12 关节角。
- 新增录制脚本，从 `/joint_states_feedback` 读当前姿态写入该文件。
- L3 改为**原地使能并保持当前姿态**，不再自动爬起。
- □ 从任意姿态平滑到 `home`；○ 从任意姿态平滑到 `idle`；R3 到 `idle` 后失能。
- 所有切换走**关节空间最小 jerk（五次多项式）S 曲线**插值，带最大关节速度限幅，并在进入/退出时与 CHAMP 做 0.3s 交叉淡入淡出，避免突变。

## 2. Current State Analysis

### 现有链路

```
CHAMP quadruped_controller ──/joint_commands(JointTrajectory,50Hz)──▶ motor_protocol_node ──MIT──▶ CAN
                                     ▲                                        ▲
                        power_sequence_node 发 /body_pose、/cmd_vel       /power_sequence/gate_open (Bool, transient_local)
```

- CHAMP 侧：`body_pose.position.z` 是**相对 `nominal_height=0.2` 的增量**，回调里直接 `cmdPoseCallback_` 赋值，无平滑（[quadruped_controller.cpp](file:///home/cat/trotbot_ws/src/champ/champ_base/src/quadruped_controller.cpp#L139-L159)）。
- `motor_protocol_node` 门禁关闭时丢弃轨迹；打开时逐关节走 `SmoothJointCommand`（[motor_protocol_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/motor_protocol_node.cpp#L1301-L1365)）：
  - `enable_startup_smoothing_`：从**节点启动时抓到的反馈**线性过渡 1.5s，只在节点启动后第一次命令有效；门禁重开**不会**重置锚点。
  - `command_max_velocity_rad_s=6.0`，但 `limit_velocity_only_during_smoothing_=true`，非平滑阶段**不限速**。
  - `enable_feedback_step_limit_`：每帧相对反馈最多 0.2 rad。
- `power_sequence_node`（[power_sequence_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp)）：
  - 状态机 `Idle/Precheck/EnableInit/SoftStand/Running/SoftProne/ProneHold/Disable/Fault`。
  - **L3** → Precheck → EnableInit（校验 12/12 反馈新鲜 → 写 `0x7026` → 使能 → MIT hold `p=feedback` → 上报 ON）→ **SoftStand**。
  - **SoftStand/SoftProne 只对 body 高度 z 做 SmoothStep**（2.5s），从固定配置 `prone_z=-0.2` 到 `stand_z=0.0`，**锚点是配置值，不是当前真实姿态**。
  - **□** 仅 `ProneHold` 可用 → SoftStand；**○** 仅 `Running` 可用 → SoftProne；**R3** → SoftProne(true) → Disable。
  - 话题命令：`start / prone / shutdown / set_zero / motor_init`。

### 结论（回答提问）

1. **没有** `named_pos` 之类的配置文件，仓库内无任何命名姿态实现（`hal/ref/dog/dog_static_poses.cc` 是参考 C++ HAL 的编译期静态点，未接入 ROS 链路）。
2. 现在按 L3 **不会跳到零位**，但会：使能时先 MIT hold 当前反馈 → 立刻把 `body_pose.z` 命令改为 `prone_z=-0.2`（对应机体高度 0.0m）→ 再从该点缓慢抬到 `stand_z=0.0`。也就是「先被拉到一个固定趴姿，再爬起」，而不是「从当前姿态原地保持」；若当前趴姿与 `prone_z` 不符，开门禁瞬间只能靠 0.2 rad/帧的步进限制兜底。
3. **没有 ruckig**（全仓无引用）。现有平滑只有：body z 的 SmoothStep + 上面那套启动期线性混合/步进限制，**不是关节空间轨迹规划**。

## 3. Proposed Changes

### 3.1 新增 `src/trotbot_can_bridge/config/named_poses.yaml`

普通 ROS 参数文件（同时供脚本读写的纯数据格式）：

```yaml
/**:
  ros__parameters:
    # 单位 rad；CHAMP 关节顺序：
    # base_lf1, lf1_lf2, lf2_lf3, base_rf1, rf1_rf2, rf2_rf3,
    # base_lb1, lb1_lb2, lb2_lb3, base_rb1, rb1_rb2, rb2_rb3
    # 由 record_named_pose.py 写入；手改也可
    idle_joints: [0,0,0, 0,0,0, 0,0,0, 0,0,0]
    home_joints: [0,0,0, 0,0,0, 0,0,0, 0,0,0]
```

> 初始值先用当前实测零位（12×0），录制后覆盖。

### 3.2 新增 `scripts/record_named_pose.py`

- 用法：`ros2 run trotbot_can_bridge record_named_pose.py idle` / `... home`
- 订阅 `/joint_states_feedback`，收 ~25 帧后：
  - 校验 `${pose}_joints` 12 维、关节名与 `DogJointOrder` 一致（按名取，不靠顺序）
  - 校验窗口内 span < 0.02 rad，否则报「机器人还在动」并退出
  - 取均值，**只改目标那一段，保留另一段**
- 文件路径优先级：`--file` > `$TROTBOT_NAMED_POSES_FILE` > `ros2 pkg prefix trotbot_can_bridge` + `/share/trotbot_can_bridge/config/named_poses.yaml`
- 同时把结果以表格打印（每关节 deg），便于人工确认
- 写入后提示：`ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: reload_poses}"`
- 加入 `CMakeLists.txt` 的 `install(PROGRAMS ...)`

### 3.3 `src/trotbot_can_bridge/src/motor_protocol_node.cpp`：关节姿态直通 + 交叉淡入

> 目的：让 power_sequence 能在**不经过 CHAMP IK** 的情况下直接驱动 12 关节，同时与 CHAMP 输出无缝交接。

新增参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_pose_servo` | `true` | 总开关，false 则完全走旧行为 |
| `pose_servo_active_topic` | `/pose_servo/active` | `std_msgs/Bool`，transient_local |
| `pose_servo_joint_topic` | `/pose_servo/joint_targets` | `sensor_msgs/JointState`，`position[12]`，CHAMP 顺序 |
| `pose_servo_blend_duration_s` | `0.30` | 进/出淡入淡出时长 |

实现要点：

- 成员：`pose_servo_active_`、`pose_servo_target_[12]`（初始 NaN）、`pose_servo_blend_`（0..1）、`pose_servo_last_stamp_ns_`。
- 50Hz 小定时器：`target = active ? 1.0 : 0.0`，`blend += clamp(target-blend, ±dt/blend_duration)`；仅在 `blend` 变化时更新。
- `pose_servo_joint_topic` 回调：长度 12 才接受，写 `pose_servo_target_` 与时间戳。
- **混合点**放在 `OnTrajectory` 每关节取到 `positions[i]` 之后、`SendMitFrame` 之前：

```cpp
double target = positions[i];                 // CHAMP 原始目标
if (pose_servo_blend_ > 1e-3 && std::isfinite(pose_servo_target_[i])) {
  target = (1.0 - pose_servo_blend_) * target + pose_servo_blend_ * pose_servo_target_[i];
}
```

- 超时保护：若 `active` 为 true 但 `pose_servo_target_` 超过 `0.5s` 没更新，视为无效，`blend` 自动回落到 0 并打 WARN。
- 不改动既有 `SmoothJointCommand` / 步进限制，二者继续作为最后一道保险。

### 3.4 `src/trotbot_can_bridge/src/power_sequence_node.cpp`：新状态机 + 命名姿态

新增参数（`config/power_sequence.yaml` 同步）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_named_poses` | `true` | false 则完全回退到现有 SoftStand/SoftProne 行为 |
| `named_poses_file` | `""` | 空则用 `<share>/config/named_poses.yaml` |
| `pose_move_duration_s` | `2.5` | 命名姿态切换基础时长 |
| `pose_max_joint_speed_rad_s` | `1.2` | 据此把时长放大到满足限速 |
| `pose_servo_active_topic` / `pose_servo_joint_topic` | 同 3.3 | 与 motor_protocol 对齐 |
| `enable_stand_after_home` | `true` | 到 home 后自动交给 CHAMP |

新增状态：`PoseHold`（En=1 刚上电保持当前姿态）、`PoseMove`（命名姿态切换中）。

**加载姿态**：用 `yaml-cpp` 读 `named_poses_file` 的 `idle_joints` / `home_joints`；维度非 12 或文件缺失 → WARN 并 `enable_named_poses_ = false`（回退旧行为，不硬崩）。

**新增发布**：`pose_servo_active_pub_`（Bool, transient_local）、`pose_servo_joint_pub_`（JointState，`name` 用 `DogMapper::kChampJointNames`，`position` 填 12 值）。

**L3（Idle）**：`Precheck → EnableInit → PoseHold`

- `EnableInit` 保持现有内容（12/12 反馈校验、`0x7026`、使能、MIT hold `p=feedback`、上报 ON），**不再** `PublishHoldPose(prone_z_)`，改为：
  - 记录 `hold_q_[12] = 当前反馈`（`CountFreshFeedback` 已通过，直接取 `last_feedback_champ_rad_`）
  - `pose_servo_target = hold_q_`，发 `active=true`
  - `SetGateOpen(true)`
  - `body_pose` 保持 `CurrentHeightCmdZ()` 不动（不写 `prone_z`）
- `PoseHold`：每 tick 重发 `hold_q_`，`PublishHoldPose(当前 z)` 只发零 `cmd_vel`；不进 `Running`，等 □ / ○ / R3。

**PoseMove(name, on_done)**

```cpp
struct PoseMove { std::array<double,12> q0, q1; double dur; double t; Target after; };
// 触发时：q0 = 当前反馈；q1 = poses_[name]
// dur = max(pose_move_duration_s, max_i|q1-q0| / pose_max_joint_speed_rad_s)
// 每 tick: a = clamp(t/dur,0,1); s = 10a³-15a⁴+6a⁵; q = q0 + s*(q1-q0)
```

- 触发时先 `active=true` 且 `pose_servo_target = q0`，等 0.3s 淡入完成再开始插值（用 `t` 从 −blend_duration 起算，负值阶段只锁 q0），保证进入也平顺。
- 完成（`a≥0.999`）后：
  - `home` → `active=false`（0.3s 淡出到 CHAMP）；同时 `PublishHoldPose(stand_z_)`、清零 `cmd_vel`、`EnterState(Running)`。`enable_stand_after_home=false` 时改为 `PoseHold`。
  - `idle` → 保持 `active=true` 锁住 `idle`，`EnterState(ProneHold)`。
- 过程中强制 `cmd_vel=0`，并忽略 `body_pose` 外部输入（当前 `block_body_pose_until_running` 已覆盖 Running 之外）。

**按键 / 话题语义**

| 输入 | 状态 | 行为 |
|------|------|------|
| L3 短按 | Idle | Precheck → EnableInit → **PoseHold**（原地使能保持） |
| □ 短按 | PoseHold / ProneHold / Running | **PoseMove(home)** → Running |
| ○ 短按 | PoseHold / Running | **PoseMove(idle)** → ProneHold |
| R3 短按 | PoseHold / Running | PoseMove(idle) → ProneHold → Disable |
| R3 短按 | ProneHold | Disable |
| R3 短按 | 启动中 | 取消启动 → Disable（保持现状） |
| L1+R1 长按 3s | Idle 且门禁关 | `motor_init`（不变） |
| 话题 `start` / `home` / `idle` / `prone` / `shutdown` / `reload_poses` / `motor_init` | — | `start`≡L3，`home`≡□，`idle`/`prone`≡○，`shutdown`≡R3，`reload_poses` 重读文件 |

- `reload_poses`：重新解析 `named_poses_file`，成功后 `RCLCPP_WARN` 打印两组关节角（deg）。
- 过温保护、`motor_init` 屏蔽、反馈新鲜校验等既有约束全部保留；新增：`PoseMove` 期间若触发过温，仍走 `BeginSoftProne(true)` 等价路径（直接 `Disable`，因为已由 pose servo 接管）。

### 3.5 `config/power_sequence.yaml` 更新

在文件末尾追加 3.4 的参数字段并写注释；`stand_z/prone_z/stand_duration_s/prone_duration_s` **保留**（`enable_named_poses=false` 时的回退路径仍依赖）。

### 3.6 `launch/can_bridge.launch.py`

- 新增 launch 参数 `named_poses_file`（默认 `PathJoinSubstitution([bridge_package,"config","named_poses.yaml"])`），传给 `power_sequence_node`。
- `motor_protocol_node` 参数里补 `pose_servo_*` 四个默认值（显式写出，便于现场调）。

### 3.7 依赖与构建

- `package.xml`：加 `<depend>yaml_cpp_vendor</depend>`
- `CMakeLists.txt`：`find_package(yaml_cpp_vendor REQUIRED)`、`find_package(yaml-cpp REQUIRED)`，`power_sequence_node` 链接 `yaml-cpp`；`install(PROGRAMS scripts/record_named_pose.py ...)`
- 已确认本工作区是 **symlink-install**（`install/.../config/*` 均为指向 `src` 的软链），脚本写 share 路径即等同写源码。

### 3.8 文档

`src/trotbot_can_bridge/README.md`：补一节「命名姿态与按键语义」，含录制流程、参数表、`:7029`/`0x18` 等无关内容不动。

## 4. Assumptions & Decisions

1. **姿态基准**：`idle`/`home` 均按**当前机械零位下的实测关节角**录制，单位为 CHAMP 关节域（即 `/joint_states_feedback` 的数值），与 `SendMitFrame` 的 `champ_position_raw` 同域，直接可用。
2. **home 与 CHAMP 站姿的差异**由 0.3s 交叉淡入吸收；若 `home` 就是在 CHAMP `stand_z=0` 站姿下录的，差异接近 0。
3. **不使用 ruckig**（用户已确认），用五次多项式 S 曲线 + 逐关节速度限幅；不引入新依赖（yaml-cpp 除外，已有 vendor 包）。
4. **录制用命令行脚本**（用户已确认），节点侧只提供 `reload_poses` 热重载，不做写文件。
5. **到 home 后自动交给 CHAMP**（用户已确认），摇杆即刻可走；`enable_stand_after_home=false` 可退回「锁住 home」。
6. 12 路反馈不新鲜时**拒绝**所有 pose 动作（沿用 `require_feedback_before_enable` 的判定），不做盲动。
7. 旧的 `SoftStand/SoftProne` 代码保留作为 `enable_named_poses=false` 的回退，避免一次改动彻底不可用。

## 5. Verification

按顺序执行，每步都要有可量化判据：

1. **构建**：`colcon build --symlink-install --packages-select trotbot_can_bridge`，无警告级错误。
2. **录制 idle**：狗子自然趴地、未上电 → `ros2 run trotbot_can_bridge record_named_pose.py idle` → 打印 12 角、span<0.02。
3. **录制 home**：把狗子摆到目标站姿（或先走旧流程站起）→ 录制 `home`。
4. **热重载**：`ros2 topic pub --once /power_sequence/command ... reload_poses`，日志打印两组角度与文件一致。
5. **L3 原地使能**：上电前记录 `/joint_states_feedback`；按 L3 后 3s 内观测：
   - `max_i |q_i(t) - q_i(0)| < 0.05 rad`
   - `/power_sequence/state` 停在 `PoseHold`，**不进入 Running**
   - 门禁 `gate_open=true`
6. **□ 到 home**：按 □，采样 50Hz 记录：
   - 每 20ms 单关节增量 `< pose_max_joint_speed_rad_s * 0.02 * 1.5`（允许 50% 余量）
   - 全程无反向抖动；到 `Running` 后 CHAMP 接管，摇杆推杆能正常走
7. **○ 到 idle**：Running 下按 ○ → `ProneHold`，关节应收敛到 `idle_joints`（误差 < 0.03 rad）。
8. **R3**：`PoseHold/Running` 下按 R3 → 先到 `idle` 再 `Disable` → `Idle`，全程无跳变。
9. **异常路径**：拔掉一路 CAN（模拟 52 掉线）→ L3 被拒绝并打印 `abort start: feedback 11/12 fresh`；恢复后正常。
10. **回退**：`enable_named_poses=false` 重启，确认 L3/□/○/R3 行为退回现有 SoftStand/SoftProne 版本。

## 6. 风险

- 若 `idle`/`home` 录制时机械零位已变（重新设零后），必须**重新录制**，否则姿态会整体偏一个偏移量。README 中需明确写这条。
- `PoseMove` 期间若用户推摇杆：`cmd_vel` 被强制清零且门禁在 pose 侧，CHAMP 输出被 blend 覆盖，不会打架，但摇杆操作会被丢弃直到进入 Running。
