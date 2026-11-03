# 按键重映射与 a3_arm_ws 参考规则

## 一、目标摘要

1. 把 `/home/cat/a3_arm_ws` 写入本仓规则，作为狗子电机/CAN/手柄控制的**参考仓库**（同款 EL05 MIT 电机与协议）。
2. 重新映射 `power_sequence_node` 手柄按键：
   - 「机械零位 set_zero + 断电位置域 zero_sta=-180°~180°」合并为一个事件，绑到 **L1+R1 长按**（与机械臂 `arm_init` 一致）；上电仅保留 **□ 长按**。
   - **R3 短按** = 回 idle（SoftProne 趴下）再失能（Disable），与机械臂 R3 语义一致；保留原 `L1+R1+Share` 下电组合。
   - 其它按键保持原样。
3. 按仓库既有文档规则同步更新相关文档。

## 二、现状分析

### 2.1 规则文件

- 本仓目前只有 Cursor 规则 [docs-dynamic-update.mdc](file:///home/cat/trotbot_ws/.cursor/rules/docs-dynamic-update.mdc)，无 `.trae/rules/` 目录。
- 参考仓 `a3_arm_ws` 同时维护 `.trae/rules/` 与 `.cursor/rules/`，其 `reference-first.md` 已把 trotbot 列为生产执行层来源。

### 2.2 手柄现状（`power_sequence_node`）

配置 [power_sequence.yaml](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/config/power_sequence.yaml#L15-L67)：

| 物理键 | joy 下标 | 当前行为 |
|---|---|---|
| L1+R1（未按 Share） | 4+5 | 长按 ≈ `start` 上电 |
| □ | 3 | 长按 ≈ `start` 上电 |
| L1+R1+Share | 4+5+8 | 长按 ≈ `shutdown`（SoftProne→Disable） |
| ○ | 1 | 长按 ≈ `prone`（趴下到 ProneHold，保持使能） |
| Options | 9 | 长按 = 单独 `set_zero`（不写 zero_sta） |
| Share+Options | 8+9 | 长按 3s = `motor_init`（set_zero + zero_sta=1 + 保存 + 回读） |
| L3 / R3 | 11 / 12 | 未绑（[PS4 按键说明](file:///home/cat/trotbot_ws/docs/trotbot/PS4%20按键说明.md#L27-L28)） |

代码位置：

- 参数声明 [power_sequence_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp#L87-L127)
- 按键处理 `OnTimer()` [power_sequence_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp#L341-L461)
  - Options 单独 set_zero：L348-L359
  - Share+Options motor_init：L361-L370
  - L1+R1 start / L1+R1+Share shutdown / ○ prone：L377-L425
- `motor_init` 已实现「set_zero → 写 zero_sta(0x7029)=1 → 保存 → 回读」，见 [BeginMotorInit/TickMotorInit](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp#L850-L1004)。

### 2.3 参考仓机械臂映射（已确认）

[ds4_linux.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/ds4_linux.yaml#L19-L33)：`l3: 11`、`r3: 12`；[default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L118-L132)：

- `l1_r1` 长按 2s → `arm_init`（设零）
- `l3` 短按 → `arm_power_enable`
- `r3` 短按 → `arm_disable`（先退出当前模式 → safe-park → 失能）

断电位置域：`kParamZeroSta(0x7029)` 中 `0=0~2π`、`1=-π~π`，见 [protocol_codec.hpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/include/trotbot_can_bridge/protocol_codec.hpp#L26-L27)。本仓 `motor_init_zero_sta` 默认已是 `1`，即 **-180°~180°**，数值无需修改，只需把该事件绑到新按键。

## 三、变更方案

### 3.1 新增规则文件

新建 `/home/cat/trotbot_ws/.trae/rules/reference-repos.md`（Trae 规则，`alwaysApply: true`），内容包含：

- **定位**：`/home/cat/a3_arm_ws` 是狗子控制的成熟参考仓，电机与 CAN 协议同源，**只读，不得修改**。
- **可复用范围**：EL05 MIT 帧、`0xFD` 主站/`0x18` 主动上报、CMD `0x06` set_zero、`0x7029` zero_sta、`0x16` 保存、`0x11` 读参数、PS4 evdev 映射与长按/组合键实现。
- **检索入口**（给出真实路径）：
  - `src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp`
  - `src/a3_can_bridge/src/motor_protocol_node.cpp`
  - `src/a3_can_bridge/scripts/el05_motor_cansend.sh`
  - `src/a3_teleop_ps4/config/ds4_linux.yaml`、`config/mappings/default.yaml`、`a3_teleop_ps4/actions.py`
- **使用约束**：
  1. 开发电机/CAN/手柄功能前先查参考仓，有现成实现优先移植适配，并注明来源。
  2. 不可直接照搬：机械臂 6/7 关节、狗 12 电机；关节映射（`DogMapper`）、限位、方向、状态机均不同。
  3. 参考仓路径可能只读；跨仓改动只在本仓落地并说明来源。
  4. 涉及真实电机写参数/设零的操作，必须在 Idle、门禁关闭、零位工装姿态下执行。

### 3.2 `power_sequence.yaml`

文件：[power_sequence.yaml](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/config/power_sequence.yaml)

- 顶部按键注释改为：
  - `L1+R1 长按（未按 Share）` = `motor_init`（设零 + zero_sta=1）
  - `□ 长按` = `start`（上电）
  - `L1+R1+Share 长按` = `shutdown`
  - `○ 长按` = `prone`
  - `R3 短按` = `shutdown`（回 idle 再失能）
- 删除 `button_option` 的「单独 set_zero」说明；`set_zero_longpress_s` 参数删除（代码同步删除）。
- 新增：
  ```yaml
  # PS4 R3（右摇杆按下，evdev 下标 12）：短按 = 回 idle（SoftProne）后失能；仅 Running/ProneHold 生效
  button_r3: 12
  ```
- `motor_init_longpress_s` 保持 **3.0s**（沿用现防误触时长；机械臂为 2s，但狗子该按键会改写机械零位，保守保留 3s）。
- `motor_init_do_set_zero: true`、`motor_init_zero_sta: 1` 保持不变，确保「设零 + -180~180 记忆范围」是同一事件。

### 3.3 `power_sequence_node.cpp`

文件：[power_sequence_node.cpp](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/src/power_sequence_node.cpp)

1. **参数**
   - 新增 `button_r3_ = declare_parameter<int>("button_r3", 12);`
   - 删除 `set_zero_longpress_s_` 的声明与成员（`enable_remote_set_zero_` 保留，供话题 `set_zero` 使用）。
   - 新增成员 `bool r3_prev_{false};`。

2. **`OnTimer()` 重排**
   - 在函数前部先取 `share`、`option_pressed`、`l1`、`r1`、`square`、`r3`。
   - 计算 R3 上升沿并**最先更新** `r3_prev_ = r3;`（避免早退导致边沿丢失）：
     ```cpp
     const bool r3 = (button_r3_ >= 0) && ButtonOn(button_r3_);
     const bool r3_edge = r3 && !r3_prev_;
     r3_prev_ = r3;
     ```
   - **删除** Options 单独 set_zero 块（原 L348-L359）及 `option_hold_s_` / `option_action_done_this_hold_` 成员。
   - `motor_init` 组合键改为：
     ```cpp
     const bool init_combo =
       ((l1 && r1 && !share) || (option_pressed && share)) &&
       state_ == SequenceState::Idle && !gate_open_;
     ```
     `Share+Options` 保留为兼容别名；`motor_init_active_` 早退与屏蔽逻辑保持不变。
   - `start_combo = square;`（取消 L1+R1 上电；□ 行为不变）。
   - `shutdown_combo` 保持 `l1 && r1 && share`。

3. **R3 行为**
   - `Running` 分支：
     ```cpp
     if (r3_edge) {
       BeginSoftProne(true);   // 先趴下到 prone_z_，完成后进入 Disable
     } else if (...shutdown...) { ... } else if (...prone...) { ... }
     ```
   - `ProneHold` 分支：`r3_edge` → `EnterState(SequenceState::Disable);`
   - 其它状态（Idle / 启动过程 / motor_init 期间）忽略 R3，避免误失能。
   - 日志：`RCLCPP_WARN(..., "joy R3 short press: safe-park (SoftProne) then disable")`。

4. **日志/语义**
   - motor_init 触发日志中注明 `L1+R1`。
   - 保留话题 `set_zero` / `motor_init`，不影响无手柄调试。

### 3.4 文档同步（遵循仓库文档规则）

- [src/trotbot_can_bridge/README.md](file:///home/cat/trotbot_ws/src/trotbot_can_bridge/README.md#L44-L47)：按键表改为 L1+R1=motor_init、R3=下电；删除 Options 单独 set_zero 描述。
- [docs/trotbot/PS4 按键说明.md](file:///home/cat/trotbot_ws/docs/trotbot/PS4%20按键说明.md)：§1.1 的 11/12 行改为 L3 未绑 / R3 下电；§4 表格更新组合键。
- [docs/trotbot/架构说明.md](file:///home/cat/trotbot_ws/docs/trotbot/架构说明.md#L77-L79)：状态机按键描述更新。
- [docs/trotbot/软件需求清单.md](file:///home/cat/trotbot_ws/docs/trotbot/软件需求清单.md#L161)：按键约定与 RQ-024 触发键改为 L1+R1。
- [docs/trotbot/README.md](file:///home/cat/trotbot_ws/docs/trotbot/README.md#L17)：上电/下电按键摘要。
- 根 [README.md](file:///home/cat/trotbot_ws/README.md#L199-L203)：2a-1 手柄等效说明。
- [docs/trotbot/开发问题跟踪.md](file:///home/cat/trotbot_ws/docs/trotbot/开发问题跟踪.md)：追加一条本次按键重映射记录（现象/变更/验证/结论）。

## 四、决策与假设

| 项 | 决策 |
|---|---|
| 设零组合键 | **L1+R1 长按 3s**，触发既有 `motor_init`（set_zero + zero_sta=1 + 保存 + 回读） |
| L1+R1 上电 | **取消**；上电只保留 □ 长按（已确认） |
| Share+Options | 保留为 `motor_init` 兼容别名，不破坏旧习惯 |
| Options 单独 set_zero | **删除按键入口**；话题 `/power_sequence/command: set_zero` 保留 |
| R3 | 短按（按下沿），仅 `Running`→SoftProne→Disable、`ProneHold`→Disable |
| zero_sta | 默认已是 `1` = -π~π = -180°~180°，不改数值 |
| 安全 | L1+R1 从「上电」改为「改写机械零位」属高风险变更，文档与 YAML 注释需明确警告：必须在零位工装姿态、Idle、门禁关时操作 |

## 五、验证

1. 编译：
   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/cat/trotbot_ws
   colcon build --packages-select trotbot_can_bridge --symlink-install
   ```
2. 静态核对：`grep -n "button_r3\|set_zero_longpress_s\|start_combo\|init_combo" src/trotbot_can_bridge/src/power_sequence_node.cpp`，确认无残留引用。
3. 参数核对：
   ```bash
   ros2 param get /power_sequence_node button_r3
   ros2 param get /power_sequence_node motor_init_zero_sta   # 应为 1
   ```
4. 真机（空载/零位工装）：
   - `ros2 topic echo /joy` 按 L3/R3，确认下标 11/12（若不符改 YAML）。
   - Idle + 门禁关：L1+R1 长按 3s → 日志出现 `motor_init: set_zero round...` → `wrote 0x7029=1` → `save params` → 回读 12/12。
   - □ 长按仍可正常上电到 Running。
   - Running 时短按 R3 → SoftProne 2.5s → Disable → Idle；L1+R1+Share 仍等效。
   - ProneHold 时短按 R3 → 直接 Disable。
   - Idle 时短按 R3 → 无动作。
5. 回归：○ 长按 prone、L1 横移、R1 yaw、△/✕ 高度、Options 不再单独设零。
