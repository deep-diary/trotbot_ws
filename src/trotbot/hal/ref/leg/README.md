# 单腿控制模块 (Leg)

## 功能概述

**机器狗单腿**抽象：一条腿 3 个关节（髋侧摆、髋前后、膝），对应 3 个无刷电机，由 `../motor` 中的 `DeepMotor` / `MotorProtocol` 驱动。本模块负责**步态相位**、**站立/卧倒**、**迈一步**的目标位置计算，并下发到电机；**不**直接操作 `ESP32Can`。

---

## 目录与文件

| 文件 | 说明 |
|------|------|
| `leg_control.h` | 腿类型 `LegType`、`LegControl`、关节索引与常量 |
| `leg_control.cc` | 正弦步态、`RegisterLegMcpTools()` |

---

## 机械与关节（简要）

| 关节 | 索引 | 说明 |
|------|------|------|
| 髋侧摆 (Hip AA) | `LEG_JOINT_HIP_AA` (0) | 站立时维持足端间距，可小幅摆动 |
| 髋前后 (Hip FE) | `LEG_JOINT_HIP_FE` (1) | 前进/后退主动力 |
| 膝 (Knee) | `LEG_JOINT_KNEE` (2) | 摆动相抬脚，支撑相承重 |

默认站立位、行走范围与机械限位见 `../dog/README.md` 表格；本模块用 `clampJoint` 将目标夹在控制上下限内。

---

## LegControl 类

### 配置

- **腿类型**：`LegType`（fl / fr / rl / rr），决定默认 **3 个电机 ID** 与默认站立/限位（见 `LEG_DEFAULTS`）。
- **电机 ID**：`motor_ids_[3]`，也可用 `setMotorIds(hip_aa, hip_fe, knee)` 覆盖。
- **站立位 / 限位**：`setStancePosition`、`setLimits`。
- **步态**：`total_steps_`、`current_step_`，`computeStepPosition` 正弦偏移。

### 方法

| 方法 | 说明 |
|------|------|
| `setLegType` / `setMotorIds` / `setLimits` / `setStancePosition` | 配置 |
| `init()` | 注册本腿 3 电机并 `MotorProtocol::initializeMotor`（每电机使能、位置模式等） |
| `disable()` | 对 3 电机 `MotorProtocol::resetMotor`，并对已绑定的 `DeepMotor` 调用 `invalidateMotorCommandCache` |
| `goToZero` | 先对 3 关节 `setMotorSpeedLimit`，再 `setMotorPositionRefOnly(0)` |
| `goToStance` | 同上，位置为站立位（限幅后） |
| `stepForward` / `stepBackward` | 步数 ±1，计算 3 关节目标，先限速再仅位置 |
| `getMotorId(j)` | 关节 `j` 对应电机 ID |
| `getStanceTargetJoint(j)` | 站立目标（已 clamp） |
| `fillCurrentStepPositions(out, forward)` | 按当前 `current_step_` 填 3 关节目标（不改步数） |
| `advanceStepForward` / `advanceStepBackward` | 仅修改步态相位（供整机同步迈步） |
| `clampJoint` | 限幅 |

---

## 与整机 `DogControl` 的配合

- **`DogControl::stand` / `lieDown` / `goForward` / `goBack`** 不再按「一条腿跑完再下一条」串行调用单腿的 `goToStance`/`goToZero`/`stepForward`，而是：
  - 先对 **12 个电机**统一 `setMotorSpeedLimit`（或等价流程），
  - 再按 **关节维度**：同一关节的 **4 条腿**连续 `setMotorPositionRefOnly`，实现视觉上更同步。
- **前进/后退**：整机使用 **`../dog/gait_planner`**（`GaitPlanner`）维护全局周期索引，每腿调用 **`fillStepPositionsAtStepIndex(step_index, …)`**，`step_index` 由步态类型（默认 Trot 对角）决定；与单腿 `current_step_` 独立推进的旧逻辑不同。
- 单腿 MCP / 调试仍使用 `LegControl::goToStance`、`stepForward` 等，行为为 **本腿 3 关节**先全部限速再全部位置（仍用 `current_step_`）。

---

## Leg MCP 工具

`RegisterLegMcpTools(McpServer&, LegControl* legs[4])`，板级传入 `dog_` 内四条腿指针。

| 工具名 | 说明 |
|--------|------|
| `self.leg.init` | 单腿 3 电机初始化 |
| `self.leg.stand` | 单腿回站立位 |
| `self.leg.lie_down` | 单腿回零（卧倒） |
| `self.leg.step_forward` / `self.leg.step_back` | 单腿迈一步 |

参数 `leg_id`：支持 `fl`/`fr`/`rl`/`rr`、中文「前左」「左前」、以及 `1`–`4` 号腿等（见 `leg_control.cc` 中 `str_to_leg_index`）。

**电机模式（统一）**：由 `config.h` 中 **`DEEP_DOG_USE_MIT_WALK`** 一处决定整条链路：`LegControl::init` / `sendThreeJointsPositionOrMit` 与 `DogControl::sendAllLegJointTargets` 均用 `#if` 选择 **MIT** 或 **位置模式**；运控增益为 **`DEEP_DOG_MIT_DEFAULT_KP/KD/TAU_FF`**（默认 1/1/0，与单电机 MCP 对齐）；MIT 下运控 **`v_des`** 与位置模式默认限速均见 **`DEEP_DOG_MIT_VDES_RAD_S`**。无需再在各层 `setMitMode`。

**建议单腿台架测试顺序**：机身可靠支撑 → 选一条腿 → `self.leg.init`（观察该腿 3 轴 CAN 与日志）→ `self.leg.stand` → 小幅度 `step_forward` / `step_back` → `self.leg.lie_down` 或 `self.motor.reset_motor`（单电机 MCP）按需失能。**期望**：无报错字符串；关节按站立/步态/卧倒方向平滑运动，且被机械限位夹紧时日志有 `机械限位裁剪`。

---

## 与上下层关系

- **上层**：`../dog/dog_control` 持有 4×`LegControl`，编排站立/步态。
- **下层**：通过 `DeepMotor::setMotorSpeedLimit`、`setMotorPositionRefOnly` 等下发；`init`/`disable` 仍直接使用 `MotorProtocol` 做初始化与 reset。
- **轨迹**：若需平滑可在 `../trajectory` 扩展，由上层按点设置目标。

---

## 开发状态（摘要）

- [x] `LegControl` 与单腿 MCP  
- [x] 与 `DeepMotor` 限速 + 运控/位置参考下发（见 `DEEP_DOG_USE_MIT_WALK`）  
- [x] 整机层关节同步站立/迈步（见 `dog_control.cc`）  

更多电机编号与限位以 `../dog/README.md` 为准。
