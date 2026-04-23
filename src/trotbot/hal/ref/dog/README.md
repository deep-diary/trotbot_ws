# 整机控制模块 (Dog)

## 功能概述

**机器狗整机**控制：4 条腿、12 个电机，一路 CAN。负责前进、后退、站立、卧倒、跳舞等**步态编排**，以及对外提供统一接口供板级 MCP 工具（语音控制）调用。不依赖 URDF，直接做关节角规划。

---

## 目录与文件

- `dog_control.h` / `dog_control.cc`：整机接口，持有 4×`LegControl`、**`GaitPlanner`** 与 **`DogStateMachine`**
- `dog_state_machine.h`：整机姿态/任务状态枚举与轻量状态机（无 CAN）
- `gait_planner.h` / `gait_planner.cc`：**步态规划**（全局周期索引 + 各腿相位偏移），无 URDF

---

## 姿态状态机（`DogPoseState` / `DogStateMachine`）

| 状态 | 含义 |
|------|------|
| `Uninitialized` | 未执行整机 `init`，或已 `disable` |
| `UnknownPose` | 行走下发失败等导致**姿态不确定**（与 init 默认态无关） |
| `Lying` | 已卧倒（趴下），**待机**。整机若在趴姿下完成 init（写零位），**`init` 成功后即为此状态** |
| `Standing` | 已站立，**可安全行走** |
| `Moving` | 正在执行前进/后退（**单步**或 `goForwardSteps` / `goBackSteps` **整段**） |
| `WalkingForward` | **持续前进**（触摸长按 2 等触发，由后台任务循环迈步，直至 `stopContinuousLocomotion`） |
| `WalkingBackward` | **持续后退**（触摸长按 3 等触发，同上） |

**行为约定**

- **趴下后误发前进/后退**：不会直接按卧姿去“迈一步”。`goForward` / `goBack` / `goForwardSteps` / `goBackSteps` 会先调用 **`ensureStandingForWalk()`**：若当前为 `Lying` 或 `UnknownPose`，先 **`stand()`**，再迈步。
- **持续前进/后退**与单步 `goForward` / `goBack` 互斥；停止持续行走时**不再下发新目标**，关节保持**最后一次下发的姿态**（`stopContinuousLocomotion`）。
- **板级扩展**：可通过 `DogControl::getPoseState()` 读取状态，驱动灯带/屏显（例如 `Standing` 常亮、`Moving` 流水、`WalkingForward`/`WalkingBackward` 不同闪烁、`Lying` 呼吸灯等）。

---

## 限位与初始化说明

- **行走上下限**（上表「行走下/上极限」）：在 `LegControl` 内用于步态规划时的 `clampJoint`，正常步态幅度应落在此范围内。
- **机械上下限**（上表「机械下/上极限」）：在 **`setMotorPositionRefOnly` 之前** 由 `LegControl::clampJointPositionsMechanical` 再夹紧一道，防止异常目标角顶机械。
- **电机初始化**（`LegControl::init` → `MotorProtocol::initializeMotor`）：协议侧当前下发的是 **位置模式、速度限制 `PARAM_LIMIT_SPD` 等**，**未向电机关节驱动写入位置软限位参数**（`protocol_motor.h` 中无对应 `PARAM`）。因此 **关节安全以本仓库软件限位为主**；若后续固件支持位置限位寄存器，再在 `initializeMotor` 中补充。

---

## 步态规划（`GaitPlanner`）

- **全局变量**：`cycle_index ∈ [0, total_steps-1]`，每次 `goForward` / `goBack` 只 **±1**（一个离散节拍），不再四腿各 `advanceStep` 各迈一格。
- **每条腿的有效步数**：`effectiveStepForLeg(leg) = (cycle_index + offset_leg) % total_steps`，用于 `LegControl::fillStepPositionsAtStepIndex` 中的 `sin(2π·step/total_steps)`。
- **默认 `QuadrupedGaitType::Trot`（对角小跑）**：FL(0) 与 RR(3) 同相；FR(1) 与 RL(2) 相对 **半周期**（`total_steps/2`）。
- **`QuadrupedGaitType::SyncAllLegs`**：四腿 `offset=0`，与旧版「四腿同相」一致；可通过 `DogControl::setQuadrupedGaitType(...)` 切换。
- **`total_steps`**：与 `LegControl::total_steps_` 一致，构造时设为 `LEG_DEFAULT_TOTAL_STEPS`（偶数，便于半周期）；`GaitPlanner::setTotalSteps` 若为奇数会 **自动 +1**。
- **站立 / 卧倒 / init 成功**：`resetCycle()`，并把各腿 `current_step_` 设为当前 `effectiveStepForLeg`，便于单腿 MCP 与调试一致。

---

## 电机 ID 与腿/关节对应

4 条腿 × 3 关节 = 12 个电机，建议分配如下（与 `config.h` / 硬件一致）：

| 腿 | 关节 | 电机ID | 零位 | 站立位置(rad) | 行走下极限 | 行走上极限 | 机械下极限 | 机械上极限 |
|----|------|--------|------|---------------|------------|------------|------------|------------|
| 前左 fl | 髋侧摆 | 11 | 0 | 0 | -0.01 | 0.01 | -0.3(外) | 0.3(内) |
| 前左 fl | 髋前后 | 12 | 0 | 0.2 | 0.04 | 0.36 | -0.6(后极限) | 1.4(前极限) |
| 前左 fl | 膝 | 13 | 0 | -1.4 | -1.58 | -1.22 | -2.26(伸直) | 0(弯曲) |
| 前右 fr | 髋侧摆 | 21 | 0 | 0 | -0.01 | 0.01 | -0.3(内) | 0.3(外) |
| 前右 fr | 髋前后 | 22 | 0 | -0.2 | -0.36 | -0.04 | -1.4(前极限) | 0.6(后极限) |
| 前右 fr | 膝 | 23 | 0 | 1.4 | 1.22 | 1.58 | 0(弯曲) | 2.26(伸直) |
| 后左 rl | 髋侧摆 | 51 | 0 | 0 | -0.01 | 0.01 | -0.3(内) | 0.3(外) |
| 后左 rl | 髋前后 | 52 | 0 | 0.2 | 0.04 | 0.36 | -0.6(后极限) | 1.4(前极限) |
| 后左 rl | 膝 | 53 | 0 | -1.4 | -1.58 | -1.22 | -2.26(伸直) | 0(弯曲) |
| 后右 rr | 髋侧摆 | 61 | 0 | 0 | -0.01 | 0.01 | -0.3(外) | 0.3(内) |
| 后右 rr | 髋前后 | 62 | 0 | -0.2 | -0.36 | -0.04 | -1.4(前极限) | 0.6(后极限) |
| 后右 rr | 膝 | 63 | 0 | 1.4 | 1.22 | 1.58 | 0(弯曲) | 2.26(伸直) |

> 当前实现中，行走上下限由“站立位 + 默认迈步幅度”自动推导：  
> - HipAA：`DEEP_DOG_STANCE_HIP_AA_RAD ± LEG_DEFAULT_HIP_AA_AMP`  
> - HipFE：`±DEEP_DOG_STANCE_HIP_FE_RAD ± LEG_DEFAULT_HIP_FE_AMP`（左右腿符号相反），另加经 `hip_fe_sign` 后的 **`LEG_HIP_FE_COS_AMP * cos(phase)`**（默认负小幅，抵消后漂时可再微调符号/模长）  
> - Knee：`±DEEP_DOG_STANCE_KNEE_RAD ± LEG_DEFAULT_KNEE_AMP`（左右腿符号相反），摆动半周叠加 **`LEG_KNEE_SWING_LIFT_RAD * sin²`**（**左膝加、右膝减**，与左右膝角极性一致）

- **行走上下极限**：正常行走时关节活动范围（与 `leg_control.cc` 中 `LEG_DEFAULTS` 的 `limit_low`/`limit_high` 一致）。步态正弦项叠加后再经 `clampJoint`。
- **机械上下极限**：实测机械限位（`mech_limit_low`/`mech_limit_high`）；**下发前**再经 `clampJointPositionsMechanical`。

---

## 控制指令（对外接口 / MCP 对应）

| 指令 | 说明 |
|------|------|
| 初始化 | CAN + 12 电机使能、各腿初始化 |
| 站立 | 4 条腿回到站立位 |
| 卧倒 | 4 条腿回零位 |
| 前进 | 推进全局周期并按当前步态（默认 Trot）下发各腿目标 |
| 后退 | 周期反向，关节正弦项取反（`forward=false`） |
| 向左 / 向右 | 步态上差速或身体偏转（可后续实现） |
| 跳舞 | 预定义动作序列（关键帧 + 可选 trajectory 插值） |

---

## 与板级入口的对接

在 `esp_sparkbot_board.cc` 中，MCP 工具应调用本模块，而不是 UART（`RegisterDogMcpTools`）：

- `self.dog.init` / `self.dog.stand` / `self.dog.lie_down`
- `self.chassis.forward_big` / `backward_big`（一大步，参数 `speed`×100=rad/s、`step_delay_ms`）
- `self.chassis.start_forward` / `start_backward`（持续前进/后退，参数 `speed`、`step_period_ms`）
- `self.chassis.stop`（停持续行走）、`self.chassis.status`（中文状态）、`self.chassis.set_speed`（调速+步频）
- `self.chassis.dance` → `DogControl::dance()`

内部仍保留 `goForward`/`goBack`/`goForwardSteps` 等 API，供内部序列与调试使用；**跳舞**为站立 → **2× 前进一大步** → **2× 后退一大步** → 站立；触摸无组合窗时短按 2/3 为一小步。整机 `init()` 成功后打印 `[init 后反馈]` 核对各电机是否近零位。语音侧优先用上述 MCP 名称。

---

## 开发计划

- [x] 定义 `DogControl` 类，持有 4 个 `LegControl` 及 motor/CAN 依赖。
- [x] 实现初始化、站立、卧倒（调用各腿接口）。
- [x] 实现前进/后退：`GaitPlanner` 对角 Trot + 可切换 SyncAllLegs；`LegControl` 支持按显式 `step_index` 计算正弦目标。
- [x] 在 `esp_sparkbot_board.cc` 中注册 MCP 并改为调用 `DogControl`（`RegisterDogMcpTools`）；保留单腿 `RegisterLegMcpTools` 用于调试。
- [x] 实现跳舞：站立 → 2×`goForwardBigStep` → 2×`goBackBigStep` → 站立；后续可改为关键帧+插值。
- [ ] 可选：左转/右转。
- [x] **一大步** `goForwardBigStep`/`goBackBigStep`（半周期小步+延时）；`start_forward`/`backward` 可调步频；MCP `speed` 在位置模式为限速参考，**MIT 下运控 `v_des` 见 `DEEP_DOG_MIT_VDES_RAD_S`**（**范围** `DEEP_DOG_CHASSIS_SPEED_X100_*`）。
- [x] 姿态状态机（趴下后前进先站立、软件机械限位、`getPoseState()` 供 LED 扩展）。
