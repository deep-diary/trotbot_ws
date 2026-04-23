# 电机控制模块 (Motor Control Module)

## 功能概述

本模块提供 **小米无刷关节电机** 的 CAN 协议封装与 **`DeepMotor` 多电机管理**：

- **`MotorProtocol`**：静态方法组帧/发帧（位置、限速、电流、模式、使能等），解析反馈帧。
- **`DeepMotor`**：电机注册、反馈状态缓存、**目标量与「上次成功下发」缓存**（可去重跳过相同 CAN）、录制/播放、LED 状态（可选）、与 MCP 工具对接（见 `deep_motor_control.cc`）。
- **不直接**在业务层操作 `ESP32Can`，统一经 `MotorProtocol::sendCanFrame` → `ESP32Can.writeFrame`。
- 核对报文：`config.h` 中 **`DEEP_DOG_CAN_HEX_LOG`** 为 1 时，`MotorProtocol::sendCanFrame` 与板级 `CanRxTask` 以 **INFO** 打印 **CAN TX/RX** 的扩展 id、dlc、8 字节 HEX；调完改为 0。

---

## 文件说明

| 文件 | 说明 |
|------|------|
| `protocol_motor.h` / `protocol_motor.cpp` | 29 位扩展帧 ID、参数索引（`PARAM_LOC_REF`、`PARAM_LIMIT_SPD`、`PARAM_IQ_REF` 等）、`initializeMotor`、`setPosition`/`setPositionOnly`、`setSpeed`、`setCurrent`、`controlMotor`、反馈解析 `parseMotorData`；**`sendRunModeForStatusQuery`**：按 `DEEP_DOG_USE_MIT_WALK` 周期性写 RUN_MODE（MIT 或位置）以配合反馈 |
| `deep_motor.h` / `deep_motor.cpp` | 多电机注册表、`motor_status_t[]` 反馈、`MotorCommandCache` 下发去重、`processCanFrame`、位置/限速/IQ 下发封装 |
| `deep_motor_control.cc` / `deep_motor_control.h` | MCP 工具注册（`self.can.*`、`self.motor.*`），供语音/调试 |
| `deep_motor_led_state.*` | 角度 LED 指示（需灯带） |

---

## 数据与语义

### 反馈（实际量，来自 CAN 反馈帧）

`motor_status_t`（见 `protocol_motor.h`）包含：`current_angle`（rad）、`current_speed`（rad/s）、`current_torque`（N·m）、温度、错误位、模式等。  
`DeepMotor::processCanFrame` 解析后写入对应槽位。

便捷读取：

- `getMotorStatus(motor_id, &status)`
- `getMotorActualPosition` / `getMotorActualSpeed` / `getMotorActualTorque`

### 软件目标位置

- `motor_target_angles_[i]`：期望目标角（rad），用于 LED 等；`setMotorTargetAngle` / 各 `setMotor*` 会更新。

### 上次成功下发的指令（用于去重）

`MotorCommandCache`（每注册电机一份）记录最近一次 **成功发到总线** 的：

- 位置参考 `PARAM_LOC_REF`
- 限速 `PARAM_LIMIT_SPD`
- 电流指令 `PARAM_IQ_REF`（与 `setCurrent` 一致，力矩/电流环目标）

对应查询：`getMotorLastSentPosition` / `getMotorLastSentSpeedLimit` / `getMotorLastSentIqRef`（`*known == false` 表示尚未通过本类成功下发过该项）。

若新指令与缓存值在容差内相等，则 **跳过该路 CAN**（仍返回 `true`）。  
在 **`MotorProtocol` 侧直接 `resetMotor` 等** 后，须调用 `invalidateMotorCommandCache(motor_id)`，否则缓存可能与驱动器不一致（`LegControl::disable`、录制开始处已示例性调用）。

---

## DeepMotor 主要接口（摘要）

```cpp
// 注册（未收到反馈前也可先发控制）
deep_motor->registerMotor(motor_id);

// 位置 + 限速（内部分解为 setSpeed / setPositionOnly，各自可因去重跳过）
deep_motor->setMotorPosition(motor_id, position_rad, max_speed_rad_s);

// 仅限速 / 仅位置（整机站立等批量场景常用）
deep_motor->setMotorSpeedLimit(motor_id, max_speed_rad_s);
deep_motor->setMotorPositionRefOnly(motor_id, position_rad);

// 电流环目标（PARAM_IQ_REF）
deep_motor->setMotorIqRef(motor_id, iq_ref);

// 运控帧（角/角速/Kp/Kd，量程在 protocol 内裁剪）；发后会 invalidate 位置类缓存
deep_motor->setMotorMitCommand(motor_id, position_rad, velocity_rad_s, kp, kd, torque_ff /* N·m，编码入 CAN ID */);

// 外部协议直接改驱动状态后
deep_motor->invalidateMotorCommandCache(motor_id);
```

### 运控（MIT）通信类型 1（EL05）

- **29 位 ID**：bit24–28=命令类型 `0x1`；**bit8–23=前馈力矩**（uint16，±6 N·m）；bit0–7=电机 ID。`buildMitControlCanId`。
- **8 字节数据**：目标角、目标角速度、Kp、Kd；各 16 位 **大端**，`floatToUint16` 按 `P_MIN/P_MAX`（±12.57 rad）、`V_MIN/V_MAX`（±50 rad/s）、`KP_*`、`KD_*` 裁剪。

`initializeMotorMitMode`：`reset` → `setMotorZero` → `setMotorControlMode` → `PARAM_LIMIT_SPD` → `enableMotor` → 一帧运控保持（τ=0）。

整机验证：`config.h` 中 `DEEP_DOG_USE_MIT_WALK` + 可选 `DEEP_DOG_MIT_VALIDATE_FL_ONLY`；腿/整机初始化与下发均由 **`DEEP_DOG_USE_MIT_WALK`** 与 **`DEEP_DOG_MIT_DEFAULT_*`** 统一控制（见 `leg_control.cc` / `dog_control.cc`）。

### MCP 单电机 MIT（`deep_motor_control.cc`）

- `self.motor.initialize`：`motor_id`、`max_speed`（整数 ÷10 = rad/s，默认 10 → 1 rad/s，上限 50 rad/s）→ `MotorProtocol::initializeMotor`（按编译配置初始化 MIT/位置模式），并 `registerMotor`。
- `self.can.control_motor`：`position_x10`、`velocity_x10`、`kp_x10`、`kd_x10`、`tau_ff_x10` 均为 **整数÷10** 得 rad、rad/s、kp、kd、τ → `DeepMotor::setMotorMitCommand`（会先注册）。默认除扭矩外 **物理量 1.0**（对应 `_x10` 均为 10，`tau_ff_x10` 默认 0）。

底层仍可直接调 `MotorProtocol::initializeMotor`、`resetMotor` 等（如 `LegControl::init`）；建议与 `invalidateMotorCommandCache` 策略保持一致。

---

## 常量与容量

- `MAX_MOTOR_COUNT`：当前为 **13** 槽位（机器狗 12 电机 + 1 余量）；整机 ID 见 `../dog/README.md`（11–13, 21–23, 51–53, 61–63）。
- `MOTOR_CAN_TIMEOUT_MS`：见 `protocol_motor.h`，`sendCanFrame` 调用 `ESP32Can.writeFrame(..., timeout)`。
- 批量发送时，`sendCanFrame` 的详细 hex 使用 **`ESP_LOGD`**，避免默认日志级别下 UART 拖慢。

---

## MCP 工具（`deep_motor_control.cc`）

板级 `esp_sparkbot_board.cc` 的 `InitializeTools()` 中，**电机级 MCP 默认注释掉**（避免 MCP 工具数量过多），需要单电机调试时可取消 `RegisterMotorMcpTools(mcp_server, deep_motor_)` 的注释。

当前文件中注册的工具包括但不限于（以代码为准）：

| 前缀 | 示例工具名 |
|------|------------|
| `self.can.*` | `send_motor_position`、`enable_motor`、`reset_motor` |
| `self.motor.*` | `get_status`、`set_position_mode`、`set_zero_position`、`initialize`、`start_status_task`、`stop_status_task`、录制/播放、正弦、角度 LED、版本号等 |

---

## 依赖与在 deep-dog 中的角色

- **CAN**：`../can/ESP32-TWAI-CAN`，全局 `ESP32Can`。
- **上层**：`../leg`、`../dog` 通过 `DeepMotor` 与 `MotorProtocol` 控制 12 个电机；**不**直接调用 `ESP32Can`。

---

## 相关文档

- `../dog/README.md` — 电机 ID 与腿/关节对应、整机 MCP。
- `../leg/README.md` — 单腿控制与 `LegControl`。
- `../can/README.md` — TWAI 初始化与硬件说明。
