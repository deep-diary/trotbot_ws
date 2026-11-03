---
description: 参考仓库优先：狗子电机/CAN/手柄开发先查 a3_arm_ws
alwaysApply: true
---

# 参考仓库：a3_arm_ws（狗子控制参考）

本仓（`/home/cat/trotbot_ws`，四足狗）与参考仓 **`/home/cat/a3_arm_ws`**（EDULITE A3 机械臂）使用**同款 EL05 MIT 电机与同一套 CAN 协议**。机械臂项目更成熟，是狗子控制实现的**首选参考**。

**参考仓只读：只查询、借鉴、移植，禁止修改 `/home/cat/a3_arm_ws` 内任何文件。**

## 可复用范围

- EL05 MIT 控制帧、`0xFD` 主站 / `0x18` 主动上报
- CMD `0x06` set_zero、参数 `0x7029` zero_sta、`0x16` 保存参数、`0x11` 读参数
- 电机使能/失能、限位与 MIT 参数映射
- PS4（evdev）按键映射、长按/组合键/边沿触发实现

## 检索入口

| 能力 | 路径 |
|------|------|
| 协议编解码 | `/home/cat/a3_arm_ws/src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp` |
| 电机协议节点 | `/home/cat/a3_arm_ws/src/a3_can_bridge/src/motor_protocol_node.cpp` |
| CAN 手动脚本（zero/zero_sta/check） | `/home/cat/a3_arm_ws/src/a3_can_bridge/scripts/el05_motor_cansend.sh` |
| PS4 驱动下标 | `/home/cat/a3_arm_ws/src/a3_teleop_ps4/config/ds4_linux.yaml`（`l3: 11`、`r3: 12`） |
| PS4 按键语义 | `/home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml` |
| 手柄动作实现 | `/home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py` |

机械臂关键映射（供对照）：

- `L1+R1` 长按 2s → `arm_init`（设零）
- `L3` 短按 → 一键上电 + 使能
- `R3` 短按 → 退出当前模式 → safe-park → 失能

## 使用约束

1. 开发电机 / CAN / 手柄相关功能前，**先查参考仓是否已有现成实现**；有则优先移植适配，并在代码或文档中注明来源（仓库 + 文件）。
2. **不可直接照搬**：机械臂 6/7 关节，狗 12 电机；关节映射（`DogMapper`）、限位、方向、状态机、电源时序均不同，必须按本仓 `trotbot_can_bridge` 适配并实测。
3. 涉及真实电机写参数 / 设零 / 保存 flash 的操作，必须满足：`Idle`、门禁关闭、零位工装姿态；写后需断电重启生效。
4. 跨仓改动只在本仓落地，不要假设参考仓可写。
