# TrotBot 文档导航（`docs/trotbot`）

该目录按「长期沉淀、跨模块协作」放在工作空间 `docs/trotbot/`，与包内执行型文档分工如下。

## 核心文档

| 文件 | 内容 |
|------|------|
| **[开发路线.md](./开发路线.md)** | 对标行业的 L0-L5 能力分层、本仓完成度看板、分阶段路线与本周优先事项 |
| **[软件需求清单.md](./软件需求清单.md)** | RQ-001～RQ-018 等需求与实现状态（CHAMP→CAN→反馈→RViz） |
| **[架构说明.md](./架构说明.md)** | As-Is 节点/话题/数据流、与 To-Be 演进建议 |
| **[开发问题跟踪.md](./开发问题跟踪.md)** | ISSUE 模板与已登记问题（含 RViz 反馈、`enable_rx_decode_log`、启停编排等） |
| **[本地运行指南.md](./本地运行指南.md)** | 运行入口摘要；**完整步骤见 `src/trotbot/本地运行指南.md`** |
| **[音频使用说明.md](./音频使用说明.md)** | 板卡 ALSA/PulseAudio：蓝牙与板载播放录音、`pactl` 默认设备、命令行 `aplay`/`paplay`/`arecord` |

## 运行与调试专题（自根 README 拆出）

| 文件 | 内容 |
|------|------|
| **[CAN配置与报文调试.md](./CAN配置与报文调试.md)** | 设备树 overlay 修复、`can-up.service`、1Mbps 配置、`cansend` 主动上报 / 设零、`candump` |
| **[遥控与电源启停.md](./遥控与电源启停.md)** | `power_sequence_node` 指令与状态语义、PS4 / 键盘遥控、轴映射 |
| **[CAN桥架构与话题细节.md](./CAN桥架构与话题细节.md)** | 双节点桥接架构、节点收发关系、话题消息示例、发送统计日志 |
| **[CAN频率与负载.md](./CAN频率与负载.md)** | 轨迹频率 → CAN 帧率 / 1Mbps 带宽估算、推荐值与调参入口 |
| **[模型重力前馈.md](./模型重力前馈.md)** | Pinocchio 定基重力前馈（RQ-025）启用与标定 |
| **[部署与开机自启.md](./部署与开机自启.md)** | systemd 编译重启、开机自启、灯带 sudo 说明 |

## 实机联调快速索引

- **上电 / 站起 / 趴下 / 下电**：`power_sequence_node` + `motor_protocol_node` 门禁；手柄 **`L3` 短按** ≈ `start`（唯一上电），**`□` 短按** ≈ 站起 stand，**`○` 短按** ≈ idle 趴下，**`R3` 短按** ≈ `shutdown`（唯一下电入口），**未上电时 `L1+R1` 长按** ≈ `motor_init`（设零）；详见 **[遥控与电源启停.md](./遥控与电源启停.md)** 与 **`src/trotbot_can_bridge/config/power_sequence.yaml`**。
- **高度跟踪**：`SoftProne`/`SoftStand` 可跟踪 `body_pose.position.z` 与趴下前 **`stand_resume_z`**（参数 `track_body_pose_height` 等）。
- **反馈可视化**：`/joint_states_feedback` + `robot_state_publisher_feedback`（`frame_prefix:=fb/`）；`motor_protocol` 侧 **`feedback_joint_states_timer_hz`** 保证无 CAN 时仍可有 TF 刷新。
- **构建范围**：真机至少 `trotbot` + `trotbot_can_bridge` + `champ` 栈；详见包内 **`本地运行指南.md`** 第 4～5 节。

## 放置原则

- **`docs/trotbot/`**：需求、架构、问题跟踪、评审类（协作型）
- **`src/trotbot/`**：与 launch/config 强绑定的运行说明（执行型）
- **`src/trotbot/docs/images/`**：图片资源
