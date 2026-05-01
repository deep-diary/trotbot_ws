# TrotBot 文档导航（`docs/trotbot`）

该目录按「长期沉淀、跨模块协作」放在工作空间 `docs/trotbot/`，与包内执行型文档分工如下。

## 核心文档

| 文件 | 内容 |
|------|------|
| **[软件需求清单.md](./软件需求清单.md)** | RQ-001～RQ-018 等需求与实现状态（CHAMP→CAN→反馈→RViz） |
| **[架构说明.md](./架构说明.md)** | As-Is 节点/话题/数据流、与 To-Be 演进建议 |
| **[开发问题跟踪.md](./开发问题跟踪.md)** | ISSUE 模板与已登记问题（含 RViz 反馈、`enable_rx_decode_log`、启停编排等） |
| **[本地运行指南.md](./本地运行指南.md)** | 运行入口摘要；**完整步骤见 `src/trotbot/本地运行指南.md`** |

## 实机联调快速索引

- **上电 / 趴下 / 下电**：`power_sequence_node` + `motor_protocol_node` 门禁；手柄 **`L1+R1` 或 `□` 长按** ≈ `start`，**`○` 长按** ≈ `prone`，**`L1+R1+Share`** ≈ `shutdown`；详见根目录 **`README.md`** 2a-1 与 **`src/trotbot_can_bridge/config/power_sequence.yaml`**。
- **高度跟踪**：`SoftProne`/`SoftStand` 可跟踪 `body_pose.position.z` 与趴下前 **`stand_resume_z`**（参数 `track_body_pose_height` 等）。
- **反馈可视化**：`/joint_states_feedback` + `robot_state_publisher_feedback`（`frame_prefix:=fb/`）；`motor_protocol` 侧 **`feedback_joint_states_timer_hz`** 保证无 CAN 时仍可有 TF 刷新。
- **构建范围**：真机至少 `trotbot` + `trotbot_can_bridge` + `champ` 栈；详见包内 **`本地运行指南.md`** 第 4～5 节。

## 放置原则

- **`docs/trotbot/`**：需求、架构、问题跟踪、评审类（协作型）
- **`src/trotbot/`**：与 launch/config 强绑定的运行说明（执行型）
- **`src/trotbot/docs/images/`**：图片资源
