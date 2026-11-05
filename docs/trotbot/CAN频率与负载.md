# CAN 频率、带宽估算与推荐值（1Mbps）

当前映射为每路 CAN 负责 6 个电机（前腿 can0、后腿 can1）。本文给出轨迹频率与 CAN 帧率/总线负载的关系及调参入口。

## 1) 是不是 1200Hz？

是的，按当前映射（每路 CAN 负责 6 个电机）：

- 轨迹频率设为 `F_traj`（Hz）
- 单路 CAN 下发帧率约为：`F_can_bus = 6 * F_traj`
- 双路合计下发帧率约为：`F_can_total = 12 * F_traj`

所以当 `F_traj = 200Hz` 时：

- 每路 CAN：`6 * 200 = 1200 帧/秒`
- 双路合计：`2400 帧/秒`

## 2) 1Mbps 下的粗略负载（MIT 8字节扩展帧）

按工程估算，一个扩展 CAN 数据帧（8B）上总线约 `130~160 bit/帧`（含仲裁/控制/CRC/ACK/EOF 与位填充近似）。

当 `200Hz` 时，单路下发负载约：

- `1200 帧/s * (130~160 bit) = 156~192 kbit/s`
- 占 `1M` 总线约 `15.6%~19.2%`

若电机反馈帧与下发同量级（近似 1:1），单路总负载约翻倍到：

- `31%~38%` 左右（仍有余量）

## 3) 推荐轨迹频率（1Mbps，当前双路各6电机）

- **推荐起步**：`100~200Hz`（调试稳定、错误率低）
- **常用建议**：`200Hz`（当前实测稳定，可作为默认）
- **可尝试上探**：`250~300Hz`（需重点观察 `fail_rate/drop_*`）
- **不建议长期**：`>300Hz`（突发排队、ENOBUFS 风险明显增大，需更严格限流与优先级设计）

实机以这几个指标判定是否该降频：

- `drop_overflow`、`drop_non_retry`、`drop_retry_exceeded` 是否持续非零
- `fail_rate` 是否持续升高
- `queue` 是否长期不回落

## 4) 各节点频率关系（当前实现）

| 环节 | 当前来源/默认 | 频率关系 |
|---|---|---|
| `quadruped_controller_node` 控制循环与轨迹发布 | `loop_rate`，默认 `200Hz`（CHAMP 源码默认） | 发布 `/joint_group_effort_controller/joint_trajectory` |
| `motor_protocol_node` | 被动订阅，无独立定时器 | 每收到 1 条轨迹，编码 12 帧到 `/can_tx_frames` |
| `can_transport_node` 发送出队 | `rx_poll_ms`（默认 `2ms`） + `max_tx_per_cycle`（默认 `8`） | 理论出队上限约 `500*8=4000 帧/s`（两路合并） |
| 单路 CAN 下发 | 由映射决定每路 6 电机 | `6 * F_traj` |

## 5) 如何修改频率

1) **改轨迹频率（推荐入口）**  
文件：`src/trotbot/config/champ/gait_minidog.yaml`  
参数（`ros__parameters` 顶层）：

```yaml
loop_rate: 200.0
```

例如改为 `150Hz`：

```yaml
loop_rate: 150.0
```

2) **必要时配合传输层节流**  
文件：`src/trotbot_can_bridge/config/bridge.yaml`

- `rx_poll_ms`：轮询周期（ms）
- `max_tx_per_cycle`：每轮最多发送帧数
- `tx_queue_max`：发送队列深度

## 6) 常用“看频率/看负载”命令

```bash
# 轨迹话题频率
ros2 topic hz /joint_group_effort_controller/joint_trajectory

# 桥接收发频率
ros2 topic hz /can_tx_frames
ros2 topic hz /can_rx_frames

# 单行十六进制（需先在 bridge.yaml 打开 publish_hex_lines）
ros2 topic echo /can_frames_tx_line
ros2 topic echo /can_frames_rx_line

# 查看单条消息内容（含 time_from_start 等）
ros2 topic echo /joint_group_effort_controller/joint_trajectory

# 看 CAN 控制器统计（错误/丢包/bus-off）
ip -details -statistics link show can0
ip -details -statistics link show can1
```

## 7) 轨迹频率 -> CAN 帧率小抄（当前 6 电机/路）

估算公式：

- 单路下发帧率：`F_can_bus = 6 * F_traj`
- 双路下发帧率：`F_can_total = 12 * F_traj`
- 单路下发负载估算：`F_can_bus * (130~160 bit/帧) / 1e6`

| 轨迹频率 `F_traj` | 单路下发帧率 | 双路下发帧率 | 单路下发负载估算（1Mbps） | 单路总负载估算（含近似1:1反馈） |
|---:|---:|---:|---:|---:|
| 100 Hz | 600 fps | 1200 fps | 7.8% ~ 9.6% | 15.6% ~ 19.2% |
| 150 Hz | 900 fps | 1800 fps | 11.7% ~ 14.4% | 23.4% ~ 28.8% |
| 200 Hz | 1200 fps | 2400 fps | 15.6% ~ 19.2% | 31.2% ~ 38.4% |
| 250 Hz | 1500 fps | 3000 fps | 19.5% ~ 24.0% | 39.0% ~ 48.0% |
| 300 Hz | 1800 fps | 3600 fps | 23.4% ~ 28.8% | 46.8% ~ 57.6% |

> 注：这是工程近似值（位填充、仲裁竞争、反馈策略会影响实测），最终以 `ip -details -statistics` 与 `TX stats(5s)` 为准（统计日志字段见[《CAN桥架构与话题细节》](./CAN桥架构与话题细节.md)）。
