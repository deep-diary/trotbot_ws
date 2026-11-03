# RViz 默认单模型（实时反馈）+ 帧率 30→15 降载方案

## 摘要

当前 RViz 同时渲染两个 RobotModel（期望 `/robot_description` + 实测 `/robot_description_feedback`），加上 30 FPS 全局刷新，rviz2 常驻消耗约 39–47% 单核，是整机最大的 CPU 消耗项。本方案：

1. 默认只显示 **实时反馈模型**（`RobotModelFeedback`），期望模型 `RobotModel` 保留在配置中但默认关闭，勾选即可恢复双模型对比；
2. 全局 `Frame Rate` 由 30 改为 15；
3. 不动 launch/节点：两个 `robot_state_publisher` 与 `fb/` TF 桥继续运行，保证双模型随时可用。

只改 1 个配置文件，无需 `colcon build`（`install` 下为指向源码的符号链接），重启 rviz2 即可生效。

## 现状分析（实测）

### CPU / 负载（8 核，2026-11-03 21:07 实测）

| 进程 | 单核占用 | 说明 |
|------|---------|------|
| rviz2 (13501) | 39–47% | 最大消耗项，双模型 + 30FPS |
| ds4_feedback_node (13461) | 28–40% | 与本次改动无关，但属于第二大消耗项 |
| motor_protocol_node | ~13% | CAN 编解码 |
| state_estimation_node | 7–9% | CHAMP 状态估计 |
| can_transport_node | 6–8% | CAN 收发 |
| robot_state_publisher ×2 | 各 2.5–3% | 期望 / 反馈 TF |

- `load average: 3.10, 2.75, 2.40`，`%Cpu: 31.7 us / 10.1 sy / 56.1 id`（约 44% 占用，8 核）。
- 期间有一次 `cc1plus` 编译（94%）在跑，属于临时负载；稳态主要就是上表。
- 话题实测：`/joint_states` 与 `/joint_states_feedback` 均 **50.0 Hz**，`/diagnostics` 50 Hz，`/motor_feedback` ≈114 Hz，`/tf` ≈33 Hz。`/can_tx_frames` 在 Idle 无下发时无数据。

### RViz 双模型结构

配置文件：[trotbot.rviz](file:///home/cat/trotbot_ws/src/trotbot/rviz/trotbot.rviz)

| 行 | 内容 |
|----|------|
| 55–165 | `RobotModel`（Name: `RobotModel`），话题 `/robot_description`，Alpha 0.8，橙褐色，**Enabled: true / Value: true** |
| 166–276 | `RobotModelFeedback`，话题 `/robot_description_feedback`，TF Prefix `fb`，Alpha 0.75，蓝青色 |
| 277–399 | `TF` 显示（含 `fb/*` 帧，箭头/坐标轴/名称全开） |
| 403–404 | `Fixed Frame: base_link`，**`Frame Rate: 30`** |

配套节点在 [trotbot_basic.launch.py](file:///home/cat/trotbot_ws/src/trotbot/launch/trotbot_basic.launch.py#L150-L213)：
- `robot_state_publisher`（期望）→ `/robot_description` + 无前缀 TF
- `robot_state_publisher_feedback` → `/robot_description_feedback` + `fb/` 前缀 TF，订阅 `/joint_states_feedback`（实机数据来自 `motor_protocol_node` 按 50Hz 发布的电机反馈角）
- `feedback_tf_bridge`（static_transform）`base_link → fb/base_link`
- 三者都只在 `rviz:=true` 时启动；真机模式 `main.sh` 默认 `rviz:=false`，当前实例是用户手动加 `rviz:=true` 启动的。

`/home/cat/trotbot_ws/install/trotbot/share/trotbot/rviz/trotbot.rviz` 是 **软链接 → 源码同文件**，因此改源码即生效（注意不要改 `src/install/` 下的旧副本）。

## 变更内容

### 1. 修改 `src/trotbot/rviz/trotbot.rviz`（唯一改动文件）

**a) 默认关闭期望模型，保留条目（双模型备用）** 第 55–165 行 `RobotModel` 块内：

```
      Enabled: true      →  Enabled: false        # 第 66 行（Description Topic 为 /robot_description 的那块）
      Value: true        →  Value: false          # 第 164 行（TF Prefix: "" 下方，显示列表的勾选值）
```

- 仅这两行，`Name: RobotModel`、话题、Alpha 等全部保留；在 RViz `Displays` 面板勾选 `RobotModel` 即恢复原双模型对比。
- 反馈模型 `RobotModelFeedback`（166–276 行）保持 `Enabled/Value: true` 不动，外观（蓝青、Alpha 0.75）也不改，这样切回双模型时配色对比关系不变。

**b) 全局帧率减半** 第 404 行：

```
    Frame Rate: 30   →    Frame Rate: 15
```

**c) 不改动的部分（明确决策）**

- 不删任何显示项，不改 `TF` 显示、`Fixed Frame`、视图、窗口几何。
- 不改 launch 文件：两个 `robot_state_publisher`、`feedback_tf_bridge` 保持启动（各约 2.5–3% CPU），这是"双模型备用"的前提——若停掉期望模型的 rsp，勾选后也没有 TF/描述可选。
- 不把 `RobotModelFeedback` 改为不透明：保持 Alpha 0.75，保证双模型叠加时仍能区分。
- 不调整 `Update Interval`（保持 0）。

### 2. 生效方式（不编译）

因为 `install` 是符号链接，只需重启 rviz2：

```bash
pkill -f "/opt/ros/humble/lib/rviz2/rviz2" || true
DISPLAY=:0 nohup /opt/ros/humble/lib/rviz2/rviz2 \
  -d /home/cat/trotbot_ws/install/trotbot/share/trotbot/rviz/trotbot.rviz \
  > /tmp/rviz2.log 2>&1 &
```

不加 `--ros-args` 也能读到同域话题/TF（其余节点保持运行，无需重启整机栈）。

## 假设与决策

1. "单模型"= 只显示实时反馈模型；"保留双模型"= 显示条目保留、默认关闭、UI 一键恢复，而不是另建 `dual.rviz` 文件或新 launch 参数（避免多份配置漂移，且 RViz 无法从命令行切换单个 display）。
2. 期望模型的 rsp 继续运行，接受约 2.5–3% 单核开销换取双模型随时可开。
3. 帧率 15 FPS 对调试观察足够；`Frame Rate` 是全局渲染节拍，与 50Hz 控制/CAN 无关，只影响显示刷新，不会降低反馈数据频率。
4. 只做用户要求的两项；`ds4_feedback_node`（28–40%）、`TF` 全帧显示等其它可降载点本次不动（可后续单独评估）。
5. 观察项（不在本次范围）：当前有两个 `power_sequence_node` 实例（PID 33811/56926，其中一个用相对路径启动），属于重复启动，未知是否影响 CAN 时序，仅记录。

## 验证步骤

1. 配置落地检查：
   - `grep -n "Frame Rate" src/trotbot/rviz/trotbot.rviz` → `15`
   - `grep -n -A2 "Value: /robot_description$" -A6 ...` 人工确认期望模型块 `Enabled: false`、其 `Value: false`，反馈模型仍为 `true`
2. 视觉检查（RViz 窗口）：默认只见一个蓝青色模型；`Displays` 中 `RobotModel` 未勾选、`RobotModelFeedback` 已勾选；勾选 `RobotModel` 后立即出现橙色期望模型（双模型功能可用），再取消勾选恢复单模型。窗口右下角帧率显示应为 ~15 FPS（若有 FPS 状态栏）。
3. CPU 对比：`top -b -n2 -d 2 -p $(pgrep -f "rviz2 -d")`，rviz2 单核占用预期从 39–47% 降到 20–28%；再整体看 `top` 的 `%Cpu idle`。
4. 数据链路不变：`ros2 topic hz /joint_states_feedback` 仍 ≈50 Hz；`/can_tx_frames`（行走/使能时）频率应不低于改动前（若此前受 CPU 挤压有丢帧，应可见改善）。

## 回滚

改回两处即可：`Enabled: true` / `Value: true`（第 66、164 行）、`Frame Rate: 15 → 30`，重启 rviz2。
