# RQ-025 实现计划：Pinocchio 重力前馈（定基 + 关节反馈）

## 一、目标与范围

按 `docs/trotbot/软件需求清单.md` 的 RQ-025，首版实现：

1. 新增 `trotbot_dynamics` 包，节点 `pinocchio_tau_ff_node` 用 Pinocchio 从 URDF 建立 **定基（base_link 固定）** 模型；
2. 订阅 `/joint_states_feedback`（12 关节 CHAMP 顺序、URDF 语义），以 50Hz 计算 **广义重力项** `g(q)`，发布到 `/model_tau_ff`；
3. `motor_protocol_node` 订阅 `/model_tau_ff`，把模型力矩经 **符号/缩放/限幅/斜坡** 后并入 MIT `tau_ff`；
4. 默认 **关闭**，可一键启用/关闭，异常或数据过期自动回落到现有 RQ-022 查表/总线偏置；
5. 同步文档与验证步骤。

首版只做 **定基、仅重力项（v=0, a=0）**，不做浮基、科氏力、摩擦、接触力、MPC。

## 二、现状分析（已核实）

- RQ-025 明确推荐“方案 A：独立 `trotbot_dynamics` 节点 + 桥接节点订阅 `/model_tau_ff`”，并要求优先用 `/joint_states_feedback` 的 `position`。见 `docs/trotbot/软件需求清单.md` L130-L153。
- `src/trotbot_dynamics/` 目前只有空的 `src/`、`launch/`，**没有 package.xml/CMakeLists**，需要从零建包。
- Pinocchio 已安装：`ros-humble-pinocchio 4.0.0`；CMake 包位于 `/opt/ros/humble/lib/aarch64-linux-gnu/cmake/pinocchio`，聚合目标为 `pinocchio::pinocchio`。
- Pinocchio API 已确认：
  - `pinocchio::urdf::buildModelFromXML(const std::string&, Model&, verbose=false, mimic=false)`，位于 `pinocchio/parsers/urdf.hpp`；
  - `pinocchio::computeGeneralizedGravity(model, data, q)`，位于 `pinocchio/algorithm/rnea.hpp`，对 `v=0` 的静力保持力矩返回 `g(q)`。
- URDF：`src/trotbot/urdf/minidog_champ.urdf.xacro` 为 `base_link` 根 + 12 个 revolute + 固定足端/头部，未连接 floating joint，Pinocchio 按定基解析，预期 `nq=nv=12`。关节名与 `DogMapper::kChampJointNames` 完全一致（`base_lf1, lf1_lf2, lf2_lf3, ...`）。
- 现有 MIT 前馈入口：`src/trotbot_can_bridge/src/motor_protocol_node.cpp`
  - 参数声明 L109-L166；
  - `ComputeMitTorqueFf()` L803-L812；
  - `SendMitFrame()` L422-L494、`OnTxRefreshTimer()` L496-L545 都调用 `ComputeMitTorqueFf`；
  - 运行参数回调 `OnSetParameters()` L301-L350；
  - 门禁回调 `OnPowerGate()` L583-L594；
  - 成员变量 L1050-L1160。
- 现有反馈：`motor_protocol_node` 发布 `/joint_states_feedback`（`JointState`，SensorDataQoS，约 50Hz），`position` 已是 URDF/CHAMP 关节角，可直接作为 `q`。
- `robot_state_publisher` 只在 `rviz:=true` 时启动（`trotbot_basic.launch.py` L150-L168），**不能**作为模型来源；模型须由 launch 通过 `xacro` 生成字符串传入新节点。
- 现有 RQ-022 查表前馈：`control_gains.yaml` L44-L50，默认 `enable_joint_tau_ff=false`，`gravity_ff_scale=0.5`。

## 三、代码改动

### 3.1 新建 `src/trotbot_dynamics` 包（方案 A）

新增文件：

- `src/trotbot_dynamics/package.xml`
  - `ament_cmake`，依赖 `rclcpp`、`sensor_msgs`、`std_msgs`、`pinocchio`、`trotbot_can_bridge`（仅头文件 `DogMapper`）。
- `src/trotbot_dynamics/CMakeLists.txt`
  - `find_package(pinocchio REQUIRED)`、`find_package(trotbot_can_bridge REQUIRED)`；
  - `add_executable(pinocchio_tau_ff_node src/pinocchio_tau_ff_node.cpp)`；
  - `target_link_libraries(pinocchio_tau_ff_node pinocchio::pinocchio)`；
  - 安装 `launch/`、`config/`。
- `src/trotbot_dynamics/config/pinocchio_tau_ff.yaml`
  ```yaml
  /**:
    ros__parameters:
      joint_states_topic: /joint_states_feedback
      output_topic: /model_tau_ff
      publish_hz: 50.0
      input_timeout_s: 0.2
      # robot_description 由 launch 通过 xacro 生成后注入；urdf_file 仅用于手工调试普通 .urdf
      robot_description: ""
      urdf_file: ""
  ```
- `src/trotbot_dynamics/src/pinocchio_tau_ff_node.cpp`
  - 参数：`robot_description`、`urdf_file`、`joint_states_topic`、`output_topic`、`publish_hz`、`input_timeout_s`。
  - 模型加载优先级：`robot_description` 非空 → `buildModelFromXML`；否则读 `urdf_file` → `pinocchio::urdf::buildModel`。两者都为空/失败：`RCLCPP_FATAL` 后抛出异常，让节点退出（不静默降级）。
  - 模型校验：
    - `model.nq == 12 && model.nv == 12`；
    - 12 个 `kChampJointNames` 都能通过 `model.getJointId(name)` 找到，且每个关节 `nq == 1`；
    - 记录 `name -> idx_q` 映射。
  - 订阅 `/joint_states_feedback`（`rclcpp::SensorDataQoS()`），**按关节名映射**而非下标；缺关节/非有限值时丢弃该帧并节流告警。
  - 定时器 `publish_hz`（默认 50Hz）：若最近一帧反馈年龄 ≤ `input_timeout_s`，计算：
    ```cpp
    q.setZero();
    q[idx_q[i]] = latest_position[i];
    pinocchio::computeGeneralizedGravity(model, data, q);
    ```
  - 发布 `sensor_msgs/msg/JointState` 到 `/model_tau_ff`：
    - `name` = 12 个 `kChampJointNames`；
    - `effort` = `g(q)`（URDF 关节空间，Nm）；
    - `position`/`velocity` 留空；
    - QoS `SensorDataQoS()`。
  - 节点只发布 **未缩放、URDF 符号** 的原始重力力矩；缩放与符号转换统一放在 `motor_protocol_node` 的单点函数内（满足 RQ-025 “单点函数完成 URDF τ → 电机 τ_cmd”）。
  - 1Hz 日志：反馈年龄、最近计算耗时（µs）、`max|g(q)|`、发布计数。
- `src/trotbot_dynamics/launch/pinocchio_tau_ff.launch.py`
  - 参数 `description_file`（默认 `minidog_champ.urdf.xacro`）、`publish_hz`、`output_topic`；
  - `description_path = FindPackageShare("trotbot")/urdf/description_file`；
  - 节点参数 `robot_description = ParameterValue(Command(["xacro ", description_path]), value_type=str)`。

### 3.2 `trotbot_can_bridge` 导出头文件

`src/trotbot_can_bridge/CMakeLists.txt` 增加：

```cmake
ament_export_include_directories(include)
```

用途：让 `trotbot_dynamics` 复用 `DogMapper::kChampJointNames`，避免关节名重复定义。`package.xml` 增加 `<depend>trotbot_can_bridge</depend>`（在 dynamics 侧）。

### 3.3 `motor_protocol_node` 接入模型前馈

新增参数（默认保持现有行为）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `pinocchio_tau_ff_mode` | `off` | `off` / `replace` / `add`；`replace` 忽略查表，`add` 与查表叠加 |
| `pinocchio_tau_ff_topic` | `/model_tau_ff` | 模型前馈话题 |
| `pinocchio_tau_ff_scale` | `0.5` | 总增益，建议 <1 |
| `pinocchio_tau_ff_timeout_s` | `0.2` | 超过该年龄视为过期，回落 |
| `pinocchio_tau_ff_ramp_s` | `1.0` | 门禁打开后线性斜坡，避免力矩阶跃 |
| `tau_signs` | 空 | 12 维 URDF→电机符号；为空时回落到 `joint_signs` 并告警 |

新增状态：

```cpp
std::array<double, 12> model_tau_ff_urdf_{};
std::array<bool, 12> model_tau_ff_valid_{};
rclcpp::Time model_tau_ff_stamp_;
bool has_model_tau_ff_{false};
int64_t model_ff_ramp_start_ns_{0};
rclcpp::Subscription<JointState>::SharedPtr model_tau_sub_;
```

订阅：始终创建（便于运行期切模式），`SensorDataQoS()`，回调 `OnModelTauFf`：

- 校验 `name` 能覆盖全部 12 个 CHAMP 关节，按名字写回固定顺序；
- 任一值非有限或缺失 → 忽略整帧；
- 成功则 `stamp = now()`、`has_model_tau_ff_ = true`。

`ComputeMitTorqueFf()` 改为非 const，逻辑：

```cpp
const double bus_tau = is_front ? runtime_tau_can0_ : runtime_tau_can1_;
const double table_tau = enable_joint_tau_ff_
  ? gravity_ff_scale_ * tau_ff_nominal_nm_[idx] : 0.0;
double tau = bus_tau;

if (mode == OFF) { tau += table_tau; return Clamp(...); }

const bool fresh = has_model_tau_ff_ &&
  (now - model_tau_ff_stamp_).seconds() <= pinocchio_tau_ff_timeout_s_;
if (!fresh) {
  // 节流 WARN：model tau_ff stale，fallback
  tau += table_tau;
  return Clamp(...);
}

const double ramp = RampFactor(now);  // 0→1，ramp_s<=0 时直接 1
const double model_tau = model_tau_ff_urdf_[idx] * tau_signs_[idx] *
                         pinocchio_tau_ff_scale_ * ramp;
if (mode == REPLACE) {
  tau += model_tau;                 // 查表被忽略
} else {                            // ADD
  tau += table_tau + model_tau;     // 叠加，文档提示可能双计重力
}
return Clamp(tau, kTMin, kTMax);
```

- `tau_signs` 的数学初值：由 `θ_motor = s·θ_champ + offset` 的虚功关系得到 `τ_motor = s·τ_champ`，因此初值取与当前 `joint_signs` 相同；但按 RQ-025 要求**单独配置、单独验证**，不直接读取 `joint_signs`。
- `OnPowerGate(true)` 时置 `model_ff_ramp_start_ns_ = now()`；门禁关闭或 mode 切回 `off` 时清零。
- `OnSetParameters` 增加上述参数校验：
  - `mode` 只接受 `off/replace/add`，非法值拒绝或回落 `off`；
  - `scale >= 0` 且建议 ≤1，超 1 只告警不拒绝（保留调试空间）；
  - `timeout_s > 0`、`ramp_s >= 0`；
  - `tau_signs` 必须 12 个且非 0。
- `LogTxWindowStats()` 的 5s 日志追加：`pinocchio_mode`、模型数据年龄、`max|model_tau|`、`tau_signs` 是否已完成显式配置。

### 3.4 `control_gains.yaml` 配置

在 RQ-022 段之后新增：

```yaml
# --- RQ-025 Pinocchio 定基重力前馈（默认关闭）---
# off=不启用；replace=用模型前馈替代查表；add=模型+查表叠加（注意双计重力）
pinocchio_tau_ff_mode: "off"
pinocchio_tau_ff_topic: /model_tau_ff
pinocchio_tau_ff_scale: 0.5
pinocchio_tau_ff_timeout_s: 0.2
pinocchio_tau_ff_ramp_s: 1.0
# URDF τ -> 电机 τ_cmd 的符号（初值同 joint_signs；必须做静力/对拉验证后固化）
tau_signs: [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0]
```

### 3.5 Launch 接入

- `src/trotbot_can_bridge/launch/can_bridge.launch.py`
  - 新增 arguments：`use_pinocchio_tau_ff`（默认 `false`）、`description_file`（默认 `minidog_champ.urdf.xacro`）、`pinocchio_tau_ff_scale`（默认 `0.5`）；
  - 当 `use_pinocchio_tau_ff:=true`：
    - include `trotbot_dynamics/launch/pinocchio_tau_ff.launch.py`，传 `description_file`；
    - `motor_protocol_node` 参数覆盖 `pinocchio_tau_ff_mode:=replace`、`pinocchio_tau_ff_scale`。
  - 条件字符串用 `PythonExpression` 生成，`pinocchio_tau_ff_scale` 用 `ParameterValue(..., value_type=float)`。
- `src/trotbot/launch/trotbot_basic.launch.py`
  - 新增 `use_pinocchio_tau_ff`（默认 `false`）；
  - 把 `use_pinocchio_tau_ff`、`description_file` 透传给 `can_bridge_launch`，确保模型与当前选择的 URDF 一致。

### 3.6 文档

- `docs/trotbot/架构说明.md`
  - §2.1 节点表加入 `pinocchio_tau_ff_node`；
  - §2.2 话题表加入 `/model_tau_ff`（`sensor_msgs/JointState`，dynamics -> protocol）；
  - 在 RQ-022 段后补 RQ-025 数据流：`/joint_states_feedback → pinocchio_tau_ff_node → /model_tau_ff → motor_protocol_node → MIT tau_ff`，注明定基、仅重力、50Hz、超时回落。
- `src/trotbot_can_bridge/README.md`
  - 在“在线参数：关节前馈表（RQ-022）”后新增“模型前馈（RQ-025）”：
    - 启动方式：`ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true use_pinocchio_tau_ff:=true`；
    - 参数表、`replace/add` 语义、过期回落、限幅；
    - 符号验证与从小到大调增益步骤。
- 根 `README.md` §5 增补一段模型前馈的快速启用命令与 `/model_tau_ff` 查看方式。
- `docs/trotbot/软件需求清单.md`
  - RQ-025 状态更新为“首版代码完成（定基重力项），待实机 DoD 对比”；在验收标准中勾选文档化项，保留实机对比项未完成。

## 四、关键决策

| 项 | 决策 |
|---|---|
| 架构 | 采用 RQ-025 方案 A：独立节点 + 话题 |
| 基座 | 定基；模型根为 `base_link`，不引入浮动基 |
| 动力学项 | 首版仅 `computeGeneralizedGravity`（等价 v=0 RNE），不含科氏/摩擦 |
| 输入 | 只用 `/joint_states_feedback.position`，不与指令轨迹融合 |
| 输出 | `JointState`，仅填 `name` + `effort`，50Hz |
| 符号 | `tau_signs` 独立参数，初值按虚功关系取 `joint_signs`，必须静力验证 |
| 缩放 | 消费端 `pinocchio_tau_ff_scale`，默认 0.5，建议 <1 |
| 叠加 | `replace` / `add` 显式选择；`replace` 优先，避免双计重力 |
| 安全 | 默认 `off`；过期回落；限幅 `kTMin/kTMax`；门禁打开后 1s 斜坡 |
| 模型来源 | launch 用 `xacro` 生成 `robot_description` 注入，不依赖 `robot_state_publisher` |

## 五、验证步骤

1. **编译**
   ```bash
   env -u LD_LIBRARY_PATH bash -c 'source /opt/ros/humble/setup.bash && \
     colcon build --packages-select trotbot_can_bridge trotbot_dynamics trotbot --symlink-install'
   ```
2. **模型与数值自检（无硬件）**
   - 生成临时 URDF：`xacro src/trotbot/urdf/minidog_champ.urdf.xacro > /tmp/minidog.urdf`；
   - 启动 `ros2 launch trotbot_dynamics pinocchio_tau_ff.launch.py`；
   - 用 `ros2 topic pub` 发一帧 12 关节 `JointState`（站姿/趴姿各一组）；
   - `ros2 topic echo /model_tau_ff --once`，确认 12 个 `effort` 有限；
   - 用 Python Pinocchio 对同一 `q` 计算 `computeGeneralizedGravity`，与节点输出逐项对比（误差 < 1e-6）。
3. **链路接线**
   - `ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true use_pinocchio_tau_ff:=true rviz:=false`；
   - 确认 `pinocchio_tau_ff_node` 启动、`/model_tau_ff` 约 50Hz、`motor_protocol_node` 参数 `pinocchio_tau_ff_mode=replace`；
   - 观察 `MIT publish:` 日志中的 tau 与模型值一致（乘 `scale` 与 `tau_signs` 后）。
4. **过期回落**
   - 运行中结束 `pinocchio_tau_ff_node`；
   - 确认 `motor_protocol_node` 节流 WARN，`tau_ff` 回落到查表/总线偏置，无阶跃。
5. **实机符号与增益（DoD，需上机）**
   - 空载/趴姿，`pinocchio_tau_ff_scale=0.2`，`mode=replace`，逐腿观察关节误差与机体高度；
   - 若某关节误差立即变大 → 单独翻转该关节 `tau_signs[idx]`（每次只改一路）；
   - 符号确认后按 0.2 → 0.35 → 0.5 提升，对比站立/慢走的 `abs_err` 均值/最大值与姿态；
   - 记录“开启 vs 关闭”数据，满足 RQ-025 DoD。
6. **回归**
   - `pinocchio_tau_ff_mode=off` 时行为与当前版本一致；
   - RQ-022 `enable_joint_tau_ff` 仍独立可用；
   - `add` 模式仅做人工验证时使用，默认文档不推荐。

## 六、风险与边界

- **模型误差**：URDF 质量/惯量/安装位置与实机有差异，重力矩可能偏大或偏小；用 `scale<1` + 斜坡 + 默认关闭控制风险。
- **符号错误**：会导致正反馈、加重下塌；必须逐关节静力验证，`tau_signs` 不做隐式推导。
- **基座姿态假设**：定基模型假设机身水平；实际 roll/pitch 时重力投影有误差。首版接受该限制，二期可接 IMU 或 `body_pose` 调整根姿态。
- **反馈噪声**：重力项对 `q` 平滑，50Hz 足够；不引入速度项可避免反馈速度噪声放大。
- **启动顺序**：模型数据过期时回落；`/model_tau_ff` 尚未到达不会阻塞使能流程。
- **不做**：浮基 RNE、科氏力、摩擦/阻尼辨识、接触力估计、MPC/TSID、任务空间控制。

## 七、实施顺序

1. 建 `trotbot_dynamics` 包与节点；
2. `trotbot_can_bridge` 导出头文件；
3. `motor_protocol_node` 参数、订阅、合并逻辑与运行参数回调；
4. `control_gains.yaml` 增加参数；
5. launch 接入（dynamics、can_bridge、trotbot_basic）；
6. 编译 + 离线数值自检；
7. 文档同步；
8. 交付实机调参步骤（符号验证、增益阶梯、DoD 对比）。
