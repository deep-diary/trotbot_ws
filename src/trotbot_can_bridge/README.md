## 实测报文：

data: RX can1 EXT id=0x0600FD0B len=08 01 01 00 00 00 00 00 00   电机设置零位
---
data: RX can1 EXT id=0x02000BFD len=08 7F FF 7F D1 7F FF 01 04   电机反馈报文
---
data: RX can1 EXT id=0x0300FD0B len=08 00 00 00 00 00 00 00 00   电机使能
---
data: RX can1 EXT id=0x02800BFD len=08 80 00 80 19 7F FF 01 04   电机反馈报文
---
data: RX can1 EXT id=0x017FFF0B len=08 8A 3C 7F FF 0F 5C 4C CC      运控报文：扭矩：0，位置：1， 速度：0， kp：30，kd：1.5
---
data: RX can1 EXT id=0x02800BFD len=08 7F FF 80 2B 7F F6 01 04      电机反馈报文
---
data: RX can1 EXT id=0x0400FD0B len=08 00 00 00 00 00 00 00 00      电机失能
---
data: RX can1 EXT id=0x02000BFD len=08 8A 32 80 67 80 DF 01 04      电机反馈报文

---

## `el05_motor_cansend.sh`：12 电机批处理（cansend）

脚本路径：**`scripts/el05_motor_cansend.sh`**（安装后：`share/trotbot_can_bridge/scripts/el05_motor_cansend.sh`，与工作空间 install 前缀有关）。

电机 ID 与腿的对应关系见 **`src/trotbot/hal/ref/dog/README.md`**（11–13 / 21–23 / 51–53 / 61–63）。

CAN ID 组装与仓库内 **`hal/ref/motor/protocol_motor.cpp` 的 `buildCanId`** 一致：**`CMD<<24 | MOTOR_MASTER_ID<<8 | motor_id`**（默认 **`MOTOR_MASTER_ID=0xFD`**，十进制 **`253`**）。

### 依赖

```bash
sudo apt-get install -y can-utils
```

总线需已配置波特率并已 `up`（例如 1Mbps）。

`/can_tx_frames`（`motor_protocol_node` → `can_transport_node`）默认使用 **Reliable + `KeepLast(can_tx_frames_qos_depth)`**（深度默认 **4000**，与 `control_gains.yaml` / `bridge.yaml` 一致），避免高频下发时 **DDS 丢帧**导致总线统计假象（ISSUE-0002）。

`can_bridge.launch.py` 现默认同时启动 `power_sequence_node`（可用 `use_power_sequence:=false` 关闭）。该节点用于遥控上/下电编排：发布 `/power_sequence/gate_open` 控制轨迹门禁，并在启停阶段发布 `body_pose` 做 z 轴软升降。启动流程可选 **主动上报**：`power_sequence.yaml` 中 **`enable_active_report_in_startup`**（默认 true）、**`enable_active_report_at_launch`**（默认 true：节点启动后 **`active_report_boot_delay_s`** 秒即在 **Idle** 发 0x7026 + 0x18 ON，**不必等 start**）、**`active_report_hz`**（默认 10，写入 **0x7026**）、**`active_report_master_id`**（默认 -1 与 `motor_master_id` 相同）。**下电 `Disable` 只发失能、不关闭主动上报**，以便 shutdown 后 RViz 等仍能靠 **0x18** 刷新；需要「上报关 + 失能」请用 **`el05_motor_cansend.sh reset`**。**0x18** 上报帧由 **`motor_protocol_node`** 与 **0x02** 一并解析进 `/joint_states_feedback`。

默认按键（`power_sequence.yaml` 可改）：

- `L1 + R1` **或** **`□`（Square）** 长按：上电 / `start`（`Idle`→`Precheck…`；`ProneHold`→`SoftStand`），时长用 **`start_longpress_s`**
- `L1 + R1 + Share` 长按：下电收拢（`SoftProne -> Disable -> Idle`，与话题 `shutdown` 同语义）
- `Circle`（`○`）长按：趴下至 **`ProneHold`**（保持使能、门禁仍开），时长用 **`prone_longpress_s`**，与话题 **`prone`** 同语义
- **`Options`**（索引 **`button_option`**，默认 9）长按 **`set_zero_longpress_s`**：**机械零位**（CMD 0x06），成功触发后默认 **`set_zero_repeat_count`**（如 3）轮完整 `SendSetZeroAll`，轮间隔 **`set_zero_repeat_interval_ms`**；仅 **`Idle`** 且门禁关；与 URDF `joint_offsets` 不同层；勿与脚本 **`zero`** 并发

### 无电机使能的安全抓包模式（推荐先联调）

可以在启动时关闭“上电使能”与“初始化 MIT 零位帧”，这样可先验证状态机、话题和 CAN 报文链路，避免电机动作带来结构风险。

```bash
ros2 launch trotbot trotbot_basic.launch.py \
  use_can_bridge:=true \
  use_teleop:=true \
  use_joystick:=true \
  use_xterm:=false \
  rviz:=false \
  use_power_sequence:=true \
  power_sequence_file:=/home/cat/trotbot_ws/src/trotbot_can_bridge/config/power_sequence.yaml
```

然后单独覆写参数（或直接改 `power_sequence.yaml`）：

```bash
# 建议联调时先设为 false，确认逻辑后再打开
send_enable_in_startup: false
send_mit_zero_in_startup: false
```

### 无遥控器触发启停（话题指令）

`power_sequence_node` 现支持订阅 `/power_sequence/command`（`std_msgs/String`）：

- `start`：`Idle` 上电流程；**`ProneHold`** 时再次站起（`SoftStand -> Running`）
- `prone`：仅 **`Running`** → `SoftProne` → **`ProneHold`**（不发 Reset）
- `shutdown`：`ProneHold` 直接 `Disable`；其余允许态先 `SoftProne` 再 `Disable`（见根 `README.md` 2a-1 表）
- `set_zero`：十二电机机械零位（同上 gate）；非 Idle 拒绝

示例：

```bash
ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: 'prone'}"
ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: 'shutdown'}"
ros2 topic pub --once /power_sequence/command std_msgs/msg/String "{data: 'set_zero'}"
```

状态观测：

```bash
ros2 topic echo /power_sequence/state
ros2 topic echo /power_sequence/gate_open
```

### 用法

```bash
# 源码目录直接跑
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh --help

# 编译安装后（路径按你的 prefix 调整）
bash install/trotbot_can_bridge/share/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero
```

子命令：

| 子命令 | 说明 |
|--------|------|
| `zero` | 12 电机依次「设置零位」（CMD=6，`data` 首字节 `01`） |
| `init` | 依次「写上报间隔参数 0x7026」→「使能」（CMD=3）→「通信类型 24 主动上报开」 |
| `reset` | 依次「上报关」（同上帧 Byte6=`00`）+ 「失能」（CMD=4，数据全 0） |
| `all` | `zero` → 短暂停顿 → `init` |
| `mit` | 发送一轮 MIT 运控帧（CMD=1）。默认：`p=0`、`v=0`、`kp=20`、`kd=1.5`、`tau=0` |
| `zero_sta_read` | 读取参数 `0x7029`（通信类型17），打印原始应答与解析 `value_u8`（若固件不回17则会超时告警） |
| `zero_sta_set` | 写参数 `0x7029`（通信类型18，值来自 `ZERO_STA_TARGET`，默认 `1`） |
| `zero_sta_save` | 参数保存（通信类型22，掉电保留） |
| `zero_sta_apply` | 一键执行：`zero_sta_set` → `zero_sta_save` → `zero_sta_read` |
| `check` | 快速链路自检：逐个电机发送 `set_zero` 并等待反馈帧（CMD=2） |
| `ping` | 仅打印每个电机将使用的 **接口名**（不发送） |

### 接口分配（默认）

与当前桥接约定一致：**前腿（11–23）→ `can0`，后腿（51–63）→ `can1`**。可通过环境变量覆盖：

- **`IFACE`**：若设置，则 **本轮所有帧走该接口**（常用于台架共总线）。
- **`IFACE_FRONT`** / **`IFACE_REAR`**：未设 `IFACE` 时的前、后腿 SocketCAN 名（默认 `can0` / `can1`）。
- **`BUS=rear`**：**只处理后腿 6 个 ID（51–63）**，且 **只往 `IFACE_REAR`（默认 can1）发**，不会访问 `can0`（适合 can0 未接线、仅在 can1 上测后腿）。
- **`BUS=front`**：**只处理前腿 6 个 ID**，且 **只往 `IFACE_FRONT` 发**。
- **`MOTOR_IDS`**：空格分隔自定义列表（如 `13`）；若某 ID 默认映射到未接线的口，请同时设 **`IFACE=can1`**（或相应的 `IFACE_FRONT`/`IFACE_REAR`）指定实际总线。
- **`SLEEP_MS`**：帧间隔，默认 **20 ms**，减轻 **`write: No buffer space available`**（与其它 CAN 发送并存时）。
- **`REPORT_HZ`**：`init` 时目标主动上报频率（Hz），默认 **5**（降负载）。
- **`REPORT_SCAN_TIME`**：直接指定 `EPScan_time`（`0x7026`，uint16），优先于 `REPORT_HZ`。
- **`CHECK_TIMEOUT_S`**：`check` 子命令等待单电机反馈超时（秒），默认 **0.40**。
- **`PARAM_READ_TIMEOUT_S`**：`zero_sta_read` 等待读参数应答超时（秒），默认 **0.40**。
- **`ZERO_STA_TARGET`**：`zero_sta_set/zero_sta_apply` 写入值（`0` 或 `1`，默认 `1`）。

### `zero_sta(0x7029)` 说明与建议流程

按 EL05 手册：

- `zero_sta=0`：上电位置域 `0~2π`（负向小偏移可能被映射到接近 `2π`）
- `zero_sta=1`：上电位置域 `-π~π`（更适合趴姿断电后的双向小偏移恢复）

建议一次性执行：

```bash
# 全 12 电机：写 zero_sta=1，保存，再回读
ZERO_STA_TARGET=1 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero_sta_apply
```

也可以分步：

```bash
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero_sta_read
ZERO_STA_TARGET=1 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero_sta_set
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero_sta_save
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero_sta_read
```

### 主动上报频率（`EPScan_time=0x7026`）说明

按手册截图规则：

- `scan_time = 1` 对应约 `10ms`（约 100Hz）
- 每增加 1，周期增加 `5ms`

脚本 `init` 默认会把频率降到 **5Hz**（约 `200ms`，即 `scan_time=39`），以减轻总线负载。  
你可以按需覆盖：

```bash
# 默认（5Hz）
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# 改为 20Hz
REPORT_HZ=20 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# 直接指定 scan_time=1（约100Hz）
REPORT_SCAN_TIME=1 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init
```

### 参数组合速查（重点）

```bash
# 1) 不加任何变量：按默认映射，前腿发 can0、后腿发 can1
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh ping

# 2) 只测后腿总线（推荐 can0 未接时）
BUS=rear bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh ping

# 3) 只测前腿总线
BUS=front bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh ping

# 4) 单电机（ID13）且强制走 can1（台架常用）
IFACE=can1 MOTOR_IDS="13" bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh ping

# 5) 自定义多电机（例如后左腿 51 52 53）
IFACE=can1 MOTOR_IDS="51 52 53" bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh ping
```

说明：

- `MOTOR_IDS` 一旦设置，会覆盖 `BUS` 的 ID 列表。
- `IFACE` 一旦设置，会覆盖 `IFACE_FRONT/IFACE_REAR` 与 `BUS` 的接口选择逻辑。
- 常见安全用法：**先 `ping` 看路由，再执行 `zero/init/reset/mit`**。

### 常见命令示例（zero / init / reset / all）

```bash
# A. 全 12 电机设置零位（按默认前后腿分口）
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh zero

# B. 仅后腿初始化（使能 + 主动上报开），且只访问 can1
BUS=rear bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# C. 单电机初始化（ID13，走 can1）
IFACE=can1 MOTOR_IDS="13" bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# D. 单电机复位（上报关 + 失能）
IFACE=can1 MOTOR_IDS="13" bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh reset

# E. 前腿全流程：zero -> init
BUS=front bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh all

# F. 初始化时把上报频率设为 5Hz（默认就是 5Hz，写法用于显式说明）
REPORT_HZ=5 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init
```

### 12 电机通信快速验证（`check`）

用途：快速判断每个电机链路是否通（你提到的“发 set_zero，电机同步返回反馈”场景）。

逻辑：

1. 对每个目标电机发送 `set_zero`（CMD=6）
2. 在该电机对应总线上等待一帧反馈（CMD=2，带 motor_id 过滤）
3. 输出 `[OK]/[FAIL]` 与最终统计

示例：

```bash
# 全 12 电机检查
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh check

# 仅后腿 6 电机检查（只看 can1）
BUS=rear bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh check

# 单电机检查（ID13，强制 can1）
IFACE=can1 MOTOR_IDS="13" bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh check
```

判定建议：

- 全部 `[OK]`：链路基本正常
- 个别 `[FAIL]`：优先检查该电机电源/CANH-CANL/ID 拨码/总线映射
- 全部 `[FAIL]`：优先检查接口是否 `UP`、波特率、终端电阻、主站 ID

### 主站 ID 与上报 ID 示例（MASTER / MASTER_REPORT）

```bash
# 所有命令使用主站 0xFD（默认，可省略）
MASTER=253 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# 控制帧仍用 MASTER=253，但通信类型24上报帧改用主站 0
MASTER=253 MASTER_REPORT=0 \
  bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init
```

### 发送节奏示例（SLEEP_MS）

```bash
# 放慢发送节奏，降低并发时 ENOBUFS 概率
SLEEP_MS=40 bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh init

# 单电机可适当减小
IFACE=can1 MOTOR_IDS="13" SLEEP_MS=5 \
  bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh mit
```

### MIT 运控报文（`mit` 子命令）

脚本会按协议编码：

- **CAN ID**：`(CMD_CONTROL<<24) | (tau_u16<<8) | motor_id`，其中 `CMD_CONTROL=1`
- **Data[8]**：`p_u16 | v_u16 | kp_u16 | kd_u16`（均为大端）

默认参数（与你要求一致）：

- `MIT_P=0`（rad）
- `MIT_V=0`（rad/s）
- `MIT_KP=20`
- `MIT_KD=1.5`
- `MIT_TAU=0`（Nm，编码到 CAN ID bits 8–23）

对应协议裁剪范围（超界会自动 clamp）：

- `p ∈ [-12.57, 12.57]`
- `v ∈ [-50, 50]`
- `kp ∈ [0, 500]`
- `kd ∈ [0, 5]`
- `tau ∈ [-6, 6]`

示例：

```bash
# 全部 12 电机发送一轮 MIT（默认 p=0,v=0,kp=20,kd=1.5,tau=0）
bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh mit

# 仅后腿 can1 发一轮
BUS=rear bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh mit

# 单电机 ID13：目标位置 0.2rad，kp=10，其余默认
IFACE=can1 MOTOR_IDS="13" MIT_P=0.2 MIT_KP=10 \
  bash src/trotbot_can_bridge/scripts/el05_motor_cansend.sh mit
```

验证建议（另一终端）：

```bash
candump -tz can1
```

### 12 电机 MIT 200Hz 连续发送（显式脚本）

用于快速核对“主站实际发送频率”和“电机反馈频率”是否接近 200Hz。  
脚本路径：`src/trotbot_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh`

默认参数：

- `TARGET_HZ=200`
- `DURATION_S=5`（当 `LOOPS=0`）
- MIT 固定帧：`p=0, v=0, kp=20, kd=1.5, tau=0`（`data=800080000A3D4CCD`）
- 每个 loop 向 12 个电机各发 1 帧（共 12 帧）

示例：

```bash
# 默认跑 5 秒
bash src/trotbot_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh

# 固定时长 8 秒
DURATION_S=8 bash src/trotbot_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh

# 固定循环 1000 次（优先于 DURATION_S）
LOOPS=1000 bash src/trotbot_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh
```

频率观测建议（开两个终端）：

```bash
# 终端A：看发送帧时间戳（can0/can1 分别看）
candump -tz can0,01800000:1FFFFF00
candump -tz can1,01800000:1FFFFF00

# 终端B：看反馈帧时间戳（根据你的反馈 ID 过滤）
candump -tz can0
candump -tz can1
```

### 在线 MIT 联调（`/mit_gains_cmd`）

`motor_protocol_node` 现支持运行时在线调整 `kp/kd/tau_ff`，默认话题：

- `/mit_gains_cmd`（`std_msgs/msg/String`）

命令格式为**空格分隔** `key=value`：

- `scope=all|front|rear`（或 `can0|can1`）
- `kp=<float>`，范围 `[0, 500]`
- `kd=<float>`，范围 `[0, 5]`
- `tau=<float>`（或 `tau_ff`），范围 `[-6, 6]`
- `reset=1`：恢复到参数文件中的默认值（`kp/kd/default_tau_ff`）

示例：

```bash
# 全腿同时提刚度
ros2 topic pub --once /mit_gains_cmd std_msgs/msg/String "{data: 'scope=all kp=28 kd=1.8'}"

# 仅后腿增加前馈，补偿后躯塌陷
ros2 topic pub --once /mit_gains_cmd std_msgs/msg/String "{data: 'scope=rear tau=0.6'}"

# 仅前腿减小阻尼
ros2 topic pub --once /mit_gains_cmd std_msgs/msg/String "{data: 'scope=front kd=1.2'}"

# 一键恢复默认
ros2 topic pub --once /mit_gains_cmd std_msgs/msg/String "{data: 'reset=1'}"
```

节点会周期打印当前生效参数（`can0/can1` 分开）和命令-反馈误差统计，便于联调闭环。

### 启动平滑与发送限位（新增）

`control_gains.yaml` 新增了两类安全参数：

- 启动平滑：
  - `enable_startup_smoothing`
  - `startup_smoothing_duration_s`
  - `command_max_velocity_rad_s`
  - `limit_velocity_only_during_smoothing`
- 发送前限位（CHAMP 关节域）：
  - `use_joint_cmd_limits`
  - `joint_cmd_min_rad` / `joint_cmd_max_rad`（12 维，顺序同 `joint_signs`）

说明：

- 启动初期会优先参考电机反馈位置，按插值 + 速度限幅过渡到控制目标，减小趴姿启动突变。
- `limit_velocity_only_during_smoothing=true`（默认）时，速度限幅只在平滑窗口内生效；进入稳态步态后不再持续压缩目标幅度。
- 每条关节命令在发送前先做限位，再映射到 MIT 角度发送，避免越界冲击。
- 当前默认限位已按 `config/minidog position.jpg` 的“运动范围(URDF零位)”录入（hip/thigh/calf）。

电机域（MIT角）显式限位：

- `use_motor_domain_limits`
- `derive_motor_limits_from_joint_cmd`
- `joint_mit_min_rad` / `joint_mit_max_rad`

推荐用法：

- `derive_motor_limits_from_joint_cmd=true`：按 `θ_mit = sign*θ_champ + offset` 从 `joint_cmd_*` 自动推导电机域限位（与映射一致）。
- 若你有实测电机域机械极限表（如 EL05 台架标定），可设 `derive_motor_limits_from_joint_cmd=false`，并直接填写 `joint_mit_*` 强制硬限位。

关节偏移（`joint_offset_*`）场景说明：

- 场景A（装外壳，外壳底部接地）：建议使用 `hip≈0.17444, thigh≈1.0031, calf≈2.5469`
- 场景B（无外壳，底部垫高使髋关节水平）：建议使用 `hip=0.0, thigh≈1.0031, calf≈2.5469`

当前默认配置启用场景B；场景A仅保留为注释参考。

新增（使能沿 + 反馈阈值步进）：

- `enable_mode_rising_smoothing`
- `motor_enabled_mode_status`（EL05 规格书：`0=Reset, 1=Cali, 2=Motor`，默认 `2`）
- `enable_feedback_step_limit`
- `feedback_step_limit_rad`
- `feedback_fresh_timeout_s`

行为说明：

- 当反馈 `mode_status` 发生“失能/标定 -> 运行”上升沿时，会以当前反馈角为锚点重置平滑，避免使能瞬间冲击。
- 发送目标前会参考最新反馈角：若 `|target - feedback|` 超过阈值，只发送 `feedback ± feedback_step_limit_rad`，实现分步逼近。

### 推荐联调顺序（先走起来，再提质）

建议固定步态速度、步高和机体高度后，按下列顺序微调：

1. `kp`（先）  
   从保守值开始逐步上调，观察前进/后退是否仍“原地踏步”。如果跟踪过软，优先提高 `kp`。
2. `kd`（后）  
   当 `kp` 上来后出现抖动或过冲，再逐步增加 `kd` 抑制振荡。
3. `tau_ff`（最后）  
   在 `kp/kd` 基本稳定后，用 `tau_ff` 做重力补偿；建议优先对后腿（`scope=rear`）小步调整。

验收建议：

- 前后移动有明显位移，不再只是原地踏步。
- 不出现持续高频抖振或明显发热。
- 误差统计（`MIT tracking abs_err`）随调参趋于下降。

## 反馈 JointState 与 `enable_rx_decode_log`（motor_protocol_node）

- **`/joint_states_feedback`**：`SensorDataQoS`，CHAMP 关节名顺序；CAN 回调更新内部缓存，**`feedback_joint_states_timer_hz`**（默认 50，见 `control_gains.yaml`）定时发布，供 `robot_state_publisher_feedback` 与 RViz **`RobotModelFeedback`**。
- **`enable_rx_decode_log`**：**仅**控制 `/motor_feedback` 字符串与解码路径上的 INFO 节流；**不**关闭 RX 解析与反馈角缓存（见 `开发问题跟踪.md` ISSUE-0005）。

## 仍待增强的 Backlog（非阻塞当前实机基线）

- **`cmd_joint_states`**：与 `/joint_states` 并行的“纯指令态”可视化（双模型现以指令 `/joint_states` + 反馈 `/joint_states_feedback` 为主）。
- **结构化反馈话题**：速度/力矩/状态字统一消息（对应需求 RQ-012）。
- **`safety_manager_node`**：急停与超时门禁与 `power_sequence` 的统一服务化封装。

应看到 `id=0x01xxxx0D`（CMD=1，末字节为 motor_id），`data` 为 8 字节 MIT 控制量。

### 主动上报（通信类型 24）

默认对上报帧仍使用 **`MASTER=253`（0xFD）**，即扩展 ID 形如 **`0x1800FDxx`**。若你的手册要求主 CAN 为 **0**，可单独设置：

```bash
MASTER_REPORT=0 bash scripts/el05_motor_cansend.sh init
```

### 与其它文档的关系

仓库根目录 **`README.md`** 里「**2b) cansend：电机主动上报**」一节给出的示例 **`0x1800000D`** 对应 **主 CAN_ID=0**；本脚本默认跟随 **`protocol_motor.h` 的 0xFD**。若实机仅一种配置能通，以 **`candump`** 核对后再固定环境变量。

### 开机自动拉起 CAN（systemd）

> 适用于接口已存在但重启后处于 down 的场景。  
> 若 `ip -br link` 里**没有** `can0/can1`，请先修复设备树/overlay（见下方排查）。

1) 新建服务：

```bash
sudo tee /etc/systemd/system/can-up.service >/dev/null <<'EOF'
[Unit]
Description=Bring up SocketCAN interfaces at boot
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/bin/bash -lc 'for i in can0 can1; do ip link show "$i" >/dev/null 2>&1 || continue; ip link set "$i" down || true; ip link set "$i" type can bitrate 1000000 restart-ms 100; ip link set "$i" up; ip link set "$i" txqueuelen 1000 || true; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

其中在 `up` 之后设置 **`txqueuelen 1000`**，可减轻高频/并发发送时的 **`ENOBUFS`（No buffer space available）**；低负载下对时延影响很小。

2) 启用并立即执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now can-up.service
```

3) 验证：

```bash
systemctl status can-up.service --no-pager
ip -br link | grep -E '^can[01]\b'
ip -details link show can0
ip -details link show can1
```

4) 修改波特率（例如 500k）：

- 编辑服务中的 `bitrate 1000000` 为目标值后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl restart can-up.service
```

#### 排查：重启后看不到 can0/can1

若 `ip -br link` 完全没有 `can0/can1`，说明不是“没 up”，而是“接口没被创建设备”：

- 先核对 `/boot/firmware/ubuntuEnv.txt` 的 `fdtfile` / `overlays`
- 再核对系统里是否存在对应 CAN overlay 文件（不同发行版路径可能不同）
- 仅在接口已出现时，`can-up.service` 才能生效

#### 常见问题：`RobotModelFeedback` 提示无 TF（No transform from ...）

现象：

- RViz 中反馈模型 `RobotModelFeedback` 报错 `No transform from ...`
- TF 视图提示反馈帧与主帧不在同一棵树

原因：

- 反馈模型使用 `TF Prefix=fb`，其根帧为 `fb/base_link`；
- 若未发布 `base_link -> fb/base_link` 的桥接 TF，RViz 固定帧（通常是 `base_link`）无法变换到反馈树。

修复（已集成到 `trotbot_basic.launch.py`）：

- 启动时自动发布静态变换：`base_link -> fb/base_link`（单位变换）。

临时手工验证命令：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link fb/base_link
```

#### 调整 RViz 两套模型颜色（目标 vs 反馈）

本工程采用“双色双描述”：

- 目标模型：`/robot_description`
- 反馈模型：`/robot_description_feedback`

颜色不是在 RViz 面板里改，而是在 launch 里给 xacro 传参数。

文件：`src/trotbot/launch/trotbot_basic.launch.py`

- 目标模型参数：
  - `model_r:=0.88 model_g:=0.58 model_b:=0.28 model_a:=0.95`
- 反馈模型参数：
  - `model_r:=0.28 model_g:=0.72 model_b:=0.88 model_a:=0.55`

含义：

- `model_r/g/b`：颜色通道，范围 `[0,1]`
- `model_a`：透明度，`1.0` 不透明，越小越透明

修改步骤：

1. 编辑上述 launch 文件中的两组 `model_*` 参数
2. 重新构建并 source：

```bash
colcon build --packages-select trotbot --symlink-install
source install/setup.bash
```

3. 重新启动：

```bash
ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true rviz:=true
```
