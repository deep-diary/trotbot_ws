# trotbot_status_led（RQ-023）

鲁班猫 RK3588 上通过 **libgpiod** 驱动 **WS2812** 圆环（默认 **16** 灯），订阅 `power_sequence_node` 发布的 **`/power_sequence/state`**、**`/power_sequence/gate_open`**，按 YAML 显示灯语。

## 依赖

- ROS 2 Humble：`rclcpp`、`std_msgs`、`yaml_cpp_vendor`
- 系统：**`libgpiod-dev`**（提供 `gpiod.h` 与 `libgpiod`）。若构建时未安装，CMake 会生成 **stub** 二进制（**不链 libgpiod**，**灯带绝不会亮**）。实机请先 **`sudo apt install -y libgpiod-dev`** 再 **`colcon build`**，并用 **`ldd …/status_led_node | grep gpiod`** 确认已链接。

## 构建

```bash
sudo apt install libgpiod-dev   # 实机 / 交叉编译环境（若仅用 rockchip_mmap 可不链 gpiod，仍建议安装以便可选 gpiod 后端）
cd /path/to/trotbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select trotbot_status_led --allow-overriding trotbot_status_led
source install/setup.bash
```

### 完整编译 + 实机灯语自测（可复制）

在 **工作空间根目录** 执行；**`rockchip_mmap` 需 `sudo` 才能 mmap `/dev/mem`**，否则灯带不刷新（见下节「常见故障」）。

```bash
cd ~/trotbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select trotbot_status_led --allow-overriding trotbot_status_led
source install/setup.bash

sudo -E env "PATH=$PATH" bash -lc '
  source /opt/ros/humble/setup.bash
  source ~/trotbot_ws/install/setup.bash
  export TROTBOT_WS=$HOME/trotbot_ws
  bash ~/trotbot_ws/install/trotbot_status_led/share/trotbot_status_led/scripts/test_status_led_manual.sh
'
```

若工作空间不在 `~/trotbot_ws`，把上述路径与 `TROTBOT_WS` 改成你的实际目录即可。

也可直接 **`sudo bash ~/trotbot_ws/install/.../scripts/test_status_led_manual.sh`**：脚本会沿目录向上查找 **`install/setup.bash`**（兼容「从 `install` 下运行」与「从 `src/.../scripts` 运行」）。若仍失败，请 **`export TROTBOT_WS=/你的/trotbot_ws`**。

## 后端：rockchip_mmap（默认）与 gpiod

- **`ws2812_backend: rockchip_mmap`（默认）**：通过 `/dev/mem` mmap GPIO bank。**RK3588 使用 Rockchip GPIO 控制器 V2**：**DR/DDR 寄存器布局与 V1 不同**，须按内核 **`gpio-rockchip.c`** 的 **掩码写单比特**（不可简单 `*DR |= bit`）。本实现已按 V2 实现。**启动时用 libgpiod 做一次 `request_output` 后立刻 `release`**（促使 Pad→GPIO，且不与 mmap 长期争用 DR）。**需 `sudo` 打开 `/dev/mem`**；并需能访问 **`/dev/gpiochip*`**。
- **`ws2812_backend: gpiod`**：走 libgpiod。因 **`gpiod_line_set_value` 多为 ioctl，单次耗时可达微秒级**，远大于 WS2812 位宽（约 1.25µs），常见症状为**各色灯语看起来都像全亮、发白**。若必须用 gpiod，仅适合验证接线；量产请用 **mmap** 或其它 DMA/SPI 方案。

参数 **`mmap_gpio_phys_base`**：RK3588 **GPIO3** 控制器一般为 **`0xFEC40000`**（与 `gpiochip3`/fec40000 对应），与 **`gpio_line_offset`**（Pin16=`GPIO3_C1`→ **17**）组合成掩码 `1<<17`。

## 参数与映射

- 默认参数：[`config/status_led_params.yaml`](config/status_led_params.yaml)（**务必**按板上 `gpioinfo` 填写 **`gpiochip`** 与 **`gpio_line_offset`**）。
- 灯语规则：[`config/status_led_map.yaml`](config/status_led_map.yaml)（状态 → RGB / 低速呼吸）。
- **`simulate_hardware: true`**：不访问 GPIO（订阅与映射仍运行），用于无硬件桌面调试。

## 启动

单独：

```bash
ros2 launch trotbot_status_led status_led.launch.py
```

整机（示例，`trotbot_basic` **默认不启用**，需显式打开）：

```bash
ros2 launch trotbot trotbot_basic.launch.py use_can_bridge:=true use_status_led:=true
```

覆盖参数文件：

```bash
ros2 launch trotbot_status_led status_led.launch.py params_file:=/path/to/my_params.yaml
```

## 降级语义

- **GPIO 打开失败**或 **stub 构建**：节点保持运行，**不打断**运动栈；仅 WARN 一次，无灯带输出。
- **绿灯（就绪）语义**：仅在 **完整 ROS 栈已启动**、**映射条件成立**且 **硬件驱动成功**时成立；若 systemd 未拉起状态灯节点或映射未进入「可遥控」状态，**不得**仅凭「无灯」推断整机状态。

## RK3588 / GPIO 实测

填写模板见 [`docs/RK3588_GPIO_WS2812_HAL.md`](docs/RK3588_GPIO_WS2812_HAL.md)（**GPIO3_C1**、**Pin16**、`gpiochip`+line、镜像记录）。

## systemd 示例（现场按需改路径与单元名）

```ini
[Unit]
Description=TrotBot WS2812 status LED (ROS 2)
After=network-online.target

[Service]
Type=simple
User=bot
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/bot/trotbot_ws/install/setup.bash && ros2 launch trotbot_status_led status_led.launch.py'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

将 `ExecStart` 与整机 bringup 对齐；通常灯带节点与 `trotbot_basic` 同会话启动时，用 **同一 install/setup** 与 **`use_status_led:=true`** 的顶层 launch 更合适。

## 验证

```bash
ros2 node list | grep status_led
ros2 topic echo /power_sequence/state --once
# 预期 Running + gate_open 时呈绿色全环（见 status_led_map.yaml）
```

## 常见故障

| 现象 | 排查 |
|------|------|
| **各色都像白 / 全亮** | 多为位时序被判成全 **1**。已默认 **AArch64 CNTVCT 忙等** + 可调 **`timing_*_ns`**；建议 **`sudo`** 并启用 **`mlock_all`** / **`realtime_sched_priority`**（见 `status_led_params.yaml`）。仍发白则继续**减小** `timing_t0h_ns`、`timing_t1h_ns` 或微调 `t0l`/`t1l`。 |
| **无 sudo 时似乎永远停在上一档灯语（例如一直是 Precheck）** | `rockchip_mmap` 打不开 **`/dev/mem`** → 节点**不发帧**；灯珠仍显示**上一次成功点亮会话的最后一帧**，不是话题没更新。请 **`sudo`** 运行节点，或改用 **`ws2812_backend:=gpiod`**（见日志 WARN/ERROR）。 |
| **第一颗灯高频率闪、且偏绿** | WS2812 协议为 **GRB**，线路上**第一字节是 G**；首 bit 时序或总线边沿略抖时，常先反映为**绿通道**闪。可试：① 设 **`timing_lead_in_reset_us`** 为 **500～800**（帧前长低，比 `pre_frame` 更稳）；② 略增 **`timing_reset_us`**；③ 将 **`refresh_hz`** 降到 **30～40**；④ 硬件上 DIN 串 **200～470Ω**、**3.3V/5V 电平**与**共地**可靠。若仍明显，需考虑 **SPI/MOSI 发码** 等更稳的物理层。 |
| **第一颗灯偶尔乱色 / 闪**（一般） | 当不用 `lead_in` 时，可增大 **`timing_pre_frame_idle_us`**；与上一行二选一或组合试验。 |
| `Device or resource busy` | DTS/其它驱动占用该 GPIO；见野火文档与 `docs/RK3588_GPIO_WS2812_HAL.md` |
| 灯色错乱 | 微调 `timing_*_ns` / `timing_reset_us`；检查电平与供电共地 |
| 构建提示 stub | 安装 `libgpiod-dev` 后 **`colcon build` 重编** |
