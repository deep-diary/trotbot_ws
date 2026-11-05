# CAN 配置与报文调试

本文承接根目录 `README.md`，涵盖：`can0/can1` 设备树 overlay 修复、开机自动 `up`、1Mbps 手工配置、`cansend` 报文（主动上报 / 设零）与总线抓包。

## 1) 开机后先确认 CAN 接口

```bash
ip -br link
ip -details link show can0
ip -details link show can1
```

若看不到 `can0/can1`，先按下面的**稳定配置**修复（LubanCat-4-v1 已验证）：

```bash
sudo cp /boot/firmware/ubuntuEnv.txt /boot/firmware/ubuntuEnv.txt.bak.$(date +%Y%m%d-%H%M%S)

# 关键点1：主 DTB 保持 4-v1（保底，避免依赖手工生成的 display.dtb）
sudo sed -i 's#^fdtfile=.*#fdtfile=rk3588s-lubancat-4-v1.dtb#' /boot/firmware/ubuntuEnv.txt

# 关键点2：overlay_prefix 置空，避免把 rk3588-can overlay 错拼成 rk3588s 前缀
sudo sed -i 's#^overlay_prefix=.*#overlay_prefix=#' /boot/firmware/ubuntuEnv.txt

# 关键点3：使用运行时 overlays（显示 + CAN）
sudo sed -i 's#^overlays=.*#overlays=rk3588s-lubancat-dp0-in-vp1-overlay rk3588s-lubancat-4-hdmi0-overlay rk3588-lubancat-can0-m0-overlay rk3588-lubancat-can2-m0-overlay#' /boot/firmware/ubuntuEnv.txt

# 关键点4：重启前先确认 overlays 文件都存在
ls -1 /boot/firmware/dtbs/rockchip/overlay | rg -n '^(rk3588s-lubancat-dp0-in-vp1-overlay|rk3588s-lubancat-4-hdmi0-overlay|rk3588-lubancat-can0-m0-overlay|rk3588-lubancat-can2-m0-overlay)\.dtbo$'

sudo reboot
```

> 说明：`rk3588s` 板子里使用 `rk3588-lubancat-can*.dtbo` 属于官方固件的常见命名，不是异常。

### 1a) 开机自动拉起 CAN（推荐）

当 `can0/can1` 已经由设备树创建出来时，用 systemd 保证每次开机自动 `up`：

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

sudo systemctl daemon-reload
sudo systemctl enable --now can-up.service
systemctl status can-up.service --no-pager
```

`txqueuelen 1000` 用于加大发送队列深度，减轻高频下发或并发发送时的 **`write: No buffer space available`**；负载不高时行为与默认相近。

### 1b) 复盘结论（为什么会“有时好有时坏”）

- **根因 1：overlay 命名匹配问题**  
  `overlay_prefix` 与 CAN dtbo 真实前缀不一致时，`can0/can1` 根本不会创建。
- **根因 2：显示 DTB/overlay 组合不稳**  
  启动日志出现 `vop2 ... No crtc register` / `display-subsystem ... bind failed` 时，显示链路失败，常伴随其它 overlays 生效异常（包括 CAN）。
- **关键认知：`can-up.service` 只能把“已存在”的接口置为 UP**  
  不能创建不存在的 `can0/can1`；接口创建必须先靠设备树成功。

## 2) 配置 1Mbps 并快速自测 CAN 互发

```bash
sudo ip link set can0 down || true
sudo ip link set can1 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 up
sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
ip -details -statistics link show can0
ip -details -statistics link show can1
```

运行已有脚本（双向收发）：

```bash
bash src/trotbot/test/test_can
```

## 3) `cansend`：电机主动上报（EL05 通信类型 24）

依赖 **`can-utils`**（若未安装：`sudo apt-get install -y can-utils`）。请先按上一节把对应接口配置为 **1Mbps** 并已 **`up`**。

手册「**通信类型 24：电机主动上报帧**」要点：

- **扩展帧 29-bit CAN ID**：最高 5 位（bits 28–24）为 **`0x18`**；低字节（bits 7–0）为 **目标电机 CAN_ID**；中间字段含 **主 CAN_ID（bits 15–8）**，常见默认 **全 0**。
- **数据域 8 字节**：Byte0–Byte5 固定 **`01 02 03 04 05 06`**；Byte6 **`F_CMD`**：**`01`** 开启主动上报（说明书默认上报周期约 **10ms**），**`00`** 关闭；Byte7 填 **`00`**。

**电机 ID = 13（0x0D）、主 CAN_ID = 0** 时，扩展 ID 与手册示例一致为 **`0x1800000D`**。在同一终端可直接执行：

```bash
# 开启 ID13 主动上报（请把 can0 换成电机所在总线，如 can1）
cansend can0 1800000D#0102030405060100

# 关闭主动上报
cansend can0 1800000D#0102030405060000
```

### 主 CAN_ID 非 0 时

若你的 **主 CAN_ID ≠ 0**，不要用固定 `1800000D`，改用下面一行拼装 ID（`MASTER`、电机号按实机填写）：

```bash
MASTER=0          # 主 CAN_ID（对应手册 bits 15–8），默认 0
MID=13            # 目标电机 CAN_ID（十进制）
EXT_ID=$(printf '%08X' $((0x18000000 | (MASTER << 8) | MID)))
cansend can0 ${EXT_ID}#0102030405060100
```

## 4) 12 电机「设置零位」显式命令（前腿 can0，后腿 can1）

> 设置零位命令：`CMD=0x06`，主站默认 `0xFD`，数据区使用 `0100000000000000`。  
> 报文格式：`cansend <iface> 0600FD<电机IDHEX>#0100000000000000`

```bash
# 前腿（can0）：11 12 13 21 22 23
cansend can0 0600FD0B#0100000000000000   # ID11
cansend can0 0600FD0C#0100000000000000   # ID12
cansend can0 0600FD0D#0100000000000000   # ID13
cansend can0 0600FD15#0100000000000000   # ID21
cansend can0 0600FD16#0100000000000000   # ID22
cansend can0 0600FD17#0100000000000000   # ID23

# 后腿（can1）：51 52 53 61 62 63
cansend can1 0600FD33#0100000000000000   # ID51
cansend can1 0600FD34#0100000000000000   # ID52
cansend can1 0600FD35#0100000000000000   # ID53
cansend can1 0600FD3D#0100000000000000   # ID61
cansend can1 0600FD3E#0100000000000000   # ID62
cansend can1 0600FD3F#0100000000000000   # ID63
```

另一终端观察总线（任选接口）：

```bash
candump -tz can0
```

细节以仓库内 **`EL05使用说明书`** 为准；若与实机 ID 分层规则不一致，以最新版手册章节为准。批量脚本见 [`src/trotbot_can_bridge/README.md`](../../src/trotbot_can_bridge/README.md) 的 `el05_motor_cansend.sh` 一节。

## 5) 查看 CAN 报文

```bash
# 1) 总线原始报文（需 can-utils；-t 时间戳，-z 毫秒）
candump -tz can0          # 前腿
candump -tz can1          # 后腿
candump -tz can0 can1     # 两路同时

# 2) ROS 侧 15 字节打包帧（UInt8MultiArray，十进制数组，可读性一般）
ros2 topic echo /can_tx_frames     # MIT 下发
ros2 topic echo /can_rx_frames     # 电机反馈 / 主动上报

# 3) 单行十六进制（可读性最好；需 bridge.yaml 中 publish_hex_lines: true）
ros2 topic echo /can_frames_tx_line
ros2 topic echo /can_frames_rx_line
# 例：TX can1 EXT id=0x017FFF0D len=08 75 4B 7F FF 0F 5C 4C CC
```

- `publish_hex_lines` 默认关闭，修改 `src/trotbot_can_bridge/config/bridge.yaml` 后重启 `can_transport_node` 生效（字段说明见[《CAN桥架构与话题细节》](./CAN桥架构与话题细节.md)）。
- 主动上报帧为扩展帧、ID 形如 `0x18xxxxxx`；MIT 下发帧高字节为 `0x01`（如 `0x017FFF0D`，中间字节为力矩编码），电机应答反馈为 `0x02...`。直接用 `candump` 看时以实际打印为准。
