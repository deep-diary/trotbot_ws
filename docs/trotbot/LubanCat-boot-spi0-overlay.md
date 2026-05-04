# LubanCat：在 ubuntuEnv.txt 中启用 SPI0 M2（Pin19 MOSI）

适用于 **`/boot/firmware/ubuntuEnv.txt`**（鲁班猫 Ubuntu 镜像常见路径）。

## 修改内容

在 **`overlays=`** 一行末尾（空格分隔）**追加**：

```text
rk3588-lubancat-spi0-m2-overlay
```

示例（与当前 CAN/摄像头等并列时）：

```text
overlays=rk3588s-lubancat-dp0-in-vp1-overlay rk3588s-lubancat-4-hdmi0-overlay rk3588-lubancat-can0-m0-overlay rk3588-lubancat-can2-m0-overlay rk3588s-lubancat-4-cam0-imx415-1080p-overlay rk3588-lubancat-spi0-m2-overlay
```

## 操作命令（需 root）

```bash
sudo nano /boot/firmware/ubuntuEnv.txt
```

或在 **`overlays=` 行尚未包含该插件时** 一键追加（避免重复执行误加两遍）：

```bash
sudo perl -i -pe 'if (/^overlays=/ && !/rk3588-lubancat-spi0-m2-overlay/) { s/\s*$/ rk3588-lubancat-spi0-m2-overlay/ }' /boot/firmware/ubuntuEnv.txt
```

若你机器上 **`overlays=`** 与示例不一致，请用编辑器**手工**在行尾追加 **`rk3588-lubancat-spi0-m2-overlay`**。

保存后 **重启**，再检查：

```bash
ls -l /dev/spidev*
```

将 **`status_led_params.yaml`** 中 **`spidev_path`** 设为实际设备节点。

## 注意

- **`overlay_prefix`** 为 **`rk3588s`** 时，boot 脚本仍会按厂商规则查找 **`rk3588-lubancat-spi0-m2-overlay.dtbo`**（名称以 **`/boot/firmware/overlays`** 下文件为准）。
- SPI0 M2 与 **UART4 M2** 等同组引脚互斥；若启用 **`rk3588-lubancat-uart4-m2-overlay`**，勿与 SPI0 M2 同时占用同一脚。

## 故障：改完 `ubuntuEnv.txt` 后 HDMI/DP 不亮

常见原因（实机曾出现）：

1. **显示类 overlay 名称拼错**  
   DP/HDMI 官方插件名一般为 **`rk3588s-lubancat-…`**（中间带字母 **`s`**，对应 rk3588**s** 产品线）。若写成 **`rk3588-lubancat-dp0-in-vp1-overlay`**（少了 **`s`**），会加载**错误的 `.dtbo` 或找不到文件**，设备树与 VOP/显示子系统不匹配。  
   **dmesg 典型表现**（需 `sudo dmesg`）：
   - `rockchip-vop2 ... *ERROR* No crtc register, please check dts config`
   - `rockchip-drm display-subsystem: failed to bind fdd90000.vop ...: -19`  
   恢复为厂商文档中的 **`rk3588s-lubancat-dp0-in-vp1-overlay`** / **`rk3588s-lubancat-4-hdmi0-overlay`** 等与 **`ubuntuEnv.txt.bak.*`** 一致的写法后重启。

2. **overlay 叠加顺序 / 资源冲突**  
   摄像头等多 VP 占用插件与显示插件并存时，若顺序或与 SPI/CAN 冲突，也可能异常；优先保证 **DP/HDMI 名称正确**，再按需逐项追加 SPI、CAM。

3. **修改后必须重启**  
   `ubuntuEnv.txt` 只在下次内核加载 DTB 时生效。

## 安全提示

勿在聊天或文档中保存 root 密码；恢复完成后建议 **`passwd`** 更换。
