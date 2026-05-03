# RK3588（鲁班猫）WS2812 / GPIO3_C1 实测记录（RQ-023）

硬件冻结：**40PIN Pin16** ↔ SoC **`GPIO3_C1`** → WS2812 **DIN**，灯珠数默认 **16**。Rockchip 命名：控制器 **3**、端口 **C**、索引 **1**。

## 1. 板上探测 libgpiod（必须）

在目标机执行（示例）：

```bash
gpiodetect
gpioinfo gpiochip0 | head -80
# 逐 chip 查找 consumer 空闲且名称或偏移对应 GPIO3_C1 的 line
```

将实测结果填入 [`config/status_led_params.yaml`](../config/status_led_params.yaml)：

| 字段 | 填写 |
|------|------|
| `gpiochip` | 无 `/dev/gpiochipN` 前缀的名称，如 `gpiochip1` |
| `gpio_line_offset` | `gpioinfo` 中该行的 **Line** 偏移（整数） |

若 `gpioset`/`gpioinfo` 报 **`Device or resource busy`**：多为 DTS 已占用该脚，需按野火文档检查设备树或外设冲突。

## 2. WS2812 用户态发码

- **libgpiod 逐比特 `set_value`**：ioctl 过慢 → **发白**。**mmap** 发码时 RK3588 须遵循 **GPIO V2** 寄存器协议（**DDR 在 0x08**，DR 高低半区拆分写掩码），勿沿用老式 **V1** `*DR|=bit`。当前节点已按 **`gpio-rockchip.c`** 实现 V2 写；mmap 前 **`request_output` 一次后立刻 `release`**。
- 若灯色仍不对：微调 **`timing_*_ns`** / **`timing_reset_us`**；核对 **3.3V/5V 电平**与共地。

## 3. 镜像与内核记录（交付填写）

| 项目 | 实测值 |
|------|--------|
| 镜像/发行版名称 | |
| 内核版本 `uname -r` | |
| `gpiochip` + line | |
| 比特流是否稳定 | |

## 4. 参考

- [野火 · 鲁班猫 RK3588 GPIO](https://doc.embedfire.com/linux/rk3588/quick_start/zh/latest/quick_start/40pin/gpio/gpio.html)
