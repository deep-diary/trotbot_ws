# LubanCat-4-V1 40PIN 引脚说明（结合 TrotBot 仓库）

本文说明 **LubanCat-4-V1** 板载 **40 针排针**各脚的复用功能，并标注在 **本仓库 / 当前系统方案**中是否、以及如何被使用。完整 **官方引脚图**见 **`docs/image/LubanCat-4-V1 pins.png`**（正文表格与附图一致，便于打印对照）。

**实机占用**另依据项目组 **手绘接线备忘**（红圈标注）：区分 **外部电源输入**、**CAN 收发器供电**、**双路 CAN 信号**、**WS2812 灯带** 四组；下文 **§1.1** 与表格「本项目备注」已与该接线对齐。

---

## 1. 与 TrotBot 相关的结论摘要

| 主题 | 说明 |
|------|------|
| **双 CAN（电机）** | 40PIN 上为 **SoC 原生 CAN 控制器信号**，板载 **不含 CAN 收发芯片**，需外接模块；总线侧 **`CAN_H/CAN_L`** 接电机配电。**MCU 侧 TX/RX** 在本机占用 **Pin 7/11（硬件 CAN0）**、**Pin 38/40（硬件 CAN2）**。启用依赖设备树 overlay（见 `src/trotbot/本地运行指南.md` **§11.7**）。 |
| **CAN0（前腿总线）** | **Pin 7 = CAN0_RX**，**Pin 11 = CAN0_TX** → Linux **`can0`**（前腿 6 电机，ID `11/12/13/21/22/23`）。 |
| **CAN2（后腿总线）** | **Pin 38 = CAN2_RX**，**Pin 40 = CAN2_TX**（手绘上曾标为 CAN1，实为 **SoC CAN2**）→ 启用 overlay 后接口名常为 **`can1`**（本机：`can1` ↔ `fea70000.can`）。后腿 6 电机 ID `51/52/53/61/62/63`。**务必用 `ip -details link show` 核对 `parentdev`**。 |
| **CAN 收发器电源（由开发板供电）** | **Pin 17 = 3.3V 输出**供 **CAN 收发芯片** VCC；**Pin 20 = GND** 作为该路供电回流（与手绘「Power Out」一致）。 |
| **外部电源输入（5V）** | **Pin 4 = 5V**、**Pin 6 = GND** 作为 **整机外部 5V 电源输入**（手绘「Power In」）。 |
| **WS2812 灯带** | **Pin 2 = 5V**、**Pin 14 = GND**；数据线 **DIN** 软件默认可接 **Pin19（`SPI0_MOSI_M2`）** 走 **SPI/spidev**，或接 **Pin16（`GPIO3_C1`）** 走 **GPIO/mmap**（见 **`trotbot_status_led`** 参数 **`ws2812_backend`**）。 |
| **Pin 1（3.3V）** | 手绘中为 **不使用**（打叉）；本机配电不从此脚取电，表中备注「未接」。 |
| **其它 40PIN** | 未在红圈内的脚：**当前狗子连线未使用**；手柄等仍可按惯例走 USB。扩展时请避开 MIPI 占用脚（§2）及已有配电。 |

---

### 1.1 实机接线分组（与手绘红圈一致）

| 分组 | 物理引脚 | 作用 |
|------|-----------|------|
| **外部 5V 输入** | **4**（5V）、**6**（GND） | 外部电源接入整机（Power In）。 |
| **CAN 收发器 3.3V 供电** | **17**（3.3V 出）、**20**（GND） | 由鲁班猫排针向 **CAN 收发芯片**供电（Power Out）。 |
| **CAN0** | **7**（RX）、**11**（TX） | 接前腿路径收发器 MCU 侧 → **`can0`**。 |
| **CAN2** | **38**（RX）、**40**（TX） | 接后腿路径收发器 MCU 侧 → 常为 **`can1`**（硬件 CAN2）。 |
| **WS2812** | **2**（5V）、**14**（GND）、**16**（DIN，GPIO）或 **19**（DIN，SPI MOSI） | 供电同左；**Pin16** 为 bitbang 方案；**Pin19** 为 SPI 发码方案（见仓库 **`trotbot_status_led/docs/SPI_WS2812_RK3588.md`**）。 |
| **未使用** | **1**（板上 3.3V） | 手绘标明不接。 |

---

## 2. 使用 MIPI 屏幕/摄像头时的注意（附图原文）

> **LubanCat-4-V1 的 3、5、27、28 号物理引脚**在 MIPI 屏幕或 MIPI 摄像头方案中作为 **I2C** 使用，并有外部 **3.3V 上拉**。若开启了对应 **MIPI 设备树插件**，这些脚 **只能作 I2C**，不能再当通用 GPIO 或其它复用功能使用。

接线或编写 DTS/overlay 前，先确认当前镜像是否启用了相关 MIPI overlay。

---

## 3. 40 引脚一览（功能 + 本项目备注）

下表 **「主要复用」** 摘自官方引脚图，一行 **奇数脚 + 偶数脚**。**GPIO 编号**为图中 Rockchip 常用数字标号（与 Linux `gpiochip` line 不是同一套数字，以板上 `gpioinfo` 为准）。

| Pin | 主要复用 / 属性 | GPIO 或 ID（图中） | TrotBot / 仓库备注 |
|:---:|------------------|-------------------|---------------------|
| **1** | **3.3V** | — | **本机未使用**（手绘打叉）；勿从此脚取电以免与约定冲突。 |
| **2** | **5V** | — | **实机：WS2812 灯带 5V**（与 Pin14 GND、Pin16 DIN 同组）。 |
| **3** | I2C5_SDA_M3 | GPIO1_B7，47 | 若启用 MIPI 相关插件，可能被 I2C 占用（见 §2）。当前狗子未用。 |
| **4** | **5V** | — | **实机：外部电源 5V 输入（Power In）**。 |
| **5** | I2C5_SCL_M3 | GPIO1_B6，46 | 若启用 MIPI 相关插件，可能被 I2C 占用（见 §2）。当前狗子未用。 |
| **6** | **GND** | — | **实机：外部电源输入地（Power In）**，与 Pin4 成对。 |
| **7** | **CAN0_RX_M0**，PWM1_M0，I2C2_SDA_M0 | GPIO0_C0，16 | **实机：CAN0 RX** → 前腿收发器；Linux **`can0`**。 |
| **8** | UART0_TX_M2 | GPIO4_A3，131 | 当前狗子未用。 |
| **9** | **GND** | — | 系统地。 |
| **10** | UART0_RX_M2 | GPIO4_A4，132 | 当前狗子未用。 |
| **11** | **CAN0_TX_M0**，PWM0_M0，I2C2_SCL_M0 | GPIO0_B7，15 | **实机：CAN0 TX** → 前腿收发器；**`can0`**。 |
| **12** | PWM14_M2，I2C8_SCL_M2 | GPIO1_D6，62 | 当前狗子未用。 |
| **13** | PWM3_IR_M3，PDM1_SDIO_M1 | GPIO1_A7，39 | 当前狗子未用。 |
| **14** | **GND** | — | **实机：WS2812 灯带地**，与 Pin2（5V）、Pin16（DIN）同组。 |
| **15** | PDM1_SDI1_M1 | GPIO1_B0，40 | 当前狗子未用。 |
| **16** | **GPIO3_C1**，UART7_RX_M1，SPI1_CLK_M1 | 113 | **可选用：WS2812 DIN**（**`ws2812_backend:=rockchip_mmap`** / GPIO）。 |
| **17** | **3.3V** | — | **实机：3.3V 输出供 CAN 收发芯片**（Power Out，与 Pin20 GND 成对）。 |
| **18** | UART9_RTSN_M2 | GPIO3_D2，122 | 当前狗子未用。 |
| **19** | UART4_RX_M2，**SPI0_MOSI_M2**，PDM1_SDI3_M1 | GPIO1_B2，42 | **可选用：WS2812 DIN**（**`ws2812_backend:=spidev`**）；需在设备树启用 **SPI0 + spidev**，见 **`trotbot_status_led/docs/SPI_WS2812_RK3588.md`**。 |
| **20** | **GND** | — | **实机：CAN 收发器供电地**（Power Out），与 Pin17 成对。 |
| **21** | SPI0_MISO_M2，PDM1_SDI2_M1 | GPIO1_B1，41 | 当前狗子未用。 |
| **22** | UART9_RX_M2 | GPIO3_D4，124 | 当前狗子未用。 |
| **23** | UART4_TX_M2，SPI0_CLK_M2，PDM1_CLK1_M1 | GPIO1_B3，43 | 当前狗子未用。 |
| **24** | SPI0_CS0_M2，UART7_RX_M2，PDM1_CLK0_M1 | GPIO1_B4，44 | 当前狗子未用。 |
| **25** | **GND** | — | 系统地。 |
| **26** | SPI0_CS1_M2，UART7_TX_M2 | GPIO1_B5，45 | 当前狗子未用。 |
| **27** | I2C6_SDA_M3 | GPIO4_B0，136 | 若启用 MIPI 相关插件，可能被 I2C 占用（见 §2）。当前狗子未用。 |
| **28** | I2C6_SCL_M3 | GPIO4_B1，137 | 若启用 MIPI 相关插件，可能被 I2C 占用（见 §2）。当前狗子未用。 |
| **29** | — | GPIO3_A6，102 | 当前狗子未用。 |
| **30** | **GND** | — | 系统地。 |
| **31** | I2C3_SCL_M1，SPI1_MOSI_M1 | GPIO3_B7，111 | 当前狗子未用。 |
| **32** | PWM15_IR_M3，I2C8_SDA_M2 | GPIO1_D7，63 | 当前狗子未用。 |
| **33** | PWM10_M2，UART9_CTSN_M2 | GPIO3_D3，123 | 当前狗子未用。 |
| **34** | **GND** | — | 系统地。 |
| **35** | PWM11_IR_M3，UART9_TX_M2 | GPIO3_D5，125 | 当前狗子未用。 |
| **36** | — | GPIO4_A0，128 | 当前狗子未用。 |
| **37** | I2C3_SDA_M1，UART7_TX_M1，SPI1_MISO_M1 | GPIO3_C0，112 | 当前狗子未用。 |
| **38** | **CAN2_RX_M0**，UART5_TX_M1 | GPIO3_C4，116 | **实机：CAN2 RX**（手绘亦标 RX2）→ 后腿收发器；Linux 常为 **`can1`**。 |
| **39** | **GND** | — | 系统地；本机红圈未占用作专用回流时可作常规地参考。 |
| **40** | **CAN2_TX_M0**，UART5_RX_M1 | GPIO3_C5，117 | **实机：CAN2 TX**（手绘亦标 TX2）→ 后腿收发器；常为 **`can1`**。 |

---

## 4. 电源与地线（TrotBot 实机约定）

与「所有 GND 等价」的电气常识并行，本机在 **40PIN 上约定多套电源/地用途**，与手绘图一致，查阅时请勿混用：

| 用途 | 引脚 | 说明 |
|------|------|------|
| **外部 5V 输入** | **4（5V）、6（GND）** | 整机外部电源从此接入（Power In）。 |
| **WS2812 供电** | **2（5V）、14（GND）** | 仅灯带该路的 5V 与回流地（与 Pin16 数据线同组）。 |
| **CAN 收发器供电（开发板输出）** | **17（3.3V）、20（GND）** | 鲁班猫向板上/模块上 **CAN 收发芯片**提供 3.3V 与地（Power Out）。 |
| **Pin 1（3.3V）** | **1** | **本机不接**（手绘打叉）；勿与 17 混淆。 |

其余 **9、25、30、34、39** 等为排针上其它 **GND**，可按整机地与原理图作参考地；**高速/大电流回路**仍以原理图与 PCB 布线为准。

仓库软件层仍只识别逻辑 **`can0`/`can1`** 接口名；**配电与走线**以本文与硬件 BOM 为准。

---

## 5. CAN 与代码层的对应关系（便于联调）

- **应用层**：`trotbot_can_bridge` 中 **`can0_name` / `can1_name`** 默认 **`can0`、`can1`**（见 `can_transport_node` 参数与 `bridge.yaml`）。
- **物理层**：**Pin 7/11 ↔ CAN0**，**Pin 38/40 ↔ CAN2（接口名可能为 `can1`）**；与 **EL05 12 电机 ID 分前后腿** 的约定见 **`docs/trotbot/架构说明.md`**、**`docs/trotbot/软件需求清单.md`**。

启用 overlay、配置 bitrate、以及 **`ip -details link show`** 核对控制器地址的步骤见 **`src/trotbot/本地运行指南.md` §11.7** 与 **`src/trotbot/本地运行指南.md` §11.7.1**。

---

## 6. 参考

- 引脚附图：`docs/image/LubanCat-4-V1 pins.png`
- 双 CAN 落地与 overlay：`src/trotbot/本地运行指南.md`（§11.7、§11.7.1）
- 架构与 `can0`/`can1` 电机分工：`docs/trotbot/架构说明.md`
- WS2812 需求与 Pin16 / GPIO3_C1：`docs/trotbot/软件需求清单.md`（RQ-023）
- GPIO 命名与 `libgpiod`：软件需求清单中引用的 [野火鲁班猫 RK3588 — GPIO 控制](https://doc.embedfire.com/linux/rk3588/quick_start/zh/latest/quick_start/40pin/gpio/gpio.html)
