# PS4（DualShock 4 / CUH-ZCT2E）与 `champ_teleop.py` 映射说明

`sensor_msgs/Joy` **不按「物理印刷」排版**，按 **内核 evdev→joy** 的顺序编号；同一手柄在不同驱动（蓝牙/USB、`hid-playstation`、`xpad`）下 **`buttons[]` / `axes[]` 可能不同**。请以本机实测为准：

```bash
ros2 topic echo /joy
```

按下 **单个键/摇杆** 看哪一项从 0 变为 1，或哪一轴数值变化。

**CUH-ZCT2E（本仓库默认高度键）实测 face 按键顺序：**  
`buttons[0]` **✕（叉）**，`[1]` **○（圆）**，`[2]` **△（三角）**，`[3]` **□（正方）**。  
据此默认 **`joy_height_inc_button=2`（△ 升高）、`joy_height_dec_button=0`（✕ 降低）**。

---

## 一、`axes[N]` 是什么？

**不是「第 N 个摇杆」**，而是 **`Joy` 消息里第 N 个模拟轴**（通常为浮点 −1〜1）。PS4 常见绑定（你的环境可能略有出入）：

| `axes` 索引 | 常见含义（PS4） |
|:-------------:|-------------------|
| `axes[0]` | 左摇杆左右 |
| `axes[1]` | 左摇杆前后（代码里用作前后线速度） |
| `axes[2]` | 常为 **L2** 模拟量（松开约 +1，按下变负） |
| `axes[3]` | 右摇杆左右 |
| `axes[4]` | 右摇杆前后（pitch） |
| `axes[5]` | 常为 **R2** 模拟量（同上） |

代码里 **`axes[5]`** 用于：**仅当 `axes[5] < 0`** 时叠加机体高度偏移（等价于扣 **R2**「趴低」），数值连续变化 ⇒ **平滑**，不是离散「瞬间趴下」。参数 **`joy_height_axis5_scale`**（默认 0.5）为该通道增益；设为 **0** 则只用 △ / ✕ 离散步进。

---

## 二、`buttons[M]` — 脚本里用到的

| `buttons` 索引 | `champ_teleop.py` 中的用途 |
|:----------------:|----------------------------|
| **`buttons[4]`** | 脚本里当作 **肩键**：按住时 **左摇杆左右 → 横向平移** `cmd_vel.linear.y`；不按 → **原地转向** `angular.z`。（原 CHAMP/Xbox：**LB**，PS4 上多为 **L1**。） |
| **`buttons[5]`** | 按住时 **右摇杆左右 → yaw**；不按 → **roll**。 （原：**RB**，PS4 多为 **R1**。） |

高度 **△ / ✕** 由参数 **`joy_height_inc_button`** / **`joy_height_dec_button`** 指定索引，仓库默认 **`2 / 0`**（对应上表 CUH-ZCT2E）。

---

## 三、脚本里尚未绑定的按键 / 轴

下列在 **`champ_teleop` 代码中未读取**，`echo /joy` 会变但 **不影响该节点**：

- **Share**（`power_sequence_node` 用作下电组合键之一）
- **Options**（整机上由 **`power_sequence_node`** 用作维修 **`set_zero`** 长按触发；参数 **`button_option`**，勿与本表 Face 键默认混淆）
- **十字键**（多数驱动映射为 **`axes`** 额外项或独立 `buttons`，本脚本未用）
- **触控板按下**（若有额外 `buttons`）
- **`poseBindings`**（`f/h/r/y`）仅在早期键盘表里定义，**未接入 `poll_keys`**，等价未实现

---

## 四、若与你机顺序不一致时如何改

1. `ros2 topic echo /joy`，分别按住 **△**、**✕**，记下 **`buttons` 里为 1 的下标**。
2. 设置（CUH-ZCT2E 已内置默认）：

```bash
ros2 param set /champ_teleop joy_height_inc_button 2
ros2 param set /champ_teleop joy_height_dec_button 0
```

仓库参考参数：`trotbot/config/champ_teleop_ps4_cuhzct2e.defaults.yaml`（节点名 `champ_teleop`）。
