# Dog Motion Tuning Notes

## 步态采样点

默认一个正弦周期 20 点，可通过两种方式调整：

- 编译期：`config.h` -> `DEEP_DOG_GAIT_TOTAL_STEPS`
- 运行时：MCP `self.chassis.set_gait_steps`

点数越大，轨迹离散误差越小，但控制下发频率也会更高。

## 速度与步频

`self.chassis.set_speed` 同时调两项：

- `step_period_ms`：步频节拍
- `speed_x100`：MIT `v_des`（除以 100 得到 rad/s）

## 停止与站立保持

- `stand` 结束会补发一次 `v_des=0` 保持帧
- `stop` 结束会补发一次 `v_des=0` 保持帧

用于减少停稳阶段因速度分量导致的姿态偏移。

## MIT 增益

- 设置：`self.chassis.set_mit_gains`
- 查询：`self.chassis.get_mit_gains`

范围：

- `kp`: 0~500
- `kd`: 0~5
