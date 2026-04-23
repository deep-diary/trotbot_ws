# champ_teleop
Champ Quadruped Robot's teleoperation node. This is a forked version of [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py). 

The software has been modified to control the robot's whole-body pose (roll, pitch, yaw).

## How to use

    roslaunch champ_teleop teleop.launch

optional paramters for Logitech f710:

    roslaunch champ_teleop teleop.launch joy:=true

* Make sure the joystick's switch on top is in 'x' mode.

## Controlling the robot with Joystick

（PS4 / 键位与 `axes` 下标因驱动而异，**以你机 `ros2 topic echo /joy` 为准**；中文总表见
`docs/PS4_JOY_MAPPING_zh.md`。）

Left stick:
- Y → `cmd_vel.linear.x`
- X → strafe or turn: with **L1** (index 4) strafe `linear.y`, else `angular.z`

Right stick:
- Y → body pitch, X → roll (R1 not held) or yaw (R1 held) per `champ_teleop.py`
- **R2** (commonly `axes[5] < 0` when pulled) → **continuous** body height down (``joy_height_axis5_scale``; default 0.5)
- **Face buttons** (default inc=3 / dec=1) → **discrete** height up/down (shared with keyboard t/b)

**R2 "crouch"** is **smooth** (analog), not a one-shot snap, as long as the driver reports a continuous trigger axis. 