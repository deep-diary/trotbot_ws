#!/usr/bin/env python3
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import math
import select
import sys
import termios
import threading
import tty

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

def quaternion_from_euler(roll, pitch, yaw):
    """
    roll/pitch/yaw (rad) → Hamilton 四元数，返回 [w, x, y, z]（geometry_msgs 为 x,y,z,w 需自行重排）。
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Teleop(Node):
    def __init__(self):
        super().__init__('champ_teleop')
		
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 1)

        self.declare_parameter("gait/swing_height", 0)
        self.declare_parameter("gait/nominal_height", 0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)
        
        self.swing_height = self.get_parameter("gait/swing_height").value
        self.nominal_height = self.get_parameter("gait/nominal_height").value

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value

        # 键盘 t/b、手柄 △/✕：共用 body_pose.position.z 微调（相对于 nominal_height 的增量，±limit）
        self.declare_parameter("keyboard_body_z_step", 0.004)
        self.declare_parameter("keyboard_body_z_limit", 0.5)
        # Face 按键编号随驱动而异；实测 CUH-ZCT2E/Linux：0=✕ 1=○ 2=△ 3=□ → 升高用 △(2)、降低用 ✕(0)
        self.declare_parameter("joy_height_inc_button", 2)
        self.declare_parameter("joy_height_dec_button", 0)
        # R2（常见为 axes[5]）：扣扳机时多为负值 → 叠加负的 z（趴低/蹲下），随扳机行程连续变化（平滑）。
        # scale=0 则仅用 △/✕ 步进；默认 0.5 恢复旧版「扳机趴低」手感，且仅用 axes[5]<0，避免静止 +1 抬高。
        self.declare_parameter("joy_height_axis5_scale", 0.5)
        self._kb_body_z = 0.0
        self._joy_height_inc_prev = False
        self._joy_height_dec_prev = False

        self.msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
U    I    O
J    K    L
M    <    >
t : up (+z, step from keyboard_body_z_step)
b : down (-z)
joy: △ up / ✕ down（PS4 CUH-ZCT2E 实测 indices 2 / 0，详见 docs/PS4_JOY_MAPPING_zh.md）
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
        """

        self.velocityBindings = {
                'i':(1,0,0,0),
                'o':(1,0,0,-1),
                'j':(0,0,0,1),
                'l':(0,0,0,-1),
                'u':(1,0,0,1),
                ',':(-1,0,0,0),
                '.':(-1,0,0,1),
                'm':(-1,0,0,-1),
                'O':(1,-1,0,0),
                'I':(1,0,0,0),
                'J':(0,1,0,0),
                'L':(0,-1,0,0),
                'U':(1,1,0,0),
                '<':(-1,0,0,0),
                '>':(-1,-1,0,0),
                'M':(-1,1,0,0),
                'v':(0,0,1,0),
                'n':(0,0,-1,0),
            }

        # 注：t/b 在 poll_keys 中专用于 body_pose.position.z（与手柄 axes[5] 同语义），此处不再绑定
        self.poseBindings = {
                'f':(-1,0,0,0),
                'h':(1,0,0,0),
                'r':(0,0,1,0),
                'y':(0,0,-1,0),
            }

        self.speedBindings={
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }

        # 主线程阻塞在 poll_keys()；必须用 executor 才能处理 /joy，否则 joy_callback 永远不会执行
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.poll_keys()

    def _publish_keyboard_body_pose(self):
        """键盘专用：仅更新 height（与 joy 中 body_pose.position.z 语义一致）；姿态为单位四元数。"""
        z_lim = float(self.get_parameter("keyboard_body_z_limit").value)
        self._kb_body_z = max(-z_lim, min(z_lim, self._kb_body_z))

        body_pose_lite = PoseLite()
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.z = self._kb_body_z
        body_pose_lite.roll = 0.0
        body_pose_lite.pitch = 0.0
        body_pose_lite.yaw = 0.0
        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.x = 0.0
        body_pose.position.y = 0.0
        body_pose.position.z = self._kb_body_z
        body_pose.orientation.w = 1.0
        body_pose.orientation.x = 0.0
        body_pose.orientation.y = 0.0
        body_pose.orientation.z = 0.0
        self.pose_publisher.publish(body_pose)

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose_lite = PoseLite()
        # champ_msgs/Pose 在 rclpy 中对 float 字段做严格校验，禁止写 int（如 0）
        body_pose_lite.x = 0.0
        body_pose_lite.y = 0.0
        body_pose_lite.z = 0.0
        body_pose_lite.roll = (not data.buttons[5]) * -data.axes[3] * 0.349066
        body_pose_lite.pitch = data.axes[4] * 0.174533
        body_pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332

        step = float(self.get_parameter("keyboard_body_z_step").value)
        z_lim = float(self.get_parameter("keyboard_body_z_limit").value)
        inc_i = int(self.get_parameter("joy_height_inc_button").value)
        dec_i = int(self.get_parameter("joy_height_dec_button").value)
        if inc_i >= 0 and len(data.buttons) > inc_i:
            inc_on = bool(data.buttons[inc_i])
            if inc_on and not self._joy_height_inc_prev:
                self._kb_body_z = min(z_lim, self._kb_body_z + step)
            self._joy_height_inc_prev = inc_on
        if dec_i >= 0 and len(data.buttons) > dec_i:
            dec_on = bool(data.buttons[dec_i])
            if dec_on and not self._joy_height_dec_prev:
                self._kb_body_z = max(-z_lim, self._kb_body_z - step)
            self._joy_height_dec_prev = dec_on

        axis5_scale = float(self.get_parameter("joy_height_axis5_scale").value)
        axis5_term = 0.0
        if axis5_scale != 0.0 and len(data.axes) > 5:
            # 旧版 champ：仅 axes[5]<0 时改 z（通常为扣 R2）；松开为 +1 附近时不作用
            if data.axes[5] < 0.0:
                axis5_term = axis5_scale * data.axes[5]

        self._kb_body_z = max(-z_lim, min(z_lim, self._kb_body_z))
        body_pose_lite.z = max(-z_lim, min(z_lim, self._kb_body_z + axis5_term))

        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.x = 0.0
        body_pose.position.y = 0.0
        body_pose.position.z = body_pose_lite.z

        quaternion = quaternion_from_euler(body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw)
        # quaternion_from_euler 返回 [w,x,y,z]；geometry_msgs/Quaternion 字段顺序为 x,y,z,w
        body_pose.orientation.w = quaternion[0]
        body_pose.orientation.x = quaternion[1]
        body_pose.orientation.y = quaternion[2]
        body_pose.orientation.z = quaternion[3]

        self.pose_publisher.publish(body_pose)

    def poll_keys(self):
        self.settings = termios.tcgetattr(sys.stdin)

        x = 0
        y = 0
        z = 0
        th = 0
        roll = 0
        pitch = 0
        yaw = 0
        status = 0
        cmd_attempts = 0

        try:
            print(self.msg)
            print(self.vels( self.speed, self.turn))

            while rclpy.ok():

                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]
                    
                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x *self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist)

                    cmd_attempts += 1
                    
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15

                elif key == 't' or key == 'b':
                    cmd_attempts = 0
                    step = float(self.get_parameter("keyboard_body_z_step").value)
                    z_lim = float(self.get_parameter("keyboard_body_z_limit").value)
                    if key == 't':
                        self._kb_body_z += step
                    else:
                        self._kb_body_z -= step
                    self._kb_body_z = max(-z_lim, min(z_lim, self._kb_body_z))
                    self._publish_keyboard_body_pose()
                    print("body_pose z (keyboard) = %.4f m (limit ±%.2f)" % (self._kb_body_z, z_lim))

                else:
                    cmd_attempts = 0
                    if (key == '\x03'):
                        break

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            self._executor.shutdown()
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=2.0)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

if __name__ == "__main__":
    rclpy.init()
    teleop = Teleop()
