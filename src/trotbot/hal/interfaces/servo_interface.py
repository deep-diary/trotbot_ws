#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 TrotBot Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# TrotBot Servo Interface - Hardware Abstraction Layer

import numpy as np
import rclpy
import time
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from hal.hardware.hardware_interface import HardwareInterface


class ServoInterface(Node):
    def __init__(self):
        super().__init__('servo_interface')
        self.subscriber = self.create_subscription(
            JointTrajectory, 'joint_group_effort_controller/joint_trajectory',
            self.cmd_callback, 1)
        self.hardware_interface = HardwareInterface()

        # Joint angle limits in degrees (converted to radians for comparison)
        # Hip joints: -30° to +30° (abduction/adduction range) - PHYSICAL HARDWARE LIMIT
        # Thigh joints: 0° to +90° (forward swing range)
        # Calf joints: -90° to 0° (backward swing range)
        # Minidog 真机：URDF 与 minidog_champ.urdf.xacro 的关节零位/限位不同，需按实物重调
        # joint_limits_deg，或改为按 JointTrajectory.joint_names 映射后再下发舵机。
        self.joint_limits_deg = {
            'hip': (-30.0, 30.0),
            'thigh': (0.0, 90.0),
            'calf': (-90.0, 0.0)
        }

        # Convert to radians for efficient comparison
        self.joint_limits_rad = {
            'hip': (math.radians(-30.0), math.radians(30.0)),
            'thigh': (math.radians(0.0), math.radians(90.0)),
            'calf': (math.radians(-90.0), math.radians(0.0))
        }

        # Joint type names for logging
        self.joint_names = ["Hip", "Thigh", "Calf"]
        self.leg_names = ["Front-Right", "Front-Left", "Back-Right", "Back-Left"]

        self.get_logger().info("TrotBot Servo Interface: Ready to receive joint commands")
        self.get_logger().info("Joint limits enabled - Hip: ±30°, Thigh: 0°-90°, Calf: -90°-0°")

    def clamp_joint_angle(self, angle_rad, joint_type, leg_name, joint_name):
        """
        Clamp joint angle to safe limits and log warnings when clamping occurs

        Parameters
        ----------
        angle_rad : float
            Joint angle in radians
        joint_type : str
            Joint type ('hip', 'thigh', 'calf')
        leg_name : str
            Leg name for logging
        joint_name : str
            Joint name for logging

        Returns
        -------
        float
            Clamped angle in radians
        """
        # Get limits for this joint type
        min_rad, max_rad = self.joint_limits_rad[joint_type]

        # Check if clamping is needed
        if angle_rad < min_rad:
            angle_deg = math.degrees(angle_rad)
            min_deg = math.degrees(min_rad)
            # Only log hip joint limit violations to avoid spam (most critical for debugging)
            if joint_type == 'hip':
                self.get_logger().warn(f"⚠️ JOINT LIMIT: {leg_name} {joint_name} angle {angle_deg:.1f}° below limit {min_deg:.1f}° - clamping to minimum")
            return min_rad
        elif angle_rad > max_rad:
            angle_deg = math.degrees(angle_rad)
            max_deg = math.degrees(max_rad)
            # Only log hip joint limit violations to avoid spam (most critical for debugging)
            if joint_type == 'hip':
                self.get_logger().warn(f"⚠️ JOINT LIMIT: {leg_name} {joint_name} angle {angle_deg:.1f}° above limit {max_deg:.1f}° - clamping to maximum")
            return max_rad
        else:
            # Angle is within limits
            return angle_rad

    def cmd_callback(self, msg):
        joint_positions = msg.points[0].positions

        # Extract individual joint positions
        lf1_position = joint_positions[0]  # Left Front Hip
        lf2_position = joint_positions[1]  # Left Front Thigh
        lf3_position = joint_positions[2]  # Left Front Calf
        rf1_position = joint_positions[3]  # Right Front Hip
        rf2_position = joint_positions[4]  # Right Front Thigh
        rf3_position = joint_positions[5]  # Right Front Calf
        lb1_position = joint_positions[6]  # Left Back Hip
        lb2_position = joint_positions[7]  # Left Back Thigh
        lb3_position = joint_positions[8]  # Left Back Calf
        rb1_position = joint_positions[9]  # Right Back Hip
        rb2_position = joint_positions[10] # Right Back Thigh
        rb3_position = joint_positions[11] # Right Back Calf

        # Apply joint limits with clamping
        # Hip joints (abduction/adduction): -30° to +30°
        rf1_position = self.clamp_joint_angle(rf1_position, 'hip', self.leg_names[0], self.joint_names[0])
        lf1_position = self.clamp_joint_angle(lf1_position, 'hip', self.leg_names[1], self.joint_names[0])
        rb1_position = self.clamp_joint_angle(rb1_position, 'hip', self.leg_names[2], self.joint_names[0])
        lb1_position = self.clamp_joint_angle(lb1_position, 'hip', self.leg_names[3], self.joint_names[0])

        # Thigh joints (forward swing): 0° to +90°
        rf2_position = self.clamp_joint_angle(rf2_position, 'thigh', self.leg_names[0], self.joint_names[1])
        lf2_position = self.clamp_joint_angle(lf2_position, 'thigh', self.leg_names[1], self.joint_names[1])
        rb2_position = self.clamp_joint_angle(rb2_position, 'thigh', self.leg_names[2], self.joint_names[1])
        lb2_position = self.clamp_joint_angle(lb2_position, 'thigh', self.leg_names[3], self.joint_names[1])

        # Calf joints (backward swing): -90° to 0°
        rf3_position = self.clamp_joint_angle(rf3_position, 'calf', self.leg_names[0], self.joint_names[2])
        lf3_position = self.clamp_joint_angle(lf3_position, 'calf', self.leg_names[1], self.joint_names[2])
        rb3_position = self.clamp_joint_angle(rb3_position, 'calf', self.leg_names[2], self.joint_names[2])
        lb3_position = self.clamp_joint_angle(lb3_position, 'calf', self.leg_names[3], self.joint_names[2])

        # Create joint angles matrix with clamped values
        joint_angles = np.array([
            [rf1_position, lf1_position, rb1_position, lb1_position],  # Hip joints
            [rf2_position, lf2_position, rb2_position, lb2_position],  # Thigh joints
            [rf2_position + rf3_position, lf2_position + lf3_position, # Calf joints (combined with thigh)
             rb2_position + rb3_position, lb2_position + lb3_position]
        ])

        # Send clamped commands to hardware
        self.hardware_interface.set_actuator_postions(joint_angles)




def main(args=None):
    rclpy.init(args=args)
    servo_interface_node = ServoInterface()
    rclpy.spin(servo_interface_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
