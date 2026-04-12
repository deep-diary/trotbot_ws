#!/usr/bin/env python3
"""
TrotBot Joystick Test Script

This script helps test and verify joystick connectivity and button/axis mapping
for TrotBot gamepad control. It subscribes to the /joy topic and displays
real-time joystick input data.

Usage:
    1. Connect your Bluetooth gamepad
    2. Launch the joy node: ros2 run joy joy_node
    3. Run this script: python3 test_joystick.py
    4. Press buttons and move sticks to see the mapping

This helps verify:
- Joystick connectivity and device detection
- Button and axis mapping for your specific controller
- Deadzone and sensitivity settings
- Compatibility with TrotBot gamepad configuration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys

class JoystickTester(Node):
    def __init__(self):
        super().__init__('joystick_tester')
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info('🎮 TrotBot Joystick Tester Started')
        self.get_logger().info('📡 Listening for joystick input on /joy topic...')
        self.get_logger().info('🔧 Press Ctrl+C to exit')
        self.get_logger().info('')
        self.get_logger().info('Expected TrotBot Mapping:')
        self.get_logger().info('  Left Stick X (axis 0): Angular velocity (turning)')
        self.get_logger().info('  Left Stick Y (axis 1): Linear velocity (forward/back)')
        self.get_logger().info('  Right Stick X (axis 3): Body roll')
        self.get_logger().info('  Right Stick Y (axis 4): Body pitch')
        self.get_logger().info('  Y/Triangle Button (3): Deadman/safety button')
        self.get_logger().info('  L1/LB Button (4): Strafe mode')
        self.get_logger().info('  R1/RB Button (5): Pose control mode')
        self.get_logger().info('=' * 60)
        
        self.last_axes = None
        self.last_buttons = None

    def joy_callback(self, msg):
        """Process joystick input and display relevant information"""
        
        # Display axes (analog sticks and triggers)
        if self.last_axes != msg.axes:
            self.get_logger().info('📊 AXES:')
            for i, axis_value in enumerate(msg.axes):
                if abs(axis_value) > 0.1:  # Only show significant values
                    axis_name = self.get_axis_name(i)
                    self.get_logger().info(f'  Axis {i} ({axis_name}): {axis_value:.3f}')
            self.last_axes = msg.axes[:]
        
        # Display buttons
        if self.last_buttons != msg.buttons:
            pressed_buttons = []
            for i, button_value in enumerate(msg.buttons):
                if button_value == 1:
                    button_name = self.get_button_name(i)
                    pressed_buttons.append(f'{i} ({button_name})')
            
            if pressed_buttons:
                self.get_logger().info(f'🔘 BUTTONS: {", ".join(pressed_buttons)}')
            
            self.last_buttons = msg.buttons[:]
        
        # Show TrotBot-specific interpretations
        if len(msg.axes) >= 2 and len(msg.buttons) >= 4:
            self.show_trotbot_interpretation(msg)

    def get_axis_name(self, axis_index):
        """Get human-readable axis names"""
        axis_names = {
            0: 'Left Stick X (Angular)',
            1: 'Left Stick Y (Linear)',
            2: 'Left Trigger',
            3: 'Right Stick X (Roll)',
            4: 'Right Stick Y (Pitch)',
            5: 'Right Trigger (Height)',
            6: 'D-Pad X',
            7: 'D-Pad Y'
        }
        return axis_names.get(axis_index, f'Unknown Axis {axis_index}')

    def get_button_name(self, button_index):
        """Get human-readable button names"""
        button_names = {
            0: 'A/X/Cross',
            1: 'B/Circle',
            2: 'X/Square',
            3: 'Y/Triangle (DEADMAN)',
            4: 'L1/LB (Strafe)',
            5: 'R1/RB (Pose)',
            6: 'Back/Select',
            7: 'Start/Options',
            8: 'Home/PS',
            9: 'Left Stick Click',
            10: 'Right Stick Click'
        }
        return button_names.get(button_index, f'Unknown Button {button_index}')

    def show_trotbot_interpretation(self, msg):
        """Show how the joystick input would be interpreted by TrotBot"""
        
        # Check deadman button (button 3 - Y/Triangle)
        deadman_active = len(msg.buttons) > 3 and msg.buttons[3] == 1
        
        if deadman_active:
            # Calculate movement commands
            linear_x = msg.axes[1] if len(msg.axes) > 1 else 0.0
            angular_z = msg.axes[0] if len(msg.axes) > 0 else 0.0
            
            if abs(linear_x) > 0.1 or abs(angular_z) > 0.1:
                self.get_logger().info('🤖 TrotBot Command:')
                if abs(linear_x) > 0.1:
                    direction = 'FORWARD' if linear_x > 0 else 'BACKWARD'
                    self.get_logger().info(f'  Linear: {direction} ({linear_x:.3f})')
                if abs(angular_z) > 0.1:
                    direction = 'LEFT' if angular_z > 0 else 'RIGHT'
                    self.get_logger().info(f'  Angular: {direction} ({angular_z:.3f})')
        
        elif any(abs(axis) > 0.1 for axis in msg.axes[:2]):
            self.get_logger().info('⚠️  DEADMAN BUTTON NOT PRESSED - No movement!')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        joystick_tester = JoystickTester()
        rclpy.spin(joystick_tester)
    except KeyboardInterrupt:
        print('\n🛑 Joystick tester stopped by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
