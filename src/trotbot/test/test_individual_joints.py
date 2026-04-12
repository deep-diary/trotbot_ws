#!/usr/bin/env python3
"""
TrotBot Individual Joint Testing Script
Test individual servo joints with precise control and safety limits
"""

import sys
import time
import numpy as np
import math
import argparse

# Import TrotBot HAL modules
from hal.hardware.hardware_interface import HardwareInterface
from hal.hardware.config import ServoParams, PWMParams


class IndividualJointTester:
    """Test individual servo joints with precise control"""

    def __init__(self):
        """Initialize individual joint tester"""
        print("🔧 TrotBot Individual Joint Tester")
        print("=" * 40)

        # Initialize TrotBot hardware interface
        try:
            self.hardware_interface = HardwareInterface()
            print("✅ TrotBot hardware interface initialized")
        except Exception as e:
            print(f"❌ Failed to initialize TrotBot hardware interface: {e}")
            sys.exit(1)

        # Get configuration parameters
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()

        # Leg and joint names
        self.leg_names = ["Front-Right", "Front-Left", "Back-Right", "Back-Left"]
        self.joint_names = ["Hip", "Thigh", "Calf"]

        # Safe angle ranges (degrees)
        self.angle_ranges = {
            'hip': (-30, 30),      # Hip abduction range
            'thigh': (0, 90),      # Thigh range
            'calf': (-90, 0)       # Calf range
        }

        print(f"📍 Servo mapping: 4 legs × 3 joints = 12 servos")
        self.display_servo_mapping()

    def display_servo_mapping(self):
        """Display servo channel mapping"""
        print(f"\n📍 Servo Channel Mapping:")
        for leg_index in range(4):
            leg_name = self.leg_names[leg_index]
            hip_pin = self.pwm_params.pins[0, leg_index]
            thigh_pin = self.pwm_params.pins[1, leg_index]
            calf_pin = self.pwm_params.pins[2, leg_index]
            print(f"  {leg_name}: Hip={hip_pin}, Thigh={thigh_pin}, Calf={calf_pin}")

    def create_neutral_matrix(self):
        """Create neutral position matrix for all servos"""
        joint_angles = np.zeros((3, 4))
        
        # Set all joints to neutral positions
        for leg_index in range(4):
            joint_angles[0, leg_index] = math.radians(0)    # Hip: 0°
            joint_angles[1, leg_index] = math.radians(45)   # Thigh: 45°
            joint_angles[2, leg_index] = math.radians(-45)  # Calf: -45°
        
        return joint_angles

    def set_all_neutral(self):
        """Set all servos to neutral position"""
        try:
            print(f"\n🎯 Setting all servos to neutral position...")
            joint_angles = self.create_neutral_matrix()
            self.hardware_interface.set_actuator_postions(joint_angles)
            print(f"✅ All servos set to neutral")
            return True
        except Exception as e:
            print(f"❌ Failed to set neutral position: {e}")
            return False

    def validate_angle(self, joint_type, angle_deg):
        """Validate if angle is within safe range"""
        joint_type = joint_type.lower()
        if joint_type not in self.angle_ranges:
            return False
        
        min_angle, max_angle = self.angle_ranges[joint_type]
        return min_angle <= angle_deg <= max_angle

    def set_individual_joint(self, leg_index, joint_index, angle_deg):
        """Set individual joint to specific angle

        Parameters
        ----------
        leg_index : int
            Leg index (0=FR, 1=FL, 2=BR, 3=BL)
        joint_index : int
            Joint index (0=Hip, 1=Thigh, 2=Calf)
        angle_deg : float
            Target angle in degrees

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if not (0 <= leg_index <= 3):
            print(f"❌ Invalid leg index: {leg_index}. Must be 0-3")
            return False

        if not (0 <= joint_index <= 2):
            print(f"❌ Invalid joint index: {joint_index}. Must be 0-2")
            return False

        joint_type = self.joint_names[joint_index].lower()
        if not self.validate_angle(joint_type, angle_deg):
            min_angle, max_angle = self.angle_ranges[joint_type]
            print(f"❌ Angle {angle_deg}° outside safe range for {joint_type}: {min_angle}° to {max_angle}°")
            return False

        try:
            leg_name = self.leg_names[leg_index]
            joint_name = self.joint_names[joint_index]
            
            print(f"\n🎯 Setting {leg_name} {joint_name} to {angle_deg}°")

            # Create neutral matrix and modify only the target joint
            joint_angles = self.create_neutral_matrix()
            joint_angles[joint_index, leg_index] = math.radians(angle_deg)

            # Send command
            self.hardware_interface.set_actuator_postions(joint_angles)
            print(f"✅ {leg_name} {joint_name} set to {angle_deg}°")
            return True

        except Exception as e:
            print(f"❌ Failed to set joint: {e}")
            return False

    def test_joint_range(self, leg_index, joint_index, step_size=15, hold_time=2.0):
        """Test full range of motion for a specific joint

        Parameters
        ----------
        leg_index : int
            Leg index (0=FR, 1=FL, 2=BR, 3=BL)
        joint_index : int
            Joint index (0=Hip, 1=Thigh, 2=Calf)
        step_size : float
            Step size in degrees (default: 15°)
        hold_time : float
            Time to hold each position (default: 2.0s)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if not (0 <= leg_index <= 3) or not (0 <= joint_index <= 2):
            print(f"❌ Invalid indices: leg={leg_index}, joint={joint_index}")
            return False

        leg_name = self.leg_names[leg_index]
        joint_name = self.joint_names[joint_index]
        joint_type = joint_name.lower()
        min_angle, max_angle = self.angle_ranges[joint_type]

        print(f"\n🔄 Testing {leg_name} {joint_name} Range of Motion")
        print(f"=" * 50)
        print(f"Range: {min_angle}° to {max_angle}°, Step: {step_size}°, Hold: {hold_time}s")

        # Generate test angles
        test_angles = []
        current_angle = min_angle
        while current_angle <= max_angle:
            test_angles.append(current_angle)
            current_angle += step_size
        
        # Ensure max angle is included
        if test_angles[-1] != max_angle:
            test_angles.append(max_angle)

        print(f"Test sequence: {len(test_angles)} positions")

        try:
            for i, angle in enumerate(test_angles):
                print(f"\n--- Step {i+1}/{len(test_angles)}: {angle}° ---")
                
                if not self.set_individual_joint(leg_index, joint_index, angle):
                    print(f"❌ Failed at {angle}°")
                    return False
                
                print(f"⏱️ Holding {angle}° for {hold_time}s...")
                time.sleep(hold_time)

            print(f"\n✅ {leg_name} {joint_name} range test completed successfully")
            
            # Return to neutral
            print(f"🔄 Returning to neutral position...")
            self.set_all_neutral()
            return True

        except KeyboardInterrupt:
            print(f"\n⚠️ Range test interrupted by user")
            print(f"🔄 Returning to neutral position...")
            self.set_all_neutral()
            return False
        except Exception as e:
            print(f"\n❌ Range test failed: {e}")
            print(f"🔄 Returning to neutral position...")
            self.set_all_neutral()
            return False

    def interactive_mode(self):
        """Interactive mode for individual joint control"""
        print(f"\n🎮 Interactive Individual Joint Control Mode")
        print(f"=" * 50)
        print(f"Commands:")
        print(f"  set <leg> <joint> <angle>  - Set specific joint angle")
        print(f"    leg: 0=FR, 1=FL, 2=BR, 3=BL")
        print(f"    joint: 0=Hip, 1=Thigh, 2=Calf")
        print(f"    angle: degrees within safe range")
        print(f"  test <leg> <joint>         - Test full range of joint")
        print(f"  neutral                    - Set all joints to neutral")
        print(f"  ranges                     - Show safe angle ranges")
        print(f"  help                       - Show this help")
        print(f"  quit                       - Exit")
        print(f"=" * 50)

        while True:
            try:
                cmd = input(f"\nJoint> ").strip().lower()
                if not cmd:
                    continue

                parts = cmd.split()
                
                if parts[0] in ["quit", "exit", "q"]:
                    print(f"🔄 Returning to neutral before exit...")
                    self.set_all_neutral()
                    break

                elif parts[0] == "set" and len(parts) == 4:
                    try:
                        leg_idx = int(parts[1])
                        joint_idx = int(parts[2])
                        angle = float(parts[3])
                        self.set_individual_joint(leg_idx, joint_idx, angle)
                    except ValueError:
                        print(f"❌ Invalid parameters. Usage: set <leg> <joint> <angle>")

                elif parts[0] == "test" and len(parts) == 3:
                    try:
                        leg_idx = int(parts[1])
                        joint_idx = int(parts[2])
                        self.test_joint_range(leg_idx, joint_idx)
                    except ValueError:
                        print(f"❌ Invalid parameters. Usage: test <leg> <joint>")

                elif parts[0] == "neutral":
                    self.set_all_neutral()

                elif parts[0] == "ranges":
                    print(f"\n📐 Safe Angle Ranges:")
                    for joint_type, (min_ang, max_ang) in self.angle_ranges.items():
                        print(f"  {joint_type.capitalize()}: {min_ang}° to {max_ang}°")

                elif parts[0] == "help":
                    self.interactive_mode()
                    return

                else:
                    print(f"❌ Unknown command. Type 'help' for available commands.")

            except KeyboardInterrupt:
                print(f"\n🔄 Returning to neutral...")
                self.set_all_neutral()
                break
            except Exception as e:
                print(f"❌ Error: {e}")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="TrotBot Individual Joint Tester")
    parser.add_argument("--leg", "-l", type=int, choices=[0,1,2,3],
                       help="Test specific leg (0=FR, 1=FL, 2=BR, 3=BL)")
    parser.add_argument("--joint", "-j", type=int, choices=[0,1,2],
                       help="Test specific joint (0=Hip, 1=Thigh, 2=Calf)")
    parser.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive mode")

    args = parser.parse_args()

    # Create tester
    tester = IndividualJointTester()

    # Safety confirmation
    print(f"\n⚠️ SAFETY CONFIRMATION")
    print(f"This will test individual TrotBot servo joints.")
    print(f"Ensure robot is safely supported!")

    response = input(f"\nProceed? (y/n): ").strip().lower()
    if response != 'y':
        print(f"Test cancelled")
        sys.exit(0)

    try:
        if args.leg is not None and args.joint is not None:
            # Test specific joint
            tester.test_joint_range(args.leg, args.joint)
        else:
            # Interactive mode
            tester.interactive_mode()

    except KeyboardInterrupt:
        print(f"\n⚠️ Testing interrupted")
        tester.set_all_neutral()
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        tester.set_all_neutral()

    print(f"\n🏁 Individual joint testing completed")


if __name__ == "__main__":
    main()
