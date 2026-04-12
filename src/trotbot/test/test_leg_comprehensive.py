#!/usr/bin/env python3
"""
Comprehensive TrotBot Leg Testing System
Tests individual legs and coordinated multi-leg movements using TrotBot HAL
Supports all four legs with proper inverse kinematics and servo control
"""

import sys
import time
import numpy as np
import math
import argparse

# Import TrotBot HAL modules
from hal.hardware.hardware_interface import HardwareInterface
from hal.hardware.config import ServoParams, PWMParams


class TrotBotLegKinematics:
    """Inverse kinematics for TrotBot quadruped"""

    def __init__(self):
        """Initialize with TrotBot leg geometry"""
        # TrotBot leg geometry (similar to Mini Pupper but using TrotBot config)
        self.LEG_L1 = 0.0735  # Thigh length (m) - 73.5mm
        self.LEG_L2 = 0.060   # Calf length (m) - 60mm
        self.ABDUCTION_OFFSET = 0.03675  # Hip offset (m) - 36.75mm

        # Leg indexing: 0=Front-Right, 1=Front-Left, 2=Back-Right, 3=Back-Left
        self.leg_names = ["Front-Right", "Front-Left", "Back-Right", "Back-Left"]

        # Abduction offset signs for each leg (right legs negative, left legs positive)
        self.abduction_offsets = [-self.ABDUCTION_OFFSET, self.ABDUCTION_OFFSET,
                                 -self.ABDUCTION_OFFSET, self.ABDUCTION_OFFSET]

        # Working ranges from calibration (degrees) - same for all legs
        self.hip_range = (-30, 30)    # -30° to 30° (abduction limits)
        self.thigh_range = (0, 90)    # 0° to 90°
        self.calf_range = (-90, 0)    # -90° to 0°

        print(f"🔧 TrotBot Leg Kinematics initialized:")
        print(f"   Thigh length (L1): {self.LEG_L1*1000:.1f}mm")
        print(f"   Calf length (L2): {self.LEG_L2*1000:.1f}mm")
        print(f"   Hip offset: {self.ABDUCTION_OFFSET*1000:.1f}mm")
        print(f"   Total reach: {(self.LEG_L1 + self.LEG_L2)*1000:.1f}mm")

    def get_leg_name(self, leg_index):
        """Get human-readable leg name"""
        return self.leg_names[leg_index] if 0 <= leg_index < 4 else f"Invalid-{leg_index}"

    def leg_inverse_kinematics(self, foot_position, leg_index):
        """Calculate joint angles for given foot position and leg

        Parameters
        ----------
        foot_position : array_like
            Foot position [x, y, z] relative to hip joint (meters)
        leg_index : int
            Leg index (0=Front-Right, 1=Front-Left, 2=Back-Right, 3=Back-Left)

        Returns
        -------
        tuple
            (hip_angle, thigh_angle, calf_angle) in radians, or None if unreachable
        """
        x, y, z = foot_position
        abduction_offset = self.abduction_offsets[leg_index]

        # Distance from the leg origin to the foot, projected into the y-z plane
        R_body_foot_yz = math.sqrt(y**2 + z**2)

        # Check if foot position is reachable (must be outside hip offset)
        if R_body_foot_yz < abs(abduction_offset):
            return None

        # Distance from the leg's forward/back point of rotation to the foot
        R_hip_foot_yz = math.sqrt(R_body_foot_yz**2 - abduction_offset**2)

        # Interior angle of the right triangle formed in the y-z plane
        arccos_argument = abduction_offset / R_body_foot_yz
        arccos_argument = max(-0.99, min(0.99, arccos_argument))  # Clamp to valid range
        phi = math.acos(arccos_argument)

        # Angle of the y-z projection of the hip-to-foot vector
        hip_foot_angle = math.atan2(z, y)

        # Hip/abduction angle
        hip_angle = phi + hip_foot_angle

        # Angle between the tilted negative z-axis and the hip-to-foot vector
        theta = math.atan2(-x, R_hip_foot_yz)

        # Distance between the hip and foot
        R_hip_foot = math.sqrt(R_hip_foot_yz**2 + x**2)

        # Check if target is reachable by leg links
        max_reach = self.LEG_L1 + self.LEG_L2  # 73.5mm + 60mm = 133.5mm
        min_reach = abs(self.LEG_L1 - self.LEG_L2)  # |73.5mm - 60mm| = 13.5mm
        if R_hip_foot > max_reach or R_hip_foot < min_reach:
            return None

        # Angle between the line going from hip to foot and the thigh link L1
        arccos_argument = (self.LEG_L1**2 + R_hip_foot**2 - self.LEG_L2**2) / (2 * self.LEG_L1 * R_hip_foot)
        arccos_argument = max(-0.99, min(0.99, arccos_argument))  # Clamp to valid range
        trident = math.acos(arccos_argument)

        # Thigh angle relative to the tilted negative z axis
        thigh_angle = theta + trident

        # Angle between the leg links L1 and L2
        arccos_argument = (self.LEG_L1**2 + self.LEG_L2**2 - R_hip_foot**2) / (2 * self.LEG_L1 * self.LEG_L2)
        arccos_argument = max(-0.99, min(0.99, arccos_argument))  # Clamp to valid range
        beta = math.acos(arccos_argument)

        # Calf angle relative to the tilted negative z axis
        calf_angle = thigh_angle - (math.pi - beta)

        return (hip_angle, thigh_angle, calf_angle)

    def check_joint_limits(self, joint_angles_rad, leg_index):
        """Check if joint angles are within safe working ranges

        Parameters
        ----------
        joint_angles_rad : tuple
            (hip_angle, thigh_angle, calf_angle) in radians
        leg_index : int
            Leg index for identification in error messages

        Returns
        -------
        bool
            True if all angles are within safe limits
        """
        if joint_angles_rad is None:
            return False

        hip_angle, thigh_angle, calf_angle = joint_angles_rad

        # Convert to degrees for limit checking
        hip_deg = math.degrees(hip_angle)
        thigh_deg = math.degrees(thigh_angle)
        calf_deg = math.degrees(calf_angle)

        # Check hip limits (-30° to 30°) - abduction safety
        if not (self.hip_range[0] <= hip_deg <= self.hip_range[1]):
            return False

        # Check thigh limits (0° to 90°) - same for all legs
        if not (self.thigh_range[0] <= thigh_deg <= self.thigh_range[1]):
            return False

        # Check calf limits (-90° to 0°) - same for all legs
        if not (self.calf_range[0] <= calf_deg <= self.calf_range[1]):
            return False

        return True

    def validate_workspace_detailed(self, foot_position, leg_index):
        """Detailed workspace validation with diagnostic information

        Parameters
        ----------
        foot_position : array_like
            Foot position [x, y, z] relative to hip joint (meters)
        leg_index : int
            Leg index for identification

        Returns
        -------
        tuple
            (is_valid, diagnostic_info)
        """
        x, y, z = foot_position
        leg_name = self.get_leg_name(leg_index)

        # Calculate distance from hip
        distance_from_hip = math.sqrt(x**2 + y**2 + z**2)
        max_reach = self.LEG_L1 + self.LEG_L2  # 133.5mm
        min_reach = abs(self.LEG_L1 - self.LEG_L2)  # 13.5mm

        diagnostic = {
            'leg_name': leg_name,
            'position': f"x={x*1000:.1f}mm, y={y*1000:.1f}mm, z={z*1000:.1f}mm",
            'distance_from_hip': f"{distance_from_hip*1000:.1f}mm",
            'max_reach': f"{max_reach*1000:.1f}mm",
            'min_reach': f"{min_reach*1000:.1f}mm",
            'within_reach': min_reach <= distance_from_hip <= max_reach
        }

        # Try inverse kinematics
        joint_angles = self.foot_position_to_joint_angles(foot_position, leg_index)
        diagnostic['ik_successful'] = joint_angles is not None

        if joint_angles:
            hip_deg, thigh_deg, calf_deg = joint_angles
            diagnostic['joint_angles'] = f"Hip={hip_deg:.1f}°, Thigh={thigh_deg:.1f}°, Calf={calf_deg:.1f}°"
            diagnostic['hip_in_range'] = self.hip_range[0] <= hip_deg <= self.hip_range[1]
            diagnostic['thigh_in_range'] = self.thigh_range[0] <= thigh_deg <= self.thigh_range[1]
            diagnostic['calf_in_range'] = self.calf_range[0] <= calf_deg <= self.calf_range[1]

        is_valid = diagnostic['within_reach'] and diagnostic['ik_successful']
        return is_valid, diagnostic

    def foot_position_to_joint_angles(self, foot_position, leg_index):
        """Convert foot position to joint angles with safety checks

        Parameters
        ----------
        foot_position : array_like
            Foot position [x, y, z] relative to hip joint (meters)
        leg_index : int
            Leg index (0=Front-Right, 1=Front-Left, 2=Back-Right, 3=Back-Left)

        Returns
        -------
        tuple or None
            (hip_deg, thigh_deg, calf_deg) in degrees, or None if unsafe
        """
        # Calculate inverse kinematics
        joint_angles_rad = self.leg_inverse_kinematics(foot_position, leg_index)

        if joint_angles_rad is None:
            return None

        # Check joint limits
        if not self.check_joint_limits(joint_angles_rad, leg_index):
            return None

        # Convert to degrees
        hip_deg = math.degrees(joint_angles_rad[0])
        thigh_deg = math.degrees(joint_angles_rad[1])
        calf_deg = math.degrees(joint_angles_rad[2])

        return (hip_deg, thigh_deg, calf_deg)


class TrotBotLegTester:
    def __init__(self):
        """Initialize comprehensive TrotBot leg tester"""
        print("🦿 TrotBot Comprehensive Leg Tester")
        print("=" * 50)

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

        # Initialize inverse kinematics
        self.kinematics = TrotBotLegKinematics()

        # Current active leg for single-leg operations (default: Front-Right)
        self.active_leg_index = 0

        # Create servo channel mapping for all legs
        self.create_servo_mapping()

        # Initialize test positions
        self.initialize_positions()
        self.display_position_info()

        print(f"🎯 Default active leg: {self.kinematics.get_leg_name(self.active_leg_index)}")
        print(f"📍 All servo channels mapped for 4 legs × 3 axes = 12 servos")

    def create_servo_mapping(self):
        """Create comprehensive servo channel mapping for all legs"""
        self.servo_channels = {}

        # Map all 12 servos (3 axes × 4 legs)
        for leg_index in range(4):
            leg_name = self.kinematics.get_leg_name(leg_index)
            self.servo_channels[leg_index] = {
                'name': leg_name,
                'hip': self.pwm_params.pins[0, leg_index],      # Abduction
                'thigh': self.pwm_params.pins[1, leg_index],    # Inner Hip/Thigh
                'calf': self.pwm_params.pins[2, leg_index]      # Outer Hip/Calf
            }

        print(f"\n📍 Servo Channel Mapping:")
        for leg_index in range(4):
            channels = self.servo_channels[leg_index]
            print(f"  {channels['name']}: Hip={channels['hip']}, Thigh={channels['thigh']}, Calf={channels['calf']}")

    def initialize_positions(self):
        """Initialize test positions using calibration working ranges"""
        self.positions = {
            'neutral': {
                'hip': 0,      # Keep hip neutral
                'thigh': 45,   # Neutral: 45°
                'calf': -45    # Neutral: -45°
            },
            'thigh_extended': {
                'hip': 0,      # Keep hip neutral
                'thigh': 0,    # Thigh extended: 0°
                'calf': -45    # Keep calf at neutral
            },
            'thigh_raised': {
                'hip': 0,      # Keep hip neutral
                'thigh': 90,   # Thigh raised: 90°
                'calf': -45    # Keep calf at neutral
            },
            'calf_extended': {
                'hip': 0,      # Keep hip neutral
                'thigh': 45,   # Keep thigh at neutral
                'calf': 0      # Calf extended: 0°
            },
            'calf_retracted': {
                'hip': 0,      # Keep hip neutral
                'thigh': 45,   # Keep thigh at neutral
                'calf': -90    # Calf retracted: -90°
            }
        }

    def display_position_info(self):
        """Display test position information"""
        print(f"\n📐 Test positions:")
        for pos_name, angles in self.positions.items():
            print(f"  {pos_name}: Hip={angles['hip']}°, Thigh={angles['thigh']}°, Calf={angles['calf']}°")

    def create_neutral_joint_matrix(self):
        """Create a 3x4 joint angles matrix with all legs in neutral position

        Returns
        -------
        numpy.ndarray
            3x4 matrix with neutral positions: Hip=0°, Thigh=45°, Calf=-45°
        """
        joint_angles = np.zeros((3, 4))

        # Set all legs to neutral position (0°, 45°, -45°)
        for leg_index in range(4):
            joint_angles[0, leg_index] = math.radians(0)    # Hip: 0° (neutral abduction)
            joint_angles[1, leg_index] = math.radians(45)   # Thigh: 45° (neutral thigh)
            joint_angles[2, leg_index] = math.radians(-45)  # Calf: -45° (neutral calf)

        return joint_angles

    def set_all_legs_neutral(self):
        """Set all four legs to neutral position for stability

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        try:
            print(f"\n🎯 Setting ALL legs to neutral position for stability:")
            print(f"   Hip=0°, Thigh=45°, Calf=-45° (stable stance configuration)")

            # Create neutral joint matrix
            joint_angles = self.create_neutral_joint_matrix()

            print(f"📡 Sending coordinated servo commands to all 12 servos...")
            # Use matrix-based servo control
            self.hardware_interface.set_actuator_postions(joint_angles)

            print(f"✅ All legs set to neutral position")
            return True

        except Exception as e:
            print(f"❌ Failed to set all legs to neutral: {e}")
            return False

    def set_active_leg(self, leg_index):
        """Set the active leg for single-leg operations"""
        if 0 <= leg_index <= 3:
            self.active_leg_index = leg_index
            print(f"🎯 Active leg set to: {self.kinematics.get_leg_name(leg_index)}")
            return True
        else:
            print(f"❌ Invalid leg index: {leg_index}. Must be 0-3.")
            return False

    def set_leg_position(self, position_name, leg_index=None):
        """Set specified leg to position

        Parameters
        ----------
        position_name : str
            Position name ('neutral', 'thigh_extended', etc.)
        leg_index : int, optional
            Leg index (0-3). If None, uses active leg.

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if leg_index is None:
            leg_index = self.active_leg_index

        if position_name not in self.positions:
            print(f"❌ Unknown position: {position_name}")
            return False

        try:
            angles = self.positions[position_name]
            leg_name = self.kinematics.get_leg_name(leg_index)

            print(f"\n🎯 Setting {leg_name} leg to {position_name} position:")
            print(f"   Hip={angles['hip']}°, Thigh={angles['thigh']}°, Calf={angles['calf']}°")

            # Create 3x4 joint angles matrix with all legs in neutral position
            joint_angles = self.create_neutral_joint_matrix()

            print(f"   Non-active legs maintained in neutral position (Hip=0°, Thigh=45°, Calf=-45°)")

            # Override specified leg with desired position values
            joint_angles[0, leg_index] = math.radians(angles['hip'])   # Hip
            joint_angles[1, leg_index] = math.radians(angles['thigh']) # Thigh
            joint_angles[2, leg_index] = math.radians(angles['calf'])  # Calf

            print(f"📡 Sending servo commands via matrix-based control...")
            # Use matrix-based servo control
            self.hardware_interface.set_actuator_postions(joint_angles)

            print(f"✅ {leg_name} position set to {position_name}")
            return True

        except Exception as e:
            print(f"❌ Failed to set {position_name} position: {e}")
            return False

    def set_all_legs_position(self, position_name):
        """Set all four legs to the same position simultaneously

        Parameters
        ----------
        position_name : str
            Position name ('neutral', 'thigh_extended', etc.)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if position_name not in self.positions:
            print(f"❌ Unknown position: {position_name}")
            return False

        try:
            angles = self.positions[position_name]

            print(f"\n🎯 Setting ALL legs to {position_name} position:")
            print(f"   Hip={angles['hip']}°, Thigh={angles['thigh']}°, Calf={angles['calf']}°")

            # Create 3x4 joint angles matrix (3 axes, 4 legs)
            joint_angles = np.zeros((3, 4))

            # Set all legs to the same position
            for leg_index in range(4):
                joint_angles[0, leg_index] = math.radians(angles['hip'])   # Hip
                joint_angles[1, leg_index] = math.radians(angles['thigh']) # Thigh
                joint_angles[2, leg_index] = math.radians(angles['calf'])  # Calf

            print(f"📡 Sending coordinated servo commands to all 12 servos...")
            # Use matrix-based servo control
            self.hardware_interface.set_actuator_postions(joint_angles)

            print(f"✅ All legs set to {position_name} position")
            return True

        except Exception as e:
            print(f"❌ Failed to set all legs to {position_name}: {e}")
            return False

    def test_basic_movement(self, leg_index=None):
        """Test basic movement ranges for specified leg"""
        if leg_index is None:
            leg_index = self.active_leg_index

        leg_name = self.kinematics.get_leg_name(leg_index)
        print(f"\n🔄 {leg_name} Leg Movement Test")
        print(f"=" * 50)

        # Movement sequence
        movement_sequence = [
            ('neutral', 3.0),          # Neutral position
            ('thigh_extended', 3.0),   # Extend thigh to 0°
            ('thigh_raised', 3.0),     # Raise thigh to 90°
            ('neutral', 2.0),          # Return to neutral
            ('calf_extended', 3.0),    # Extend calf to 0°
            ('calf_retracted', 3.0),   # Retract calf to -90°
            ('neutral', 2.0),          # Return to neutral
        ]

        print(f"📋 Movement sequence: {len(movement_sequence)} positions")

        try:
            for i, (position, hold_time) in enumerate(movement_sequence):
                print(f"\n--- Step {i+1}/{len(movement_sequence)}: {position.upper()} ---")

                # Set leg position
                if not self.set_leg_position(position, leg_index):
                    print(f"❌ Failed to move to {position} position")
                    return False

                # Hold position
                print(f"⏱️ Holding {position} position for {hold_time}s...")
                time.sleep(hold_time)

            print(f"\n✅ {leg_name} movement test completed successfully")
            return True

        except KeyboardInterrupt:
            print(f"\n⚠️ Movement test interrupted by user")
            # Return to neutral position
            print(f"🔄 Returning to neutral position...")
            self.set_leg_position('neutral', leg_index)
            return False
        except Exception as e:
            print(f"\n❌ Movement test failed: {e}")
            # Return to neutral position
            print(f"🔄 Returning to neutral position...")
            self.set_leg_position('neutral', leg_index)
            return False

    def test_coordination(self):
        """Test coordinated movement of all four legs"""
        print(f"\n🔄 Coordinated Four-Leg Movement Test")
        print(f"=" * 50)

        # Coordinated movement sequence
        coordination_sequence = [
            ('neutral', 3.0),          # All legs to neutral
            ('thigh_extended', 3.0),   # All legs extend thigh
            ('neutral', 2.0),          # Return to neutral
            ('calf_extended', 3.0),    # All legs extend calf
            ('neutral', 2.0),          # Return to neutral
        ]

        print(f"📋 Coordination sequence: {len(coordination_sequence)} synchronized positions")

        try:
            for i, (position, hold_time) in enumerate(coordination_sequence):
                print(f"\n--- Step {i+1}/{len(coordination_sequence)}: ALL LEGS {position.upper()} ---")

                if not self.set_all_legs_position(position):
                    print(f"❌ Failed to set all legs to {position}")
                    return False

                print(f"⏱️ Holding synchronized {position} position for {hold_time}s...")
                time.sleep(hold_time)

            print(f"\n✅ Coordinated movement test completed successfully")
            return True

        except KeyboardInterrupt:
            print(f"\n⚠️ Coordination test interrupted by user")
            print(f"🔄 Returning all legs to neutral...")
            self.set_all_legs_position('neutral')
            return False
        except Exception as e:
            print(f"\n❌ Coordination test failed: {e}")
            print(f"🔄 Returning all legs to neutral...")
            self.set_all_legs_position('neutral')
            return False

    def set_foot_position(self, foot_position, leg_index=None, position_name="custom"):
        """Set specified leg to achieve specific foot position using inverse kinematics

        Parameters
        ----------
        foot_position : array_like
            Foot position [x, y, z] relative to hip joint in meters
        leg_index : int, optional
            Leg index (0-3). If None, uses active leg.
        position_name : str
            Name for this position (for debug output)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if leg_index is None:
            leg_index = self.active_leg_index

        try:
            x, y, z = foot_position
            leg_name = self.kinematics.get_leg_name(leg_index)
            print(f"\n🎯 Setting {leg_name} foot to {position_name} position:")
            print(f"   Target foot position: x={x*1000:.1f}mm, y={y*1000:.1f}mm, z={z*1000:.1f}mm")

            # Calculate required joint angles using inverse kinematics
            joint_angles = self.kinematics.foot_position_to_joint_angles(foot_position, leg_index)

            if joint_angles is None:
                print(f"❌ Target position is unreachable or unsafe")
                return False

            hip_deg, thigh_deg, calf_deg = joint_angles
            print(f"   Calculated joint angles: Hip={hip_deg:.1f}°, Thigh={thigh_deg:.1f}°, Calf={calf_deg:.1f}°")

            # Create joint angles matrix with all legs in neutral position
            joint_angles_matrix = self.create_neutral_joint_matrix()

            print(f"   Non-active legs maintained in neutral position (Hip=0°, Thigh=45°, Calf=-45°)")

            # Override specified leg with calculated values
            joint_angles_matrix[0, leg_index] = math.radians(hip_deg)   # Hip
            joint_angles_matrix[1, leg_index] = math.radians(thigh_deg) # Thigh
            joint_angles_matrix[2, leg_index] = math.radians(calf_deg)  # Calf

            print(f"📡 Sending servo commands via inverse kinematics...")
            # Use matrix-based servo control
            self.hardware_interface.set_actuator_postions(joint_angles_matrix)

            print(f"✅ {leg_name} foot position set to {position_name}")
            return True

        except Exception as e:
            print(f"❌ Failed to set foot position {position_name}: {e}")
            return False

    def test_vertical_movement(self, num_cycles=5, move_time=0.3, leg_index=None):
        """Test rapid vertical foot movement using inverse kinematics

        Parameters
        ----------
        num_cycles : int
            Number of complete up-down cycles to perform (default: 5)
        move_time : float
            Time to hold each position (seconds, default: 0.3s for rapid movement)
        leg_index : int, optional
            Leg index (0-3). If None, uses active leg.

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        if leg_index is None:
            leg_index = self.active_leg_index

        leg_name = self.kinematics.get_leg_name(leg_index)
        print(f"\n🔄 Rapid Vertical {leg_name} Foot Movement Test (Inverse Kinematics)")
        print(f"=" * 60)

        # Define vertical movement range based on TrotBot default stance
        base_x = 0.0    # Directly below hip (most conservative)
        base_y = -0.03 if leg_index in [0, 2] else 0.03  # Right legs negative, left legs positive (30mm from hip)
        z_low = -0.09   # 90mm below hip (ground level, using new reach)
        z_high = -0.06  # 60mm below hip (lifted position)

        print(f"📐 Movement parameters:")
        print(f"   Base position: x={base_x*1000:.0f}mm, y={base_y*1000:.0f}mm")
        print(f"   Vertical range: {z_low*1000:.0f}mm to {z_high*1000:.0f}mm ({(z_high-z_low)*1000:.0f}mm total)")
        print(f"   Cycles: {num_cycles}, Move time: {move_time}s per position")

        # Validate workspace before starting movement with detailed diagnostics
        print(f"\n🔍 Detailed workspace validation for {leg_name}...")
        low_position = [base_x, base_y, z_low]
        high_position = [base_x, base_y, z_high]

        # Validate low position
        low_valid, low_diag = self.kinematics.validate_workspace_detailed(low_position, leg_index)
        print(f"📍 Low position ({low_diag['position']}):")
        print(f"   Distance from hip: {low_diag['distance_from_hip']} (max: {low_diag['max_reach']})")
        if low_diag['ik_successful']:
            print(f"   Joint angles: {low_diag['joint_angles']}")
            print(f"   Within limits: Hip={low_diag['hip_in_range']}, Thigh={low_diag['thigh_in_range']}, Calf={low_diag['calf_in_range']}")

        if not low_valid:
            print(f"❌ Low position unreachable or unsafe for {leg_name}")
            return False

        # Validate high position
        high_valid, high_diag = self.kinematics.validate_workspace_detailed(high_position, leg_index)
        print(f"📍 High position ({high_diag['position']}):")
        print(f"   Distance from hip: {high_diag['distance_from_hip']} (max: {high_diag['max_reach']})")
        if high_diag['ik_successful']:
            print(f"   Joint angles: {high_diag['joint_angles']}")
            print(f"   Within limits: Hip={high_diag['hip_in_range']}, Thigh={high_diag['thigh_in_range']}, Calf={high_diag['calf_in_range']}")

        if not high_valid:
            print(f"❌ High position unreachable or unsafe for {leg_name}")
            return False

        print(f"✅ Comprehensive workspace validation passed for {leg_name}")
        print(f"   Movement range: {(z_high-z_low)*1000:.0f}mm vertical travel")

        try:
            # Start at low position
            start_position = [base_x, base_y, z_low]
            print(f"\n--- Starting at ground level ---")
            if not self.set_foot_position(start_position, leg_index, "ground_level"):
                return False
            time.sleep(move_time)

            # Perform rapid up-down cycles
            print(f"\n--- Performing {num_cycles} rapid up-down cycles ---")
            for cycle in range(1, num_cycles + 1):
                # Move UP
                up_position = [base_x, base_y, z_high]
                print(f"\nCycle {cycle}/{num_cycles}: Moving UP (z={z_high*1000:.0f}mm)")
                if not self.set_foot_position(up_position, leg_index, f"cycle_{cycle:02d}_up"):
                    print(f"❌ Failed at cycle {cycle} UP, stopping movement")
                    return False
                time.sleep(move_time)

                # Move DOWN
                down_position = [base_x, base_y, z_low]
                print(f"Cycle {cycle}/{num_cycles}: Moving DOWN (z={z_low*1000:.0f}mm)")
                if not self.set_foot_position(down_position, leg_index, f"cycle_{cycle:02d}_down"):
                    print(f"❌ Failed at cycle {cycle} DOWN, stopping movement")
                    return False
                time.sleep(move_time)

            print(f"\n✅ {leg_name} rapid vertical movement test completed successfully")
            return True

        except KeyboardInterrupt:
            print(f"\n⚠️ Rapid vertical movement test interrupted by user")
            # Return to safe position
            print(f"🔄 Returning to safe ground position...")
            safe_position = [base_x, base_y, z_low]
            self.set_foot_position(safe_position, leg_index, "safe_ground")
            return False
        except Exception as e:
            print(f"\n❌ Rapid vertical movement test failed: {e}")
            # Return to safe position
            print(f"🔄 Returning to safe ground position...")
            safe_position = [base_x, base_y, z_low]
            self.set_foot_position(safe_position, leg_index, "safe_ground")
            return False

    def calculate_synchronized_joint_matrix(self, leg_positions_dict, position_type="custom"):
        """Calculate joint angles matrix for all four legs simultaneously

        Parameters
        ----------
        leg_positions_dict : dict
            Dictionary with leg_index as key and foot position [x,y,z] as value
        position_type : str
            Description of the position type for debug output

        Returns
        -------
        tuple
            (joint_angles_matrix, success) where success is True if all calculations succeeded
        """
        joint_angles_matrix = np.zeros((3, 4))
        calculated_angles = {}

        print(f"🔄 Calculating {position_type} joint angles for all 4 legs...")

        for leg_index in range(4):
            if leg_index not in leg_positions_dict:
                print(f"❌ Missing position data for leg {leg_index}")
                return None, False

            foot_position = leg_positions_dict[leg_index]['position']
            leg_name = leg_positions_dict[leg_index]['name']

            joint_angles = self.kinematics.foot_position_to_joint_angles(foot_position, leg_index)
            if joint_angles is None:
                print(f"❌ Failed to calculate {position_type} position for {leg_name}")
                return None, False

            hip_deg, thigh_deg, calf_deg = joint_angles
            calculated_angles[leg_index] = (hip_deg, thigh_deg, calf_deg)
            joint_angles_matrix[0, leg_index] = math.radians(hip_deg)
            joint_angles_matrix[1, leg_index] = math.radians(thigh_deg)
            joint_angles_matrix[2, leg_index] = math.radians(calf_deg)
            print(f"   {leg_name}: Hip={hip_deg:.1f}°, Thigh={thigh_deg:.1f}°, Calf={calf_deg:.1f}°")

        return joint_angles_matrix, True

    def _gradual_vertical_move_all_legs(self, start_positions, end_positions, step_size_mm, step_delay_seconds, movement_name):
        """Helper function to perform gradual vertical movement for all legs simultaneously

        Parameters
        ----------
        start_positions : dict
            Dictionary with leg_index as key and position data as value
        end_positions : dict
            Dictionary with leg_index as key and target position data as value
        step_size_mm : float
            Vertical increment per step in millimeters
        step_delay_seconds : float
            Pause duration between each step
        movement_name : str
            Name of the movement for progress feedback

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        # Convert step size from mm to meters
        step_size_m = step_size_mm / 1000.0

        # Calculate total vertical distance for each leg
        total_distances = {}
        for leg_index in range(4):
            start_z = start_positions[leg_index]['position'][2]
            end_z = end_positions[leg_index]['position'][2]
            total_distances[leg_index] = abs(end_z - start_z)

        # Find maximum distance to determine number of steps
        max_distance = max(total_distances.values())
        num_steps = max(1, int(max_distance / step_size_m))

        print(f"📊 Gradual {movement_name} Movement Analysis:")
        print(f"   Step size: {step_size_mm}mm ({step_size_m:.3f}m)")
        print(f"   Maximum travel distance: {max_distance*1000:.1f}mm")
        print(f"   Number of incremental steps: {num_steps}")
        print(f"   Step delay: {step_delay_seconds}s")
        print(f"   Total movement time: {num_steps * step_delay_seconds:.1f}s")

        try:
            # Perform gradual movement step by step
            for step in range(num_steps + 1):  # +1 to include final position
                # Calculate interpolation factor (0.0 to 1.0)
                if num_steps == 0:
                    t = 1.0  # Single step case
                else:
                    t = step / num_steps

                # Calculate intermediate positions for all legs
                intermediate_positions = {}
                for leg_index in range(4):
                    start_pos = start_positions[leg_index]['position']
                    end_pos = end_positions[leg_index]['position']

                    # Linear interpolation for each coordinate
                    intermediate_x = start_pos[0] + t * (end_pos[0] - start_pos[0])
                    intermediate_y = start_pos[1] + t * (end_pos[1] - start_pos[1])
                    intermediate_z = start_pos[2] + t * (end_pos[2] - start_pos[2])

                    intermediate_positions[leg_index] = {
                        'position': [intermediate_x, intermediate_y, intermediate_z],
                        'name': start_positions[leg_index]['name']
                    }

                # Calculate current Z position for progress display
                current_z_mm = intermediate_positions[0]['position'][2] * 1000
                remaining_steps = num_steps - step

                # Progress feedback
                if step == 0:
                    print(f"\n🚀 Starting gradual {movement_name} movement...")
                elif step == num_steps:
                    print(f"\n🎯 Final step: Reaching target position (Z={current_z_mm:.0f}mm)")
                else:
                    print(f"\n📍 Step {step}/{num_steps}: Z={current_z_mm:.0f}mm (remaining: {remaining_steps} steps)")

                # Calculate joint angles for all legs at this intermediate position
                joint_angles_matrix, success = self.calculate_synchronized_joint_matrix(
                    intermediate_positions, f"{movement_name} Step {step}/{num_steps}"
                )
                if not success:
                    print(f"❌ Failed at step {step} of gradual {movement_name} movement")
                    return False

                # Send coordinated command to all servos
                self.hardware_interface.set_actuator_postions(joint_angles_matrix)

                if step == num_steps:
                    print(f"✅ Gradual {movement_name} movement completed - all legs at target position")
                else:
                    print(f"✅ Step {step} completed - all legs moved synchronously")

                # Wait before next step (except for the final step)
                if step < num_steps:
                    time.sleep(step_delay_seconds)

            return True

        except Exception as e:
            print(f"❌ Gradual {movement_name} movement failed at step {step}: {e}")
            return False

    def test_all_vertical_sync(self, num_cycles=3, move_time=0.5, step_size_mm=5, step_delay_seconds=0.5):
        """Test synchronized vertical movement of all four legs simultaneously with gradual incremental steps

        Parameters
        ----------
        num_cycles : int
            Number of complete up-down cycles to perform (default: 3)
        move_time : float
            Time to hold each final position (seconds, default: 0.5s for coordination)
        step_size_mm : float
            Vertical increment per step in millimeters (default: 5mm)
        step_delay_seconds : float
            Pause duration between each incremental step (default: 0.5 seconds)

        Returns
        -------
        bool
            True if successful, False otherwise
        """
        print(f"\n🔄 Synchronized Gradual Vertical Movement Test - ALL FOUR LEGS")
        print(f"=" * 70)
        print(f"🎯 All legs will move together with smooth incremental steps")

        # Define synchronized movement parameters for all legs
        # Each leg uses appropriate base position based on left/right side
        leg_positions = {}
        for leg_index in range(4):
            leg_name = self.kinematics.get_leg_name(leg_index)
            base_x = 0.0  # Directly below hip for all legs
            base_y = -0.03 if leg_index in [0, 2] else 0.03  # Right legs negative, left legs positive
            z_low = -0.12   # 120mm below hip (ground level)
            z_high = -0.06  # 60mm below hip (lifted position)

            leg_positions[leg_index] = {
                'name': leg_name,
                'base_x': base_x,
                'base_y': base_y,
                'z_low': z_low,
                'z_high': z_high,
                'low_position': [base_x, base_y, z_low],
                'high_position': [base_x, base_y, z_high]
            }

        # Calculate movement timing estimates
        vertical_distance_mm = (z_high - z_low) * 1000  # Convert to mm
        steps_per_movement = max(1, int(abs(vertical_distance_mm) / step_size_mm))
        gradual_move_time = steps_per_movement * step_delay_seconds
        total_estimated_time = num_cycles * 2 * (gradual_move_time + move_time)

        print(f"📐 Synchronized gradual movement parameters:")
        print(f"   Cycles: {num_cycles}, Final hold time: {move_time}s per position")
        print(f"   Gradual movement: {step_size_mm}mm steps with {step_delay_seconds}s delays")
        print(f"   Steps per movement: {steps_per_movement}, Time per movement: {gradual_move_time:.1f}s")
        print(f"   Total estimated time: {total_estimated_time:.1f}s")
        print(f"   Vertical range: {z_low*1000:.0f}mm to {z_high*1000:.0f}mm ({abs(vertical_distance_mm):.0f}mm total)")

        # Display leg positions
        for leg_index in range(4):
            pos = leg_positions[leg_index]
            print(f"   {pos['name']}: x={pos['base_x']*1000:.0f}mm, y={pos['base_y']*1000:.0f}mm")

        # Validate workspace for all legs before starting
        print(f"\n🔍 Validating workspace for all four legs...")
        for leg_index in range(4):
            pos = leg_positions[leg_index]
            print(f"   Testing {pos['name']} workspace...")

            if not self.kinematics.foot_position_to_joint_angles(pos['low_position'], leg_index):
                print(f"❌ {pos['name']} low position unreachable")
                return False

            if not self.kinematics.foot_position_to_joint_angles(pos['high_position'], leg_index):
                print(f"❌ {pos['name']} high position unreachable")
                return False

        print(f"✅ Workspace validation passed for all legs")

        try:
            # Start all legs at low position with gradual movement
            print(f"\n--- SYNCHRONIZED START: Gradual movement to ground level ---")

            # Get current positions (assume neutral start)
            current_positions = {}
            for leg_index in range(4):
                pos = leg_positions[leg_index]
                # Start from neutral position (approximate)
                neutral_z = -0.075  # Approximate neutral height
                current_positions[leg_index] = {
                    'position': [pos['base_x'], pos['base_y'], neutral_z],
                    'name': pos['name']
                }

            # Prepare target positions (ground level)
            target_positions = {}
            for leg_index in range(4):
                pos = leg_positions[leg_index]
                target_positions[leg_index] = {
                    'position': pos['low_position'],
                    'name': pos['name']
                }

            # Perform gradual movement to ground level
            if not self._gradual_vertical_move_all_legs(
                current_positions, target_positions, step_size_mm, step_delay_seconds, "GROUND LEVEL"
            ):
                return False

            print(f"✅ All 4 legs reached ground level with smooth gradual movement")
            time.sleep(move_time)

            # Perform synchronized up-down cycles with gradual movement
            print(f"\n--- SYNCHRONIZED CYCLES: {num_cycles} gradual up-down movements ---")
            for cycle in range(1, num_cycles + 1):
                # GRADUAL SYNCHRONIZED MOVE UP - All legs together
                print(f"\n🔼 Cycle {cycle}/{num_cycles}: GRADUAL ALL LEGS UP MOVEMENT")

                # Current positions (ground level)
                current_positions = {}
                for leg_index in range(4):
                    pos = leg_positions[leg_index]
                    current_positions[leg_index] = {
                        'position': pos['low_position'],
                        'name': pos['name']
                    }

                # Target positions (lifted level)
                up_positions = {}
                for leg_index in range(4):
                    pos = leg_positions[leg_index]
                    up_positions[leg_index] = {
                        'position': pos['high_position'],
                        'name': pos['name']
                    }

                # Perform gradual UP movement
                if not self._gradual_vertical_move_all_legs(
                    current_positions, up_positions, step_size_mm, step_delay_seconds, f"UP Cycle {cycle}"
                ):
                    print(f"❌ Failed at cycle {cycle} UP movement, stopping synchronized test")
                    return False

                print(f"✅ Cycle {cycle} UP movement completed - all legs lifted with smooth motion")
                time.sleep(move_time)

                # GRADUAL SYNCHRONIZED MOVE DOWN - All legs together
                print(f"\n🔽 Cycle {cycle}/{num_cycles}: GRADUAL ALL LEGS DOWN MOVEMENT")

                # Current positions (lifted level)
                current_positions = {}
                for leg_index in range(4):
                    pos = leg_positions[leg_index]
                    current_positions[leg_index] = {
                        'position': pos['high_position'],
                        'name': pos['name']
                    }

                # Target positions (ground level)
                down_positions = {}
                for leg_index in range(4):
                    pos = leg_positions[leg_index]
                    down_positions[leg_index] = {
                        'position': pos['low_position'],
                        'name': pos['name']
                    }

                # Perform gradual DOWN movement
                if not self._gradual_vertical_move_all_legs(
                    current_positions, down_positions, step_size_mm, step_delay_seconds, f"DOWN Cycle {cycle}"
                ):
                    print(f"❌ Failed at cycle {cycle} DOWN movement, stopping synchronized test")
                    return False

                print(f"✅ Cycle {cycle} DOWN movement completed - all legs lowered with smooth motion")
                time.sleep(move_time)

            print(f"\n✅ Synchronized gradual vertical movement test completed successfully")
            print(f"🎯 All four legs moved with smooth incremental steps through {num_cycles} cycles")
            print(f"📊 Movement summary: {step_size_mm}mm steps, {step_delay_seconds}s delays, total coordination achieved")
            return True

        except KeyboardInterrupt:
            print(f"\n⚠️ Synchronized movement test interrupted by user")
            print(f"🔄 Returning all legs to neutral position...")
            self.set_all_legs_neutral()
            return False
        except Exception as e:
            print(f"\n❌ Synchronized movement test failed: {e}")
            print(f"🔄 Returning all legs to neutral position...")
            self.set_all_legs_neutral()
            return False

    def display_calibration_info(self, leg_index=None):
        """Display current calibration data for debugging"""
        if leg_index is None:
            leg_index = self.active_leg_index

        leg_name = self.kinematics.get_leg_name(leg_index)
        print(f"\n🔧 Calibration Information for {leg_name}:")
        print(f"  MICROS_PER_RAD: {self.servo_params.micros_per_rad:.3f}")
        print(f"  Neutral angles (degrees):")
        for axis in range(3):
            axis_name = ['Hip', 'Thigh', 'Calf'][axis]
            neutral_deg = self.servo_params.neutral_angle_degrees[axis, leg_index]
            print(f"    {axis_name}: {neutral_deg:.2f}°")
        print(f"  Servo multipliers:")
        for axis in range(3):
            axis_name = ['Hip', 'Thigh', 'Calf'][axis]
            multiplier = self.servo_params.servo_multipliers[axis, leg_index]
            print(f"    {axis_name}: {multiplier}")

    def interactive_mode(self):
        """Interactive testing mode with TrotBot support"""
        active_leg_name = self.kinematics.get_leg_name(self.active_leg_index)
        print(f"\n🎮 Interactive TrotBot Leg Testing Mode")
        print(f"=" * 50)
        print(f"🎯 Active leg: {active_leg_name} (index {self.active_leg_index})")
        print(f"🔧 Geometry: Thigh={self.kinematics.LEG_L1*1000:.1f}mm, Calf={self.kinematics.LEG_L2*1000:.1f}mm")
        print(f"\nAvailable Commands:")
        print(f"  SINGLE LEG COMMANDS (affect active leg only):")
        print(f"    neutral         - Move to neutral position (thigh: 45°, calf: -45°)")
        print(f"    thigh_extended  - Extend thigh to 0° (calf stays at -45°)")
        print(f"    thigh_raised    - Raise thigh to 90° (calf stays at -45°)")
        print(f"    calf_extended   - Extend calf to 0° (thigh stays at 45°)")
        print(f"    calf_retracted  - Retract calf to -90° (thigh stays at 45°)")
        print(f"    test            - Run basic movement test")
        print(f"    vertical [N]    - Run rapid vertical movement test (N cycles, default: 5)")
        print(f"  MULTI-LEG COMMANDS:")
        print(f"    all_neutral     - Move ALL legs to neutral position")
        print(f"    all_vertical_sync [cycles] [step_size_mm] [step_delay_s]")
        print(f"                    - Run gradual synchronized vertical test on ALL legs")
        print(f"                    - cycles: Number of up-down cycles (default: 3)")
        print(f"                    - step_size_mm: Vertical increment per step (default: 5mm)")
        print(f"                    - step_delay_s: Pause between steps (default: 0.5s)")
        print(f"    test_coordination - Test coordinated movement of all legs")
        print(f"  CONTROL COMMANDS:")
        print(f"    leg_select [0-3] - Select active leg (0=FR, 1=FL, 2=BR, 3=BL)")
        print(f"    calibration     - Show calibration information")
        print(f"    help            - Show this command list")
        print(f"    quit            - Exit interactive mode")
        print(f"=" * 50)

        while True:
            try:
                active_leg_name = self.kinematics.get_leg_name(self.active_leg_index)
                cmd = input(f"\n{active_leg_name}> ").strip().lower()
                if not cmd:
                    continue

                if cmd in ["quit", "exit", "q"]:
                    # Return to neutral before exiting
                    print(f"🔄 Returning active leg to neutral position before exit...")
                    self.set_leg_position('neutral')
                    break

                # Single leg commands
                elif cmd in ['neutral', 'thigh_extended', 'thigh_raised', 'calf_extended', 'calf_retracted']:
                    self.set_leg_position(cmd)

                elif cmd == 'test':
                    self.test_basic_movement()

                elif cmd.startswith('vertical'):
                    # Parse optional cycles parameter
                    parts = cmd.split()
                    if len(parts) > 1:
                        try:
                            cycles = int(parts[1])
                            if cycles < 1:
                                print(f"❌ Invalid cycles: {cycles}. Must be >= 1")
                                continue
                        except ValueError:
                            print(f"❌ Invalid cycles: '{parts[1]}'. Must be a number")
                            continue
                    else:
                        cycles = 5  # Default

                    print(f"🔄 Running {active_leg_name} rapid vertical movement with {cycles} cycles")
                    self.test_vertical_movement(num_cycles=cycles)

                # Multi-leg commands
                elif cmd == 'all_neutral':
                    self.set_all_legs_neutral()

                elif cmd.startswith('all_vertical_sync'):
                    # Parse optional parameters for synchronized vertical movement
                    # Format: all_vertical_sync [cycles] [step_size_mm] [step_delay_seconds]
                    parts = cmd.split()
                    cycles = 3  # Default
                    step_size_mm = 5  # Default
                    step_delay_seconds = 0.5  # Default

                    # Parse cycles parameter
                    if len(parts) > 1:
                        try:
                            cycles = int(parts[1])
                            if cycles < 1:
                                print(f"❌ Invalid cycles: {cycles}. Must be >= 1")
                                continue
                        except ValueError:
                            print(f"❌ Invalid cycles: '{parts[1]}'. Must be a number")
                            continue

                    # Parse step_size_mm parameter
                    if len(parts) > 2:
                        try:
                            step_size_mm = float(parts[2])
                            if step_size_mm <= 0:
                                print(f"❌ Invalid step size: {step_size_mm}mm. Must be > 0")
                                continue
                        except ValueError:
                            print(f"❌ Invalid step size: '{parts[2]}'. Must be a number")
                            continue

                    # Parse step_delay_seconds parameter
                    if len(parts) > 3:
                        try:
                            step_delay_seconds = float(parts[3])
                            if step_delay_seconds < 0:
                                print(f"❌ Invalid step delay: {step_delay_seconds}s. Must be >= 0")
                                continue
                        except ValueError:
                            print(f"❌ Invalid step delay: '{parts[3]}'. Must be a number")
                            continue

                    print(f"🔄 Running SYNCHRONIZED vertical movement test on ALL legs")
                    print(f"   Parameters: {cycles} cycles, {step_size_mm}mm steps, {step_delay_seconds}s delay")
                    self.test_all_vertical_sync(num_cycles=cycles, step_size_mm=step_size_mm, step_delay_seconds=step_delay_seconds)

                elif cmd == 'test_coordination':
                    self.test_coordination()

                # Control commands
                elif cmd.startswith('leg_select'):
                    parts = cmd.split()
                    if len(parts) > 1:
                        try:
                            leg_idx = int(parts[1])
                            self.set_active_leg(leg_idx)
                        except ValueError:
                            print(f"❌ Invalid leg index: '{parts[1]}'. Must be 0-3")
                    else:
                        print(f"❌ Usage: leg_select [0-3]")

                elif cmd == 'calibration':
                    self.display_calibration_info()

                elif cmd == 'help':
                    # Re-display the help
                    self.interactive_mode()
                    return

                else:
                    print(f"❌ Unknown command: '{cmd}'. Type 'help' for available commands.")

            except KeyboardInterrupt:
                print(f"\n🔄 Returning active leg to neutral position...")
                self.set_leg_position('neutral')
                break
            except Exception as e:
                print(f"❌ Error: {e}")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="TrotBot Comprehensive Leg Tester")
    parser.add_argument("--test", "-t", action="store_true",
                       help="Run basic movement test on active leg")
    parser.add_argument("--vertical", "-v", type=int, nargs='?', const=5, metavar='CYCLES',
                       help="Run rapid vertical foot movement test (default: 5 cycles)")
    parser.add_argument("--vertical-sync", "-vs", type=int, nargs='?', const=3, metavar='CYCLES',
                       help="Run gradual synchronized vertical movement test on ALL legs (default: 3 cycles)")
    parser.add_argument("--coordination", "-c", action="store_true",
                       help="Run coordinated four-leg movement test")
    parser.add_argument("--leg", "-l", type=int, choices=[0,1,2,3], default=0,
                       help="Select active leg (0=FR, 1=FL, 2=BR, 3=BL, default: 0)")
    parser.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive testing mode")

    args = parser.parse_args()

    # Create tester
    tester = TrotBotLegTester()

    # Set active leg if specified
    if args.leg != 0:
        tester.set_active_leg(args.leg)

    # Display calibration information
    tester.display_calibration_info()

    # User confirmation
    print(f"\n⚠️ SAFETY CONFIRMATION")
    print(f"This test will move TrotBot leg servos.")
    print(f"Ensure:")
    print(f"  • Robot is securely held or supported")
    print(f"  • All legs can move freely")
    print(f"  • Emergency stop is accessible")
    print(f"  • Servo power is connected")

    response = input(f"\nProceed with TrotBot leg movement test? (y/n): ").strip().lower()
    if response != 'y':
        print(f"Test cancelled by user")
        sys.exit(0)

    try:
        if args.test:
            # Run basic movement test
            tester.test_basic_movement()

        elif args.vertical:
            # Run rapid vertical movement test with specified cycles
            cycles = args.vertical if isinstance(args.vertical, int) else 5
            print(f"🔄 Running rapid vertical movement test with {cycles} cycles")
            tester.test_vertical_movement(num_cycles=cycles)

        elif hasattr(args, 'vertical_sync') and args.vertical_sync:
            # Run synchronized vertical movement test with specified cycles
            cycles = args.vertical_sync if isinstance(args.vertical_sync, int) else 3
            print(f"🔄 Running synchronized vertical movement test on ALL legs with {cycles} cycles")
            tester.test_all_vertical_sync(num_cycles=cycles)

        elif args.coordination:
            # Run coordinated four-leg movement test
            print(f"🔄 Running coordinated four-leg movement test")
            tester.test_coordination()

        elif args.interactive:
            # Interactive mode only
            tester.interactive_mode()

        else:
            # Default: enter interactive mode directly
            print(f"\n🎮 No specific test selected, entering interactive mode...")
            print(f"💡 Tip: Use 'test', 'all_neutral', 'all_vertical_sync', or 'test_coordination' commands")
            tester.interactive_mode()

    except KeyboardInterrupt:
        print(f"\n⚠️ Testing interrupted by user")
        # Ensure active leg returns to neutral
        tester.set_leg_position('neutral')
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        # Ensure active leg returns to neutral
        tester.set_leg_position('neutral')

    print(f"\n🏁 TrotBot leg testing completed")


if __name__ == "__main__":
    main()
