#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'hal'))

import numpy as np
import math
from hal.hardware.config import Configuration

def leg_inverse_kinematics(foot_position, leg_index, config):
    """Calculate joint angles for given foot position and leg using TrotBot geometry
    
    Parameters
    ----------
    foot_position : array_like
        Foot position [x, y, z] relative to body center (meters)
    leg_index : int
        Leg index (0=Front-Right, 1=Front-Left, 2=Back-Right, 3=Back-Left)
    config : Configuration
        TrotBot configuration object
        
    Returns
    -------
    tuple
        (hip_angle, thigh_angle, calf_angle) in radians, or None if unreachable
    """
    x, y, z = foot_position
    
    # Leg origins relative to body center
    leg_origins = np.array([
        [ config.LEG_FB,  config.LEG_FB, -config.LEG_FB, -config.LEG_FB],  # x
        [-config.LEG_LR,  config.LEG_LR, -config.LEG_LR,  config.LEG_LR],  # y  
        [0, 0, 0, 0]  # z
    ])
    
    # Abduction offsets for each leg
    abduction_offsets = np.array([
        -config.ABDUCTION_OFFSET,  # Front-Right (negative = outward)
         config.ABDUCTION_OFFSET,  # Front-Left (positive = outward)
        -config.ABDUCTION_OFFSET,  # Back-Right (negative = outward)
         config.ABDUCTION_OFFSET   # Back-Left (positive = outward)
    ])
    
    # Transform foot position to leg coordinate system
    leg_origin = leg_origins[:, leg_index]
    foot_rel_leg = np.array([x, y, z]) - leg_origin
    x_leg, y_leg, z_leg = foot_rel_leg
    
    abduction_offset = abduction_offsets[leg_index]
    
    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = math.sqrt(y_leg**2 + z_leg**2)
    
    # Check if foot position is reachable (must be outside hip offset)
    if R_body_foot_yz < abs(abduction_offset):
        return None
        
    # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
    hip_foot_angle = math.atan2(z_leg, y_leg)
    
    # Angle between the y-axis and the line from the leg origin to the abduction axis
    phi = math.atan2(0, abduction_offset)
    
    # Ab/adduction angle, relative to the positive y-axis
    hip_angle = phi + hip_foot_angle
    
    # Distance from the abduction axis to the foot
    R_hip_foot_yz = math.sqrt((y_leg - abduction_offset)**2 + z_leg**2)
    
    # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
    theta = math.atan2(-x_leg, R_hip_foot_yz)
    
    # Distance between the hip and foot
    R_hip_foot = math.sqrt(R_hip_foot_yz**2 + x_leg**2)
    
    # Check if the foot position is reachable by the leg
    if R_hip_foot > (config.LEG_L1 + config.LEG_L2):
        return None
        
    # Angle between the line going from hip to foot and the link L1
    try:
        arccos_argument = (config.LEG_L1**2 + R_hip_foot**2 - config.LEG_L2**2) / (2 * config.LEG_L1 * R_hip_foot)
        arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
        trident = math.acos(arccos_argument)
    except:
        return None
        
    # Angle of the first link relative to the tilted negative z axis
    thigh_angle = theta + trident
    
    # Angle between the leg links L1 and L2
    try:
        arccos_argument = (config.LEG_L1**2 + config.LEG_L2**2 - R_hip_foot**2) / (2 * config.LEG_L1 * config.LEG_L2)
        arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
        beta = math.acos(arccos_argument)
    except:
        return None
        
    # Angle of the second link relative to the tilted negative z axis
    calf_angle = thigh_angle - (math.pi - beta)
    
    return (hip_angle, thigh_angle, calf_angle)

def calculate_stance_foot_positions(nominal_height, config):
    """Calculate foot positions for default stance at given height"""
    # Default stance positions (from config.default_stance)
    foot_positions = np.array([
        [ config.delta_x,  config.delta_x, -config.delta_x, -config.delta_x],  # x
        [-config.delta_y,  config.delta_y, -config.delta_y,  config.delta_y],  # y
        [-nominal_height, -nominal_height, -nominal_height, -nominal_height]   # z (negative = below body)
    ])
    return foot_positions

def test_nominal_heights():
    """Test joint angles for current and proposed nominal heights"""
    config = Configuration()
    
    # Test heights
    current_height = 0.09   # Current setting
    proposed_height = 0.105 # Proposed setting
    
    leg_names = ["Front-Right", "Front-Left", "Back-Right", "Back-Left"]
    
    print("🔧 TrotBot Nominal Height Analysis")
    print("=" * 60)
    print(f"TrotBot Leg Geometry:")
    print(f"  Thigh Length (L1): {config.LEG_L1*1000:.1f}mm")
    print(f"  Calf Length (L2): {config.LEG_L2*1000:.1f}mm")
    print(f"  Abduction Offset: {config.ABDUCTION_OFFSET*1000:.1f}mm")
    print(f"  Stance Width: {config.delta_y*1000:.1f}mm")
    print(f"  Stance Length: {config.delta_x*1000:.1f}mm")
    print()
    
    for height_name, height in [("CURRENT", current_height), ("PROPOSED", proposed_height)]:
        print(f"📏 {height_name} Nominal Height: {height*1000:.0f}mm")
        print("-" * 40)
        
        # Calculate foot positions for this height
        foot_positions = calculate_stance_foot_positions(height, config)
        
        all_angles_valid = True
        
        for leg_idx in range(4):
            foot_pos = foot_positions[:, leg_idx]
            angles = leg_inverse_kinematics(foot_pos, leg_idx, config)
            
            if angles is None:
                print(f"  {leg_names[leg_idx]}: ❌ UNREACHABLE")
                all_angles_valid = False
            else:
                hip_rad, thigh_rad, calf_rad = angles
                hip_deg = math.degrees(hip_rad)
                thigh_deg = math.degrees(thigh_rad)
                calf_deg = math.degrees(calf_rad)
                
                print(f"  {leg_names[leg_idx]}:")
                print(f"    Foot Position: [{foot_pos[0]*1000:.1f}, {foot_pos[1]*1000:.1f}, {foot_pos[2]*1000:.1f}]mm")
                print(f"    Hip:   {hip_rad:+.4f} rad ({hip_deg:+6.1f}°)")
                print(f"    Thigh: {thigh_rad:+.4f} rad ({thigh_deg:+6.1f}°)")
                print(f"    Calf:  {calf_rad:+.4f} rad ({calf_deg:+6.1f}°)")
                print()
        
        if all_angles_valid:
            print("  ✅ All leg positions are reachable")
        else:
            print("  ❌ Some leg positions are unreachable")
        print()
    
    print("🎯 Target Neutral Stance: [0°, 45°, -45°]")
    print("📊 Analysis:")
    print("  - Current height (90mm) produces near-zero angles (unstable)")
    print("  - Proposed height (105mm) should produce angles closer to neutral stance")
    print("  - Neutral stance provides stable, balanced robot posture")

if __name__ == "__main__":
    test_nominal_heights()
