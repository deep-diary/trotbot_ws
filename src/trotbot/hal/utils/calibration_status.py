#!/usr/bin/env python3
"""
TrotBot Calibration Status Tool
Check current servo calibration status and display calibration values
"""

import sys
import json
from hal.hardware import servo_calibration

def main():
    """Display current calibration status and values"""
    print("🤖 TrotBot Servo Calibration Status")
    print("=" * 50)
    
    try:
        status = servo_calibration.get_calibration_status()
        
        print(f"📁 Calibration file: {status['calibration_path']}")
        print(f"📊 File exists: {'✅ Yes' if status['calibration_file_exists'] else '❌ No'}")
        print(f"📊 Calibration source: {status['calibration_source']}")
        print(f"🔧 Using defaults: {'⚠️ Yes' if status['using_defaults'] else '✅ No'}")
        print()
        
        print("📐 Current Calibration Values:")
        print(f"   MICROS_PER_RAD: {status['micros_per_rad']:.6f}")
        print("   NEUTRAL_ANGLE_DEGREES:")
        
        angles = status['neutral_angle_degrees']
        joint_names = ['Abduction', 'Thigh    ', 'Calf     ']
        leg_names = ['FR', 'FL', 'BR', 'BL']
        
        print("      Joint      |", " | ".join(f"{leg:>6}" for leg in leg_names))
        print("      " + "-" * 42)
        
        for i, joint in enumerate(joint_names):
            values = " | ".join(f"{angles[i][j]:>6.1f}" for j in range(4))
            print(f"      {joint} | {values}")
        
        print()
        
        if status['using_defaults']:
            print("⚠️  WARNING: Using default calibration values!")
            print("   Run 'ros2 run trotbot calibrate_tool' to calibrate servos")
        else:
            print("✅ Calibration data loaded successfully")
            
        # Export option
        if len(sys.argv) > 1 and sys.argv[1] == '--json':
            print("\n📄 JSON Export:")
            print(json.dumps(status, indent=2))
            
    except Exception as e:
        print(f"❌ Error checking calibration status: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
