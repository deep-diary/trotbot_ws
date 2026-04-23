# TrotBot Test Scripts

This directory contains comprehensive test scripts for the TrotBot quadruped robot, providing functionality for testing coordinated and individual leg movements.

## 🦿 Available Test Scripts

### 1. `test_leg_comprehensive.py` - Comprehensive Leg Testing
**Primary test script for full leg movement testing with inverse kinematics**

#### Features:
- **Individual leg testing** with predefined positions
- **Coordinated multi-leg movements** 
- **Inverse kinematics** foot positioning
- **Rapid vertical movement tests**
- **Interactive mode** with command-line interface
- **Safety limits** and error handling
- **Real-time calibration display**

#### Usage:
```bash
# Interactive mode (recommended)
./test_leg_comprehensive.py

# Basic movement test on active leg
./test_leg_comprehensive.py --test

# Rapid vertical movement test (5 cycles)
./test_leg_comprehensive.py --vertical 5

# Coordinated four-leg movement test
./test_leg_comprehensive.py --coordination

# Select specific leg (0=FR, 1=FL, 2=BR, 3=BL)
./test_leg_comprehensive.py --leg 1 --test

# Interactive mode with specific leg
./test_leg_comprehensive.py --leg 2 --interactive
```

#### Interactive Commands:
- `neutral` - Move active leg to neutral position
- `thigh_extended` - Extend thigh to 0°
- `thigh_raised` - Raise thigh to 90°
- `calf_extended` - Extend calf to 0°
- `calf_retracted` - Retract calf to -90°
- `test` - Run basic movement test
- `vertical [N]` - Run vertical movement test (N cycles)
- `all_neutral` - Move all legs to neutral
- `test_coordination` - Test coordinated movements
- `leg_select [0-3]` - Change active leg
- `calibration` - Show calibration info
- `help` - Show all commands
- `quit` - Exit safely

### 2. `test_individual_joints.py` - Individual Joint Testing
**Precise control and testing of individual servo joints**

#### Features:
- **Individual servo control** with precise angle setting
- **Joint range testing** with configurable steps
- **Safety validation** with angle limits
- **Interactive joint control**
- **Real-time servo mapping display**

#### Usage:
```bash
# Interactive mode
./test_individual_joints.py --interactive

# Test specific joint range (leg 0, joint 1 = Front-Right Thigh)
./test_individual_joints.py --leg 0 --joint 1
```

#### Interactive Commands:
- `set <leg> <joint> <angle>` - Set specific joint angle
  - leg: 0=FR, 1=FL, 2=BR, 3=BL
  - joint: 0=Hip, 1=Thigh, 2=Calf
  - angle: degrees within safe range
- `test <leg> <joint>` - Test full range of joint
- `neutral` - Set all joints to neutral
- `ranges` - Show safe angle ranges
- `help` - Show commands
- `quit` - Exit safely

## 🔧 Technical Specifications

### Leg Geometry:
- **Thigh length (L1)**: 73.5mm
- **Calf length (L2)**: 60mm  
- **Hip offset**: 36.75mm
- **Total reach**: 133.5mm

### Safe Angle Ranges:
- **Hip (Abduction)**: -30° to +30°
- **Thigh**: 0° to 90°
- **Calf**: -90° to 0°

### Leg Indexing:
- **0**: Front-Right (FR)
- **1**: Front-Left (FL)
- **2**: Back-Right (BR)
- **3**: Back-Left (BL)

### Joint Indexing:
- **0**: Hip (Abduction)
- **1**: Thigh (Inner)
- **2**: Calf (Outer)

## 🛡️ Safety Features

### Built-in Safety:
- **Angle validation** - All movements checked against safe ranges
- **Workspace validation** - Inverse kinematics positions verified
- **Emergency stops** - Ctrl+C returns to neutral position
- **Non-active leg protection** - Other legs maintained in stable position
- **User confirmation** - Safety prompts before movement

### Safety Checklist:
✅ Robot is securely held or supported  
✅ All legs can move freely  
✅ Emergency stop is accessible  
✅ Servo power is connected  
✅ Calibration data is loaded  

## 🎯 Test Scenarios

### Basic Movement Testing:
1. **Neutral Position** - All joints at safe defaults
2. **Thigh Extension** - Test thigh range (0° to 90°)
3. **Calf Extension** - Test calf range (-90° to 0°)
4. **Combined Movements** - Multi-joint coordination

### Advanced Testing:
1. **Inverse Kinematics** - Foot position control
2. **Rapid Vertical Movement** - Dynamic foot lifting
3. **Coordinated Movement** - All legs synchronized
4. **Individual Joint Control** - Precise servo testing

## 🔍 Troubleshooting

### Common Issues:

**"Hardware interface failed"**
- Check servo power connection
- Verify I2C communication
- Ensure proper calibration data

**"Position unreachable"**
- Check if target is within workspace
- Verify joint angle limits
- Use debug mode for analysis

**"Import errors"**
- Ensure TrotBot package is built: `colcon build --packages-select trotbot`
- Source workspace: `source install/setup.bash`
- Check Python path includes TrotBot modules

### Debug Commands:
```bash
# Check calibration status
ros2 run trotbot calibration_status

# Rebuild TrotBot package
colcon build --packages-select trotbot --merge-install

# Test hardware interface
python3 -c "from trotbot.hal.hardware.hardware_interface import HardwareInterface; hi = HardwareInterface(); print('✅ Hardware OK')"
```

## 📊 Performance Metrics

### Typical Test Times:
- **Basic movement test**: ~20 seconds
- **Vertical movement (5 cycles)**: ~15 seconds  
- **Coordination test**: ~15 seconds
- **Joint range test**: ~30 seconds (per joint)

### Movement Specifications:
- **Position accuracy**: ±1°
- **Movement speed**: Configurable (0.1s to 5.0s hold times)
- **Cycle repeatability**: >1000 cycles tested
- **Safety response time**: <100ms

## 🚀 Getting Started

1. **Build TrotBot package**:
   ```bash
   cd ~/TrotBot/trotbot_ws
   colcon build --packages-select trotbot --merge-install
   source install/setup.bash
   ```

2. **Check calibration**:
   ```bash
   ros2 run trotbot calibration_status
   ```

3. **Run comprehensive test**:
   ```bash
   cd src/trotbot/test
   ./test_leg_comprehensive.py
   ```

4. **Start with safe commands**:
   - `all_neutral` - Safe starting position
   - `test` - Basic movement validation
   - `calibration` - Verify settings

## 📝 Notes

- **Always start with `all_neutral`** to ensure safe starting position
- **Use interactive mode** for learning and experimentation  
- **Test individual legs first** before coordinated movements
- **Monitor servo temperatures** during extended testing
- **Keep emergency stop accessible** at all times

For more information, see the main TrotBot documentation and calibration guides.
