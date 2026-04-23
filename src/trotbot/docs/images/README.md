# TrotBot Documentation Images

This directory contains placeholder image files for the TrotBot project documentation. These images are referenced in the main README.md file and should be replaced with actual diagrams and screenshots.

## Image Files and Descriptions

### Main Project Image
- **`trotbot.png`** - Main TrotBot robot image for README header display

### Basic Quadruped Locomotion
- **`ros_graph_basic.png`** - ROS2 node graph showing basic locomotion architecture
- **`tf_tree_kinematics.png`** - TF tree diagram showing coordinate frame relationships

### Enhanced Teleop Control System  
- **`ros_graph_teleop.png`** - ROS2 node graph for enhanced teleop control
- **`control_flow_gamepad.png`** - Gamepad input to servo output control flow diagram

### Hardware Abstraction Layer (HAL)
- **`hal_architecture.png`** - HAL architecture showing dual backend system
- **`servo_control_pipeline.png`** - Joint angles to PWM signal conversion pipeline

### Inverse Kinematics & Leg Geometry
- **`leg_geometry.png`** - Leg geometry diagram showing configurable segment lengths
- **`workspace_analysis.png`** - Workspace analysis showing reachable positions and limits
- **`ik_algorithm_flow.png`** - Inverse kinematics algorithm computation pipeline

### Servo Control & Calibration
- **`pwm_control_architecture.png`** - PCA9685 to servo signal distribution architecture
- **`calibration_workflow.png`** - Servo calibration workflow from joint angles to PWM

### Gazebo Simulation Support
- **`simulation_architecture.png`** - Gazebo integration with ROS2 control architecture
- **`control_interface_unified.png`** - Unified control interface for simulation and hardware

### System Architecture
- **`complete_system_architecture.png`** - End-to-end control pipeline architecture
- **`data_flow_diagram.png`** - Message flow from input to hardware
- **`layered_architecture.png`** - System abstraction levels and interfaces

### CAD Models and Mechanical Design
- **`robot_assembly.png`** - Complete TrotBot assembly with leg configuration
- **`leg_mechanism.png`** - Detailed leg mechanism with joint arrangements
- **`frame_structure.png`** - Main frame structure with mounting points
- **`power_distribution.png`** - Power system layout and wiring diagram

## Creating Actual Images

To replace these placeholder files with actual diagrams:

1. **ROS2 Graphs**: Use `rqt_graph` to generate node connectivity diagrams
2. **TF Trees**: Use `rqt_tf_tree` or `view_frames` to generate coordinate frame diagrams  
3. **Architecture Diagrams**: Create using tools like:
   - Draw.io / Lucidchart for system architecture
   - PlantUML for sequence diagrams
   - Graphviz for data flow diagrams
   - CAD software for mechanical diagrams

## Recommended Image Specifications

- **Format**: PNG (preferred) or SVG for scalability
- **Resolution**: Minimum 800x600 pixels for clarity
- **Background**: White or transparent for consistency
- **Text**: Readable font size (minimum 12pt)
- **Colors**: Use consistent color scheme throughout documentation

## File Naming Convention

- Use lowercase with underscores: `system_architecture.png`
- Be descriptive: `leg_geometry_73mm_segments.png`
- Include version if needed: `control_flow_v2.png`

---

**Note**: These are currently empty placeholder files. Replace them with actual diagrams as the project documentation is developed.
