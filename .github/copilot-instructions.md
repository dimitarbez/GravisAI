# Copilot Instructions for Gravis Robot (articubot_one)

## Project Overview
This is "Gravis" - a ROS2 differential drive robot with LiDAR, camera, and navigation capabilities. The package name is `articubot_one` but the robot entity is named "gravis" in Gazebo.

## Architecture & Key Concepts

### Launch System Architecture
- **Primary entry points**: `launch_sim.launch.py` (simulation) and `launch_robot.launch.py` (real hardware)
- **Modular launch files**: Each subsystem has its own launch file (joystick, camera, navigation, etc.)
- **Dual mode support**: Uses `sim_mode` and `use_sim_time` parameters to switch between simulation and real robot
- **Robot spawning**: Entity named "gravis" in Gazebo, controlled via `spawn_entity.py`

### Control System Flow
1. **Command prioritization**: `twist_mux` prioritizes commands: joystick (100) > tracker (20) > navigation (10)
2. **Controller chain**: cmd_vel → twist_mux → `/diff_cont/cmd_vel_unstamped` → diff_drive_controller
3. **Hardware abstraction**: `ros2_control.xacro` switches between Arduino hardware (`diffdrive_arduino/DiffDriveArduino`) and Gazebo simulation

### URDF/Xacro Structure
- **Main robot file**: `robot.urdf.xacro` conditionally includes components based on `use_ros2_control` argument
- **Modular design**: `robot_core.xacro` (base structure), `lidar.xacro`, `camera.xacro`, `face.xacro`
- **Control switching**: Automatically chooses between `ros2_control.xacro` (preferred) and `gazebo_control.xacro` (legacy)

## Critical Patterns & Conventions

### Launch File Patterns
```python
# Standard package name reference
package_name='articubot_one'  # Always use this pattern

# Include other launch files with parameters
rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory(package_name),'launch','rsp.launch.py'
    )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
)
```

### Controller Management
- **Two main controllers**: `diff_cont` (differential drive) and `joint_broad` (joint state broadcaster)
- **Spawning order**: Controllers spawn after robot entity using `spawner` executable (not `spawner.py`)
- **Update rates**: Controller manager at 30Hz, diff_cont publishes at 50Hz

### Configuration Patterns
- **Simulation vs Real**: Separate param files with `_sim` and `_robot` suffixes (e.g., `ball_tracker_params_sim.yaml`)
- **Conditional parameters**: Use `PythonExpression` in launch files to switch config files based on `sim_mode`

## Development Workflows

### Building & Testing
```bash
# Always build from workspace root
cd ~/ros2_ws
colcon build --packages-select articubot_one
source install/setup.bash

# Launch simulation (most common development task)
ros2 launch articubot_one launch_sim.launch.py
```

### Controller Debugging
```bash
# Check controller status
ros2 control list_controllers

# Manual controller loading if needed
ros2 run controller_manager spawner diff_cont
ros2 run controller_manager spawner joint_broad
```

### Navigation Stack Integration
- **Separate launch files**: Navigation requires its own launch (`navigation_launch.py`) after simulation is running
- **SLAM mapping**: Use `online_async_launch.py` for simultaneous localization and mapping
- **Parameter file**: All Nav2 config in `nav2_params.yaml`

## Integration Points

### External Dependencies
- **Ball tracking**: Depends on separate `ball_tracker` package with different param files for sim/real
- **Navigation**: Uses standard Nav2 stack with custom parameter tuning
- **Hardware interface**: Real robot expects Arduino on `/dev/ttyUSB0` at 57600 baud with 3436 encoder counts/rev

### Topic Remapping Strategy
- **Command multiplexing**: Multiple cmd_vel sources → twist_mux → controller
- **Namespace consistency**: Camera topics under `/camera/`, LiDAR typically `/scan`
- **Controller topics**: Always remap to `/diff_cont/cmd_vel_unstamped` for final control

### Cross-Component Communication
- **State publishing**: Robot state publisher generates TF tree from URDF
- **Sensor integration**: LiDAR and camera defined in separate xacro files, included conditionally
- **Visualization**: Multiple RViz configs for different scenarios (`drive_bot.rviz`, `main.rviz`, `view_bot.rviz`)

## Key Files to Understand
- `launch_sim.launch.py`: Main simulation orchestrator
- `robot.urdf.xacro`: Robot structure with conditional includes
- `my_controllers.yaml`: Controller configuration with wheel parameters
- `twist_mux.yaml`: Command prioritization logic
- `ros2_control.xacro`: Hardware abstraction layer

When modifying the robot, always consider both simulation and real hardware modes, and maintain the modular launch file structure.
