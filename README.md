# Gravis - ROS2 Differential Drive Robot

This is a ROS2 package for a differential drive robot called "Gravis". The robot is equipped with a LiDAR sensor, camera, and can be controlled via joystick or navigation commands. It supports both simulation (Gazebo) and real robot deployment.

## Features

- Differential drive robot with ROS2 control
- LiDAR sensor for navigation and mapping
- Camera for computer vision tasks
- Ball tracking capability
- Navigation stack integration (Nav2)
- Joystick/gamepad control
- Both simulation and real robot support

## Prerequisites

- ROS2 (tested with ROS2 Humble)
- Gazebo (for simulation)
- colcon (for building)

### Required ROS2 packages
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-joint-broad-controller
sudo apt install ros-humble-joy
sudo apt install ros-humble-teleop-twist-joy
```

## Installation

### Workspace Setup

**Important**: Make sure your ROS2 workspace has the correct structure. The package must be in a `src` directory:

```bash
~/ros2_ws/
├── src/
│   └── articubot_one/  # <-- Package goes here (Gravis robot)
├── build/
├── install/
└── log/
```

### Installation Steps

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

3. Install dependencies and update ROS2 repositories:
```bash
# Update ROS2 GPG key if you encounter repository errors
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key

# Update package lists
sudo apt update

# Install all required packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-joint-broad-controller
sudo apt install ros-humble-joy
sudo apt install ros-humble-teleop-twist-joy
```

4. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select articubot_one
```

5. Source the workspace:
```bash
# Source ROS2 base installation first
source /opt/ros/humble/setup.bash
# Then source your workspace
source install/setup.bash
```

**Note**: If you have other ROS2 workspaces sourced in your `~/.bashrc`, you may need to open a new terminal or manually source the correct workspace to avoid conflicts.

## Usage

### Running the Simulation

To launch the robot in Gazebo simulation:

```bash
ros2 launch articubot_one launch_sim.launch.py
```

This will start:
- Gazebo with the robot model
- Robot state publisher
- Joint state publisher
- Differential drive controller
- Joystick control (if connected)
- Twist mux for command arbitration

### Controlling the Robot

#### 1. Joystick Control
If you have a gamepad/joystick connected, you can control the robot directly. The joystick launch file is automatically included in the simulation.

#### 2. Keyboard Control
You can use the `teleop_twist_keyboard` package:
```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_joy
```

#### 3. Navigation Control
For autonomous navigation, first start the simulation, then launch navigation:
```bash
# Terminal 1: Start simulation
ros2 launch articubot_one launch_sim.launch.py

# Terminal 2: Start navigation
ros2 launch articubot_one navigation_launch.py use_sim_time:=true

# Terminal 3: Start localization (if needed)
ros2 launch articubot_one localization_launch.py use_sim_time:=true
```

#### 4. Ball Tracking
To enable ball tracking capability:
```bash
# For simulation
ros2 launch articubot_one ball_tracker.launch.py sim_mode:=true

# For real robot
ros2 launch articubot_one ball_tracker.launch.py sim_mode:=false
```

### Running on Real Robot

To run on the actual hardware:

```bash
ros2 launch articubot_one launch_robot.launch.py
```

This configures the system for real robot operation (disables sim_time, etc.).

### Additional Launch Files

- `rsp.launch.py` - Robot state publisher only
- `joystick.launch.py` - Joystick control only
- `camera.launch.py` - Camera functionality
- `rplidar.launch.py` - LiDAR sensor
- `online_async_launch.py` - SLAM mapping

## Project Structure

```
articubot_one/ (Gravis Robot Package)
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS2 package metadata
├── README.md                   # This file
├── config/                     # Configuration files
│   ├── ball_tracker_params_*.yaml    # Ball tracking parameters
│   ├── drive_bot.rviz               # RViz configuration for driving
│   ├── main.rviz                    # Main RViz configuration
│   ├── my_controllers.yaml          # Robot controller configuration
│   ├── nav2_params.yaml             # Navigation parameters
│   ├── joystick.yaml                # Joystick configuration
│   └── twist_mux.yaml               # Command multiplexer configuration
├── description/                # Robot description files (URDF/Xacro)
│   ├── robot.urdf.xacro            # Main robot description
│   ├── robot_core.xacro            # Core robot geometry
│   ├── ros2_control.xacro          # ROS2 control configuration
│   ├── gazebo_control.xacro        # Gazebo-specific control
│   ├── lidar.xacro                 # LiDAR sensor description
│   ├── camera.xacro                # Camera sensor description
│   └── face.xacro                  # Robot face/display
├── launch/                     # Launch files
│   ├── launch_sim.launch.py        # Main simulation launcher
│   ├── launch_robot.launch.py      # Real robot launcher
│   ├── navigation_launch.py        # Navigation stack
│   ├── ball_tracker.launch.py     # Ball tracking
│   └── *.launch.py                 # Other specific launchers
└── worlds/                     # Gazebo world files
    ├── empty.world                 # Empty environment
    └── obstacles.world             # Environment with obstacles
```

### Key Components Explained

1. **Robot Description (`description/`)**: Contains URDF/Xacro files that define the robot's physical structure, sensors, and control interfaces.

2. **Launch Files (`launch/`)**: Python scripts that start multiple ROS2 nodes and configure the robot system for different use cases.

3. **Configuration (`config/`)**: YAML files containing parameters for various components like controllers, navigation, and sensors.

4. **Worlds (`worlds/`)**: Gazebo simulation environments where the robot can operate.

### Robot Specifications

- **Base**: Differential drive with two main wheels and caster wheels
- **Dimensions**: ~33.5cm length × 26.5cm width × 13.8cm height
- **Sensors**: 
  - RPLiDAR for 2D scanning
  - Camera for computer vision
  - Optional depth camera support
- **Control**: ROS2 Control framework with differential drive controller

## Development

This package uses the standard ROS2/ament_cmake build system. To modify the robot:

1. **Change robot geometry**: Edit files in `description/`
2. **Add sensors**: Create new `.xacro` files and include them in `robot.urdf.xacro`
3. **Modify control**: Update `config/my_controllers.yaml` and control xacro files
4. **Adjust navigation**: Modify `config/nav2_params.yaml`

## Troubleshooting

### Common Issues

- **Robot not moving**: Check that controllers are loaded with `ros2 control list_controllers`
- **No camera image**: Verify camera topic with `ros2 topic list | grep camera`
- **Navigation issues**: Ensure map is loaded and localization is working
- **Joystick not working**: Check device permissions and that joy_node is publishing

### Installation Issues

- **`spawner.py` not found error**: 
  ```
  [ERROR] [launch]: executable 'spawner.py' not found on the libexec directory '/opt/ros/humble/lib/controller_manager'
  ```
  This happens when the launch files reference `spawner.py` but the actual executable is named `spawner`. The launch files in this repository have been updated to use the correct executable name.

- **Package not found after building**:
  - Ensure your workspace has the correct structure with the package in `src/articubot_one/` (Gravis robot package)
  - Make sure you've sourced the workspace: `source install/setup.bash`
  - If you have multiple ROS2 workspaces, make sure you're sourcing the correct one
  - Try opening a new terminal to avoid environment conflicts

- **Repository/GPG key errors**:
  ```bash
  # Update ROS2 GPG key
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
  sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key
  sudo apt update
  ```

- **Missing controller packages**:
  ```bash
  sudo apt install ros-humble-controller-manager ros-humble-ros2-controllers ros-humble-ros2-control
  ```

## Contributing

This Gravis robot project is based on Josh Newans' robot tutorials and has been adapted with improvements and fixes. Feel free to submit issues and enhancement requests!