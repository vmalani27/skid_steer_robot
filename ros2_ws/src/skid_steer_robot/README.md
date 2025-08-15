# Skid Steer Robot Package

A ROS2 Humble package for a four-wheeled skid steer robot that can be simulated in Gazebo Fortress.

## Features

- Four-wheeled skid steer robot with differential drive control
- Compatible with ROS2 Humble and Gazebo Fortress
- Keyboard teleoperation support
- Proper physics simulation with collision detection
- ROS2 control integration

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble (Hawksbill)
- Gazebo Fortress
- ros_gz package

## Installation

### 1. Install ROS2 Humble
```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Install Gazebo Fortress
```bash
# Add Gazebo repository
sudo apt update && sudo apt install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress
sudo apt update
sudo apt install gazebo-fortress
```

### 3. Install ros_gz
```bash
# Install ros_gz package
sudo apt install ros-humble-ros-gz
```

### 4. Install Additional Dependencies
```bash
# Install required ROS2 packages
sudo apt install ros-humble-controller-manager \
                 ros-humble-diff-drive-controller \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-gazebo-ros2-control

# Install build tools
sudo apt install build-essential cmake
```

## Building the Package

1. Navigate to your ROS2 workspace:
```bash
cd ~/ros2_ws
```

2. Build the package:
```bash
colcon build --packages-select skid_steer_robot
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch the Simulation

Use the simple simulation launch file (recommended):
```bash
ros2 launch skid_steer_robot simple_sim.launch.py
```

Or use the full simulation launch file:
```bash
ros2 launch skid_steer_robot sim.launch.py
```

### Control the Robot

Once the simulation is running, you can control the robot using the keyboard:

- **i**: Move forward
- **,**: Move backward  
- **j**: Turn left
- **l**: Turn right
- **k**: Stop
- **Shift + J**: Turn left faster
- **Shift + L**: Turn right faster

### View Robot State

In another terminal, you can monitor the robot's state:
```bash
# View joint states
ros2 topic echo /joint_states

# View odometry
ros2 topic echo /odom

# View TF tree
ros2 run tf2_tools view_frames
```

## Package Structure

```
skid_steer_robot/
├── config/
│   └── ros2_control.yaml      # Controller configuration
├── launch/
│   ├── sim.launch.py          # Full simulation launch
│   └── simple_sim.launch.py   # Simple simulation launch
├── urdf/
│   ├── skid_steer.urdf       # Robot description
│   └── skid_steer.urdf.xacro # Parameterized robot description
├── meshes/                    # 3D mesh files (if any)
├── worlds/                    # World files
├── model.sdf                  # Gazebo model file
├── model.config               # Model configuration
├── world.sdf                  # World file with robot
├── package.xml                # Package manifest
├── CMakeLists.txt             # Build configuration
└── README.md                  # This file
```

## Configuration

### Robot Parameters

The robot's physical parameters can be modified in `urdf/skid_steer.urdf.xacro`:
- `wheel_radius`: 0.05 meters
- `wheel_width`: 0.02 meters  
- `wheel_separation`: 0.3 meters

### Controller Parameters

Controller settings can be adjusted in `config/ros2_control.yaml`:
- Update rate: 50 Hz
- Maximum linear velocity: 2.0 m/s
- Maximum angular velocity: 3.0 rad/s

## Troubleshooting

### Common Issues

1. **Gazebo not starting**: Make sure Gazebo Fortress is properly installed
2. **Controller errors**: Check that all ROS2 control packages are installed
3. **Robot not moving**: Verify the ros2_control plugin is loaded correctly

### Debug Commands

```bash
# Check if Gazebo is running
gz topic -l

# Check ROS2 topics
ros2 topic list

# Check controller status
ros2 control list_controllers

# Check robot description
ros2 topic echo /robot_description
```

## Contributing

Feel free to submit issues and enhancement requests!

## License

This package is licensed under the Apache License 2.0.
