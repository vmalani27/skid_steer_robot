# Differential Drive Robot (Skid Steer) ROS2 Workspace

This workspace contains a complete ROS2 package suite for a differential drive (skid steer) robot with ultrasonic sensors and basic navigation capabilities.

## Package Structure

```
differential_drive_ws/
├── src/
│   ├── diffbot_description/     # URDF/Xacro + meshes
│   ├── diffbot_bringup/         # Launch files
│   ├── diffbot_control/         # ros2_control + hardware interface
│   ├── diffbot_sensors/         # Ultrasonic sensor nodes
│   ├── diffbot_navigation/      # Basic nav + obstacle avoidance logic
│   └── diffbot_msgs/            # Custom messages
└── build.sh                    # Build script
```

## Packages Overview

### diffbot_description
- Contains URDF/Xacro robot description files
- Includes Gazebo simulation configurations
- Robot visualization and state publishing

### diffbot_bringup
- Main launch files for robot startup
- Gazebo simulation launch files
- System integration scripts

### diffbot_control
- ros2_control configuration and controllers
- Differential drive controller setup
- Hardware interface definitions
- **Motor driver for Cytron MDDRC10** (Raspberry Pi GPIO)

### diffbot_sensors
- Ultrasonic sensor node implementations
- Sensor data aggregation and processing
- Obstacle detection logic

### diffbot_navigation
- Basic navigation behaviors (wandering, obstacle avoidance)
- Path planning and execution
- Autonomous movement logic

### diffbot_msgs
- Custom message definitions for sensor data
- Service definitions for robot control
- Robot status and navigation messages

## Building the Workspace

1. **Install dependencies first:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Build all packages:**
   ```bash
   ./build.sh
   ```

3. **Clean and rebuild:**
   ```bash
   ./build.sh clean
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Running the Robot

### 1. Basic Robot Visualization
```bash
# Launch robot description and RViz
ros2 launch diffbot_description display.launch.py
```

### 2. Gazebo Simulation
```bash
# Launch robot in Gazebo
ros2 launch diffbot_bringup gazebo.launch.py

# In another terminal, start navigation
ros2 launch diffbot_navigation navigation.launch.py
```

### 3. Real Robot (Hardware)
```bash
# Launch robot hardware interface with motor driver
ros2 launch diffbot_bringup hardware_bringup.launch.py use_hardware:=true

# Start sensors
ros2 launch diffbot_sensors sensors.launch.py

# Start navigation
ros2 launch diffbot_navigation navigation.launch.py
```

### 4. Motor Driver Only (Hardware Testing)
```bash
# Test motor driver directly
ros2 launch diffbot_control motor_driver.launch.py

# Send test commands
ros2 topic pub /diffbot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### 5. Sensor Testing (Hardware)
```bash
# Test individual ultrasonic sensor
ros2 run diffbot_sensors ultrasonic_sensor

# Test all sensors with monitoring
ros2 run diffbot_sensors ultrasonic_test

# Test sensors with launch file
ros2 launch diffbot_sensors ultrasonic_sensors.launch.py

# Monitor sensor topics
ros2 topic echo /ultrasonic_front/range
```

## Hardware Configuration

### Motor Driver (Cytron MDDRC10)
The robot uses a Cytron MDDRC10 motor driver controller connected to a Raspberry Pi:

**GPIO Connections:**
- Pin 18 (GPIO 24): Left motor group (RC1)
- Pin 19 (GPIO 25): Right motor group (RC2)

**Features:**
- PWM-based speed control (50Hz)
- Safety timeout (stops motors if no command received)
- Configurable velocity limits
- Mock GPIO for development/simulation

**Configuration:**
Edit `/config/motor_driver.yaml` to adjust parameters:
```yaml
motor_driver:
  ros__parameters:
    rc1_pin: 18                    # Left motor GPIO pin
    rc2_pin: 19                    # Right motor GPIO pin
    pwm_frequency: 50              # PWM frequency (Hz)
    max_linear_velocity: 1.0       # Max linear speed (m/s)
    max_angular_velocity: 1.0      # Max angular speed (rad/s)
```

### Ultrasonic Sensors (HC-SR04)
The robot uses a single HC-SR04 ultrasonic sensor on the front for obstacle detection, integrated with `lgpio` library:

**GPIO Connections:**
- Front sensor: TRIG=23, ECHO=24

**Features:**
- Range: 2cm to 4m
- Publishing rate: 10Hz (configurable)
- Automatic timeout handling (20ms)
- Mock GPIO for development/simulation
- Easy to expand to multiple sensors later

**Installation Requirements:**
```bash
# Install lgpio library on Raspberry Pi
sudo apt update
sudo apt install python3-lgpio
# OR
pip3 install lgpio
```

**Running Sensors:**
```bash
# Single front sensor (recommended)
ros2 run diffbot_sensors ultrasonic_sensor --ros-args -p sensor_name:=ultrasonic_front -p trig_pin:=23 -p echo_pin:=24

# Or use the multi-sensor node (configured for front only)
ros2 run diffbot_sensors multi_ultrasonic

# Launch sensor with aggregator
ros2 launch diffbot_sensors ultrasonic_sensors.launch.py

# Test sensor readings
ros2 run diffbot_sensors ultrasonic_test
```

**Topics Published:**
- `/ultrasonic_front/range` - Front sensor data

**Configuration:**
```yaml
ultrasonic_sensor:
  ros__parameters:
    sensor_name: 'ultrasonic_front'
    frame_id: 'ultrasonic_front_link'
    trig_pin: 23                 # TRIG pin (BCM)
    echo_pin: 24                 # ECHO pin (BCM)
    min_range: 0.02              # Minimum range (2cm)
    max_range: 4.0               # Maximum range (4m)  
    publish_rate: 10.0           # Publishing frequency (Hz)
    timeout_ms: 20               # Sensor timeout (ms)
```

## Robot Control

### Manual Control
```bash
# Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot/cmd_vel
```

### Navigation Behaviors
```bash
# Obstacle avoidance mode
ros2 launch diffbot_navigation navigation.launch.py behavior_mode:=obstacle_avoidance

# Wandering mode
ros2 launch diffbot_navigation navigation.launch.py behavior_mode:=wander
```

### Hardware Testing
```bash
# Test motor driver functionality
ros2 run diffbot_control motor_tester

# Manual motor control
ros2 topic pub /diffbot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## Monitoring

### Topics
- `/diffbot/cmd_vel` - Velocity commands
- `/diffbot/odom` - Odometry data
- `/diffbot/scan` - Laser scan data
- `/ultrasonic_*/range` - Ultrasonic sensor data
- `/robot_description` - Robot model description

### Services
- `/set_navigation_behavior` - Change navigation mode
- `/emergency_stop` - Emergency stop functionality

## Robot Specifications

### Physical Properties
- Base dimensions: 42cm x 31cm x 18cm
- Wheel diameter: 20cm
- Wheel separation: 28.7cm
- Max speed: 1.0 m/s linear, 1.0 rad/s angular

### Sensors
- 1x Ultrasonic sensor (front) - HC-SR04
- 1x Laser scanner (optional)
- Range: 2cm - 4m (ultrasonic)
- Expandable to multiple sensors

### Actuators
- 2x Drive wheels (continuous rotation)
- 1x Caster wheel (passive)

## Dependencies

This workspace requires the following ROS2 packages:
- `robot_state_publisher`
- `joint_state_publisher`
- `diff_drive_controller`
- `controller_manager`
- `gazebo_ros`
- `xacro`
- `rviz2`

## Installation

1. **Install ROS2 Humble** (if not already installed)
2. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. **For Raspberry Pi (hardware), install GPIO library:**
   ```bash
   sudo apt install python3-rpi.gpio
   ```
4. **Build the workspace:**
   ```bash
   ./build.sh
   ```

## Troubleshooting

### Build Issues
- **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` first
- **Clean and rebuild**: `./build.sh clean`
- **Check ROS2 is sourced**: `source /opt/ros/humble/setup.bash`
- **Install missing packages**: `sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-gazebo-ros2-control`

### Runtime Issues
- Verify all nodes are running: `ros2 node list`
- Check topic connections: `ros2 topic list`
- Monitor logs: `ros2 log view`

### Simulation Issues
- Ensure Gazebo is properly installed
- Check world file paths in launch files
- Verify robot spawns correctly in Gazebo

## Contributing

1. Follow ROS2 coding standards
2. Test changes in simulation before hardware
3. Update documentation for new features
4. Add appropriate error handling and logging

## License

This project is licensed under the MIT License.