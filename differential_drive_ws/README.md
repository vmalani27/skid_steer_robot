# Front Ultrasonic Sensor - Skid Steer Robot# Front Ultrasonic Sensor - Skid Steer Robot



This workspace contains a simplified ROS2 package for a front ultrasonic sensor node designed for obstacle detection on a differential drive robot.This workspace contains a simplified ROS2 package for a front ultrasonic sensor node designed for obstacle detection on a differential drive robot.



## Package Structure## Package Structure



``````

differential_drive_ws/differential_drive_ws/

├── src/├── src/

│   └── diffbot_sensors/         # Front ultrasonic sensor package│   └── diffbot_sensors/         # Front ultrasonic sensor package

│       ├── package.xml          # Package dependencies│       ├── package.xml          # Package dependencies

│       ├── setup.py            # Python package setup│       ├── setup.py            # Python package setup

│       ├── launch/             # Launch files│       ├── launch/             # Launch files

│       │   └── front_sensor.launch.py│       │   └── front_sensor.launch.py

│       ├── config/             # Configuration files│       ├── config/             # Configuration files

│       │   └── front_sensor.yaml│       │   └── front_sensor.yaml

│       ├── src/                # Executable nodes│       ├── src/                # Executable nodes

│       │   └── front_ultrasonic_node.py│       │   └── front_ultrasonic_node.py

│       └── diffbot_sensors/    # Python package module│       └── diffbot_sensors/    # Python package module

│           └── __init__.py│           └── __init__.py

└── build.sh                   # Build script└── build.sh                   # Build script

``````



## Hardware Requirements## Hardware Requirements



### HC-SR04 Ultrasonic Sensor### HC-SR04 Ultrasonic Sensor

**GPIO Connections (Raspberry Pi):****GPIO Connections (Raspberry Pi):**

- **TRIG Pin**: GPIO 23 (BCM numbering)- **TRIG Pin**: GPIO 23 (BCM numbering)

- **ECHO Pin**: GPIO 24 (BCM numbering)- **ECHO Pin**: GPIO 24 (BCM numbering)

- **VCC**: 5V power supply- **VCC**: 5V power supply

- **GND**: Ground- **GND**: Ground



### Software Requirements### Software Requirements

- ROS2 Humble- ROS2 Humble

- lgpio library for GPIO control- lgpio library for GPIO control

- Python 3.8+- Python 3.8+



## Installation## Installation



1. **Install lgpio library:**1. **Install lgpio library:**

   ```bash   ```bash

   sudo apt update   sudo apt update

   sudo apt install python3-lgpio   sudo apt install python3-lgpio

   ```   ```



2. **Build the workspace:**2. **Build the workspace:**

   ```bash   ```bash

   cd /home/rovasura/skid_steer_robot/differential_drive_ws   cd /home/rovasura/skid_steer_robot/differential_drive_ws

   colcon build --packages-select diffbot_sensors   colcon build --packages-select diffbot_sensors

   source install/setup.bash   source install/setup.bash

   ```   ```



## Usage## Usage



### 1. Running the Front Ultrasonic Sensor### 1. Running the Front Ultrasonic Sensor



**Direct node execution:****Direct node execution:**

```bash```bash

# Run the front ultrasonic sensor node# Run the front ultrasonic sensor node

ros2 run diffbot_sensors front_ultrasonic_node.pyros2 run diffbot_sensors front_ultrasonic_node.py

``````



**Using launch file:****Using launch file:**

```bash```bash

# Launch with default configuration# Launch with default configuration

ros2 launch diffbot_sensors front_sensor.launch.pyros2 launch diffbot_sensors front_sensor.launch.py



# Launch with custom rate# Launch with custom rate

ros2 launch diffbot_sensors front_sensor.launch.py rate:=20.0ros2 launch diffbot_sensors front_sensor.launch.py rate:=20.0

``````



### 2. Monitoring Sensor Data### 2. Monitoring Sensor Data



**View sensor readings:****View sensor readings:**

```bash```bash

# Monitor distance readings# Monitor distance readings

ros2 topic echo /ultrasonic_front/rangeros2 topic echo /ultrasonic_front/range



# View topics list# View topics list

ros2 topic list | grep ultrasonicros2 topic list | grep ultrasonic



# Check node info# Check node info

ros2 node info /front_ultrasonic_noderos2 node info /front_ultrasonic_node

``````



**Real-time visualization:****Real-time visualization:**

```bash```bash

# Plot distance over time# Plot distance over time

ros2 run plotjuggler plotjugglerros2 run plotjuggler plotjuggler



# Then subscribe to /ultrasonic_front/range topic# Then subscribe to /ultrasonic_front/range topic

``````



### 3. Testing and Debugging### 3. Testing and Debugging



**Check GPIO status:****Check GPIO status:**

```bash```bash

# View detailed logs# View detailed logs

ros2 run diffbot_sensors front_ultrasonic_node.py --ros-args --log-level DEBUGros2 run diffbot_sensors front_ultrasonic_node.py --ros-args --log-level DEBUG

``````



**Simulation mode (no GPIO):****Simulation mode (no GPIO):**

- The node automatically falls back to simulation mode if GPIO is unavailable- The node automatically falls back to simulation mode if GPIO is unavailable

- Generates random distance readings for testing- Generates random distance readings for testing

- Useful for development without hardware- Useful for development without hardware



## Configuration## Configuration



### Sensor Parameters### Sensor Parameters

Edit `config/front_sensor.yaml` to customize sensor behavior:Edit `config/front_sensor.yaml` to customize sensor behavior:



```yaml```yaml

front_ultrasonic_node:front_ultrasonic_node:

  ros__parameters:  ros__parameters:

    sensor_name: "ultrasonic_front"     # Sensor identifier    sensor_name: "ultrasonic_front"     # Sensor identifier

    frame_id: "ultrasonic_front"        # TF frame name    frame_id: "ultrasonic_front"        # TF frame name

    trig_pin: 23                        # GPIO TRIG pin (BCM)    trig_pin: 23                        # GPIO TRIG pin (BCM)

    echo_pin: 24                        # GPIO ECHO pin (BCM)    echo_pin: 24                        # GPIO ECHO pin (BCM)

    min_range: 0.02                     # Minimum range (2cm)    min_range: 0.02                     # Minimum range (2cm)

    max_range: 4.0                      # Maximum range (4m)    max_range: 4.0                      # Maximum range (4m)

    fov: 0.5                           # Field of view (radians)    fov: 0.5                           # Field of view (radians)

    rate: 15.0                         # Publishing rate (Hz)    rate: 15.0                         # Publishing rate (Hz)

``````



### Launch Parameters### Launch Parameters

Override parameters at launch time:Override parameters at launch time:

```bash```bash

ros2 launch diffbot_sensors front_sensor.launch.py \ros2 launch diffbot_sensors front_sensor.launch.py \

    trig_pin:=18 \    trig_pin:=18 \

    echo_pin:=19 \    echo_pin:=19 \

    rate:=10.0    rate:=10.0

``````



## Troubleshooting## Troubleshooting



### Common Issues### Common Issues



**1. Permission denied (GPIO access):****1. Permission denied (GPIO access):**

```bash```bash

# Add user to gpio group# Add user to gpio group

sudo usermod -a -G gpio $USERsudo usermod -a -G gpio $USER

# Log out and back in# Log out and back in

``````



**2. lgpio library not found:****2. lgpio library not found:**

```bash```bash

# Install lgpio# Install lgpio

sudo apt install python3-lgpiosudo apt install python3-lgpio

# Or using pip# Or using pip

pip3 install lgpiopip3 install lgpio

``````



**3. Node falls back to simulation:****3. Node falls back to simulation:**

- Check GPIO wiring connections- Check GPIO wiring connections

- Verify lgpio library installation- Verify lgpio library installation

- Run with debug logging to see error details- Run with debug logging to see error details



**4. No sensor readings:****4. No sensor readings:**

- Check power supply (5V for HC-SR04)- Check power supply (5V for HC-SR04)

- Verify TRIG/ECHO pin connections- Verify TRIG/ECHO pin connections

- Test with simple GPIO toggle script- Test with simple GPIO toggle script



### Hardware Testing### Hardware Testing

Test GPIO pins directly:Test GPIO pins directly:

```bash```bash

# Test TRIG pin (should toggle)# Test TRIG pin (should toggle)

echo "23" > /sys/class/gpio/exportecho "23" > /sys/class/gpio/export

echo "out" > /sys/class/gpio/gpio23/directionecho "out" > /sys/class/gpio/gpio23/direction

echo "1" > /sys/class/gpio/gpio23/valueecho "1" > /sys/class/gpio/gpio23/value

echo "0" > /sys/class/gpio/gpio23/valueecho "0" > /sys/class/gpio/gpio23/value

echo "23" > /sys/class/gpio/unexportecho "23" > /sys/class/gpio/unexport

``````



## Data Format## Data Format



The sensor publishes `sensor_msgs/Range` messages with:The sensor publishes `sensor_msgs/Range` messages with:

- **header.frame_id**: "ultrasonic_front"- **header.frame_id**: "ultrasonic_front"

- **radiation_type**: ULTRASOUND (1)- **radiation_type**: ULTRASOUND (1)

- **field_of_view**: 0.5 radians (~28.6 degrees)- **field_of_view**: 0.5 radians (~28.6 degrees)

- **min_range**: 0.02 meters (2cm)- **min_range**: 0.02 meters (2cm)

- **max_range**: 4.0 meters- **max_range**: 4.0 meters

- **range**: Distance in meters- **range**: Distance in meters



Example message:Example message:

```yaml```yaml

header:header:

  stamp: {sec: 1234567890, nanosec: 123456789}  stamp: {sec: 1234567890, nanosec: 123456789}

  frame_id: "ultrasonic_front"  frame_id: "ultrasonic_front"

radiation_type: 1radiation_type: 1

field_of_view: 0.5field_of_view: 0.5

min_range: 0.02min_range: 0.02

max_range: 4.0max_range: 4.0

range: 1.235  # Distance in metersrange: 1.235  # Distance in meters

``````

   ```bash

## Quick Start Guide   rosdep install --from-paths src --ignore-src -r -y

   ```

1. **Connect HC-SR04 sensor to Raspberry Pi**

2. **Install dependencies**: `sudo apt install python3-lgpio`2. **Build all packages:**

3. **Build workspace**: `colcon build --packages-select diffbot_sensors`   ```bash

4. **Source workspace**: `source install/setup.bash`   ./build.sh

5. **Run sensor**: `ros2 run diffbot_sensors front_ultrasonic_node.py`   ```

6. **Monitor data**: `ros2 topic echo /ultrasonic_front/range`

3. **Clean and rebuild:**

## License   ```bash

   ./build.sh clean

This project is licensed under the Apache-2.0 License.   ```

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
ros2 launch diffbot_control motor_driver_4.launch.py

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