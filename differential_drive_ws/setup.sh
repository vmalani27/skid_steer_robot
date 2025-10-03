#!/bin/bash

# Quick setup script for differential drive robot workspace

echo "Setting up differential drive robot workspace..."

# Source ROS2
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies with rosdep..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "Building workspace..."
./build.sh

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "Setup complete!"
echo ""
echo "To use the workspace:"
echo "1. Source it: source install/setup.bash"
echo "2. Launch visualization: ros2 launch diffbot_description display.launch.py"
echo "3. Launch simulation: ros2 launch diffbot_bringup gazebo.launch.py"
echo ""
echo "See README.md for more usage instructions."