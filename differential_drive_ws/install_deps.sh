#!/bin/bash

# Install all required ROS2 packages for differential drive robot workspace

echo "Installing ROS2 Humble dependencies for differential drive robot..."

# Update package list
sudo apt update

# Install core ROS2 packages
echo "Installing core ROS2 packages..."
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-urdf \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs

# Install control packages
echo "Installing ROS2 control packages..."
sudo apt install -y \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control-test-assets

# Install sensor packages
echo "Installing sensor packages..."
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs

# Install navigation packages (optional)
echo "Installing navigation packages..."
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-navigation2 \
  ros-humble-nav2-msgs \
  ros-humble-slam-toolbox || echo "Navigation packages not available, skipping..."

# Install RViz
echo "Installing visualization packages..."
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins

echo "All dependencies installed successfully!"
echo ""
echo "Now you can build the workspace with:"
echo "  ./build.sh"
echo ""
echo "Or run the complete setup with:"
echo "  ./setup.sh"