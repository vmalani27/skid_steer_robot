#!/bin/bash

# ROS2 Humble installation and dependency setup script
# Works for Ubuntu 22.04, Debian, and Raspberry Pi OS

echo "Setting up ROS2 Humble and dependencies..."

# Detect OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
fi

echo "Detected OS: $OS $VER"

# Install basic dependencies
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

# Set up ROS2 repository
echo "Setting up ROS2 repository..."
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository based on OS
if [[ "$OS" == *"Ubuntu"* ]]; then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
elif [[ "$OS" == *"Debian"* ]] || [[ "$OS" == *"Raspbian"* ]]; then
    # For Debian/Raspberry Pi OS, use Ubuntu Jammy (22.04) packages
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
else
    echo "Warning: Unsupported OS. Trying Ubuntu Jammy packages..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# Update package list
sudo apt update

# Check if ROS2 is already installed
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    sudo apt install -y ros-humble-ros-base
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
else
    echo "ROS2 already installed, skipping..."
fi

# Install development tools
echo "Installing development tools..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# Install project dependencies
echo "Installing project dependencies..."
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-urdf \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins

echo "Setup complete!"
echo "Please run 'source /opt/ros/humble/setup.bash' or restart your terminal."
echo "Then you can build the workspace with './build.sh'"