#!/bin/bash

echo "Installing dependencies for Skid Steer Robot Package..."
echo "This script will install ROS2 Humble, Gazebo Fortress, and required packages."
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "Please do not run this script as root. Run it as a regular user."
    exit 1
fi

# Update package list
echo "Updating package list..."
sudo apt update

# Install ROS2 Humble if not already installed
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    
    # Add ROS2 repository
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install ros-humble-desktop -y
    
    # Source ROS2
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    echo "ROS2 Humble installed successfully!"
else
    echo "ROS2 is already installed."
fi

# Install Gazebo Fortress if not already installed
if ! command -v gz &> /dev/null; then
    echo "Installing Gazebo Fortress..."
    
    # Add Gazebo repository
    sudo apt install lsb-release wget gnupg -y
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    sudo apt update
    sudo apt install gazebo-fortress -y
    
    echo "Gazebo Fortress installed successfully!"
else
    echo "Gazebo is already installed."
fi

# Install ros_gz
echo "Installing ros_gz..."
sudo apt install ros-humble-ros-gz -y

# Install additional ROS2 packages
echo "Installing additional ROS2 packages..."
sudo apt install ros-humble-controller-manager \
                 ros-humble-diff-drive-controller \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-tf2-tools -y

# Install build tools
echo "Installing build tools..."
sudo apt install build-essential cmake -y

echo ""
echo "Dependencies installation complete!"
echo ""
echo "Next steps:"
echo "1. cd ~/ros2_ws"
echo "2. colcon build --packages-select skid_steer_robot"
echo "3. source install/setup.bash"
echo "4. python3 src/skid_steer_robot/test_setup.py"
echo "5. ros2 launch skid_steer_robot simple_sim.launch.py"
echo ""
echo "Note: You may need to restart your terminal or run 'source ~/.bashrc' for ROS2 to be available."
