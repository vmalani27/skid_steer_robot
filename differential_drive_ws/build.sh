#!/bin/bash

# Build script for differential drive robot workspace

echo "Building differential drive robot workspace..."

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Clean previous build if requested
if [ "$1" == "clean" ]; then
    echo "Cleaning previous build..."
    rm -rf build/ install/ log/
fi

# Build the workspace
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Source the workspace with: source install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi