#!/bin/bash

echo "Launching Gazebo Garden with included robot model..."
echo "This should load the robot from the separate model file!"

# Get the package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WORLD_FILE="$PKG_DIR/worlds/include_robot_world.sdf"

echo "World file: $WORLD_FILE"
echo "Robot model file: $PKG_DIR/model.sdf"

# Start Gazebo with the include world
ign gazebo -r "$WORLD_FILE"
