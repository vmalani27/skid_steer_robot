#!/bin/bash

echo "Launching Gazebo Garden with better skid steer robot..."
echo "This should show a red body with 4 blue wheels!"

# Get the package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WORLD_FILE="$PKG_DIR/worlds/better_robot_world.sdf"

echo "World file: $WORLD_FILE"

# Start Gazebo with the better robot world
ign gazebo -r "$WORLD_FILE"
