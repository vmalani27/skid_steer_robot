#!/bin/bash

echo "Testing Gazebo Garden with simple world..."
echo "This should be much more stable..."

# Get the package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WORLD_FILE="$PKG_DIR/worlds/simple_world.sdf"

echo "World file: $WORLD_FILE"

# Start Gazebo with the simple world
ign gazebo -r "$WORLD_FILE"
