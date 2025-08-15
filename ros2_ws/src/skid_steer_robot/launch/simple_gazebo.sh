#!/bin/bash

# Get the package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WORLD_FILE="$PKG_DIR/worlds/world.sdf"

echo "Starting Gazebo Garden with skid steer robot world..."
echo "World file: $WORLD_FILE"

# Start Gazebo with the world file
ign gazebo -r "$WORLD_FILE"
