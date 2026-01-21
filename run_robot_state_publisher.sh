#!/bin/bash
# Script to run robot_state_publisher with proper setup

# Stop any existing instances
pkill -f robot_state_publisher 2>/dev/null

# Restart ROS2 daemon
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

# Get the URDF path
URDF_PATH="${1:-mouse2.urdf.xacro}"

# Check if file exists
if [ ! -f "$URDF_PATH" ]; then
    echo "Error: URDF file not found: $URDF_PATH"
    exit 1
fi

# Process the URDF with xacro
URDF_STRING=$(xacro "$URDF_PATH" 2>&1)
XACRO_EXIT=$?

if [ $XACRO_EXIT -ne 0 ]; then
    echo "Error processing URDF with xacro:"
    echo "$URDF_STRING"
    exit 1
fi

# Run robot_state_publisher with the processed URDF
echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF_STRING"














