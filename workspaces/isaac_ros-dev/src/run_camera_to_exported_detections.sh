#!/bin/bash

echo "Starting DOPE Detection System for all objects in export_detections.sh..."

# Source ROS setup
source ${ISAAC_ROS_WS}/install/setup.bash

# WSL2: Install USB tools
sudo apt update
sudo apt install -y linux-tools-virtual hwdata usbutils

# Start camera in background
ros2 launch astra_camera astra.launch.xml &
CAMERA_PID=$!
sleep 3

# Start bridge in background  
python3 ${ISAAC_ROS_WS}/src/dope_bridge.py &
BRIDGE_PID=$!
sleep 2

# Run detection export
${ISAAC_ROS_WS}/src/export_detections.sh

# Cleanup
kill $CAMERA_PID $BRIDGE_PID 2>/dev/null || true

echo "Detection complete. Files in: isaac_ros-dev/src/exported_detections/"
