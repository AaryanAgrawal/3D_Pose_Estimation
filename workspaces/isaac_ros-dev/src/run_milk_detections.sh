#!/bin/bash

echo "Starting DOPE Milk Detection with RViz2..."

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

# Start DOPE for Milk detection
ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py \
    model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/dope/Milk.onnx \
    engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/dope/Milk.plan \
    object_name:=Milk &
DOPE_PID=$!
sleep 20

# Start RViz2 with custom config
rviz2 -d ${ISAAC_ROS_WS}/src/isaac_ros_dope/isaac_ros_dope/rviz/default.rviz &
RVIZ_PID=$!

echo "All systems running. Press Ctrl+C to stop..."

# Wait for user interrupt
trap 'echo "Shutting down..."; kill $CAMERA_PID $BRIDGE_PID $DOPE_PID $RVIZ_PID 2>/dev/null || true; exit' INT

# Keep script running
while true; do
    sleep 1
done

