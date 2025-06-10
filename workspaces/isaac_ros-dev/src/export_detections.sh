#!/bin/bash

set -e

echo "=== Sequential DOPE Detection Script ==="

# Function to print current ROS nodes
print_nodes() {
    echo "Current ROS nodes:"
    ros2 node list | grep -E "(dope|camera)" || echo "No DOPE/camera nodes found"
    echo "---"
}

wait_for_detections() {
    local object_name=$1
    local count=$2
    echo "Press ENTER to start collecting $count $object_name detections..."
    read
    
    # Create exported_detections folder if it doesn't exist
    mkdir -p ${ISAAC_ROS_WS}/src/exported_detections
    
    local detection_count=0
    local output_file="${ISAAC_ROS_WS}/src/exported_detections/${object_name}_detections.txt"
    
    # Clear previous file
    > "$output_file"
    
    echo "Collecting detections... (need $count non-empty)"
    
    while [ $detection_count -lt $count ]; do
        # Get one detection
        detection=$(timeout 5s ros2 topic echo /detections --once 2>/dev/null)
        
        if [ -z "$detection" ]; then
            echo "⚠ No detection received, retrying..."
            continue
        fi
        
        # Check if detection has empty detections array
        if echo "$detection" | grep -q "detections: \[\]"; then
            echo "⚠ Empty detection found, skipping..."
            continue
        fi
        
        # Valid detection found
        echo "$detection" >> "$output_file"
        detection_count=$((detection_count + 1))
        echo "✓ Got detection $detection_count/$count"
    done
    
    echo "✓ Collected $count valid $object_name detections"
}

# Function to launch DOPE with better error checking
launch_dope() {
    local object=$1
    echo "Starting DOPE for $object..."
    
    # Check model files exist
    if [ ! -f "${ISAAC_ROS_WS}/isaac_ros_assets/models/dope/${object}.onnx" ]; then
        echo "ERROR: ${object}.onnx not found"
        return 1
    fi
    
    ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py \
        model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/dope/${object}.onnx \
        engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/dope/${object}.plan \
        object_name:=${object} &
    
    DOPE_PID=$!
    echo "DOPE launched with PID: $DOPE_PID"
    
    # Wait longer for DOPE to initialize
    sleep 10
    
    # Verify DOPE is running
    if ! kill -0 $DOPE_PID 2>/dev/null; then
        echo "ERROR: DOPE failed to start"
        return 1
    fi
}

# Function to stop ALL DOPE nodes completely
stop_dope() {
    echo "Shutting down ALL DOPE nodes..."
    
    # Kill main DOPE process
    if [ ! -z "$DOPE_PID" ]; then
        kill $DOPE_PID 2>/dev/null || true
        wait $DOPE_PID 2>/dev/null || true
    fi
    
    # Kill all DOPE-related processes
    pkill -f "dope_container" 2>/dev/null || true
    pkill -f "dope_decoder" 2>/dev/null || true
    pkill -f "dope_encoder" 2>/dev/null || true
    pkill -f "dope_inference" 2>/dev/null || true
    pkill -f "isaac_ros_dope" 2>/dev/null || true
    pkill -f "tensor_rt" 2>/dev/null || true
    pkill -f "image_format_converter" 2>/dev/null || true
    pkill -f "image_to_tensor" 2>/dev/null || true
    pkill -f "crop_node" 2>/dev/null || true
    pkill -f "resize_node" 2>/dev/null || true
    pkill -f "normalize_node" 2>/dev/null || true
    pkill -f "reshape_node" 2>/dev/null || true
    pkill -f "interleaved_to_planar" 2>/dev/null || true
    
    sleep 5
    echo "All DOPE nodes killed"
}

# Main execution
echo "Phase 1: Milk Detection"
launch_dope "Milk"
print_nodes
wait_for_detections "Milk" 10
stop_dope
print_nodes

echo ""
echo "Phase 2: Orange Juice Detection"
launch_dope "OrangeJuice"
print_nodes
wait_for_detections "OrangeJuice" 10
stop_dope
print_nodes

echo ""
echo "=== Detection Complete ==="
echo "Results saved to:"
echo "  - ${ISAAC_ROS_WS}/src/exported_detections/Milk_detections.txt"
echo "  - ${ISAAC_ROS_WS}/src/exported_detections/OrangeJuice_detections.txt"
