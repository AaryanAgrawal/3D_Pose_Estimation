# Isaac ROS DOPE Pose Estimation Development Environment

This repository contains the development environment setup for Isaac ROS DOPE (Deep Object Pose Estimation) with Astra camera integration for robotic manipulation tasks.

## Project Overview

This project implements a complete pose estimation pipeline using NVIDIA Isaac ROS DOPE for detecting and estimating 6D poses of everyday objects (milk containers, ketchup bottles, etc.). The system captures images from an Astra depth camera, processes them through the DOPE neural network, and outputs precise 3D position and orientation data suitable for robotic pick-and-place operations.

### Key Components
- **Isaac ROS DOPE**: Deep learning-based 6D pose estimation
- **Astra Camera Integration**: Real-time RGB-D image capture
- **Image Processing Pipeline**: Automated capture, crop, and format conversion
- **ROS2 Integration**: Standard robotics messaging and visualization

## Repository Structure

This repository should be placed in your root directory as:
```
~/workspaces/isaac_ros-dev/
```

Place the Isaac ROS configuration file in your home directory:
```
~/.isaac_ros_common-config
```

## Setup

### 1. USB Device Passthrough (Windows to WSL2)

**Prerequisites:**
```powershell
# Windows: Install USBIPD
winget install usbipd
```

```bash
# WSL2: Install USB tools
sudo apt update && sudo apt install linux-tools-virtual hwdata
```

**Connect Camera:**
```powershell
# 1. List devices
usbipd list

# 2. Bind device (one-time)
usbipd bind --busid <BUSID>

# 3. Attach to WSL2
usbipd attach --wsl --busid <BUSID>
```

**Verify:**
```bash
lsusb | grep -i orbbec  # For Astra cameras
```

### 2. Start Isaac ROS Container
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
```

## Run

### Automated Detection Scripts

**Live Milk Detection:**
```bash
${ISAAC_ROS_WS}/src/run_milk_detections.sh
```

**Live Orange Juice Detection:**
```bash
${ISAAC_ROS_WS}/src/run_orangejuice_detections.sh
```

**Export Detection Data:**
```bash
${ISAAC_ROS_WS}/src/run_camera_to_exported_detections.sh
```

### RViz2 Configuration (For Live Milk and Orange Juice Detection)

Once RViz2 opens during live detection:

1. **Change Fixed Frame:**
   - Set "Fixed Frame" to `camera_color_optical_frame`

2. **Add Camera Display:**
   - Click "Add" → "By topic"
   - Navigate to `/dope_encoder/resize/image` topic
   - Add "Camera" display

3. **Add Detection Visualization:**
   - Click "Add" → "By display type"
   - Add "Detection3DArray"
   - Set topic to `/detections`

This configuration applies to both live milk detection and live orange juice detection scripts.

## Model Setup

### Download and Convert DOPE Models

After completing the [NVIDIA Isaac ROS DOPE quickstart guide](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_dope/index.html), download additional models and convert them to ONNX format:

**1. Download Milk and Orange Juice Models:**
```bash
# Download Milk model
wget 'https://drive.nvidia.com/uc?export=download&id=1Byeg3I-GVOJu4LqJoWyoZFevWL78mZqh' -O Milk.pth

# Download Orange Juice model  
wget 'https://drive.nvidia.com/uc?export=download&id=1GCNwlW7MnTFQ3IELanYoFhbSBQwwFk2R' -O OrangeJuice.pth
```

**2. Convert Models to ONNX:**
```bash
# Inside Isaac ROS container
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/dope

# Convert Milk model
python3 ${ISAAC_ROS_WS}/isaac_ros_assets/scripts/dope_converter.py \
--format onnx --input_type rgb \
--input_file /path/to/Milk.pth \
--output_file Milk.onnx

# Convert Orange Juice model
python3 ${ISAAC_ROS_WS}/isaac_ros_assets/scripts/dope_converter.py \
--format onnx --input_type rgb \
--input_file /path/to/OrangeJuice.pth \
--output_file OrangeJuice.onnx
```

**3. Generate TensorRT Engine Files:**
```bash
# Generate Milk engine
/usr/src/tensorrt/bin/trtexec \
--onnx=Milk.onnx \
--saveEngine=Milk.plan \
--fp16

# Generate Orange Juice engine
/usr/src/tensorrt/bin/trtexec \
--onnx=OrangeJuice.onnx \
--saveEngine=OrangeJuice.plan \
--fp16
```

## Troubleshooting

**No detections:** Ensure objects match training data (milk containers, ketchup bottles)
**Camera not detected:** Verify USB passthrough is working
**Container issues:** Check Isaac ROS container started successfully

**Debug commands:**
```bash
ros2 topic list | grep -E "(camera|dope)"
ros2 topic hz /camera/color/image_raw
rqt_graph
```
