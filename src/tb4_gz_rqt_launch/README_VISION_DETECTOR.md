# Vision Detector Node

YOLOWorld-based real-time object detection for TurtleBot4 with 3D localization.

## Features

### Block 1: 2D Object Detection ✅

✅ Subscribe to camera feed (`/oakd/rgb/preview/image_raw`)  
✅ Accept target object commands via `/detection_target` topic  
✅ Run YOLO inference at 10 Hz (non-blocking timer)  
✅ Draw bounding boxes (GREEN for target, ORANGE for others)  
✅ Publish annotated images to `/vision/detections`  
✅ Skip frames if inference is slow  

### Block 2: 3D Localization ✅

✅ Synchronized RGB + Depth camera subscription  
✅ Camera intrinsics handling (fx, fy, cx, cy)  
✅ 3D point computation using pinhole camera model  
✅ TF2 coordinate transformation (camera → map/odom)  
✅ Publish object poses to `/vision/object_pose` (geometry_msgs/PoseStamped)  
✅ Real-time 3D position logging  

## Installation

### 1. Install Dependencies

```bash
# Install Python packages
pip3 install ultralytics opencv-python torch torchvision --break-system-packages

# Install ROS 2 cv-bridge 
sudo apt install ros-jazzy-cv-bridge python3-cv-bridge
```

### 2. Build the Package

```bash
# Navigate to your workspace root
cd /path/to/your/workspace  # e.g., ~/ros2_ws or ~/vision-language-navigator
colcon build --packages-select tb4_gz_rqt_launch
source install/setup.bash
```

## Usage

### Step 1: Launch TurtleBot4 in Gazebo

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

### Step 2: Start Vision Detector

```bash
ros2 run tb4_gz_rqt_launch vision_detector_node
```

**Parameters:**
- `model_size`: YOLOWorld model size (`s`, `m`, `l`) - default: `l`
- `conf_threshold`: Confidence threshold (0.0-1.0) - default: `0.5`
- `camera_topic`: Camera topic to subscribe - default: `/oakd/rgb/preview/image_raw`
- `detection_rate_hz`: Detection frequency in Hz - default: `10.0`

Example with custom parameters:
```bash
ros2 run tb4_gz_rqt_launch vision_detector_node \
    --ros-args \
    -p model_size:=m \
    -p conf_threshold:=0.6 \
    -p detection_rate_hz:=5.0
```

### Step 3: Set Detection Target

```bash
# Set target to "chair"
ros2 topic pub /detection_target std_msgs/String '{data: "chair"}' --once

# Or use the test script
ros2 run tb4_gz_rqt_launch test_vision_detector chair
```

### Step 4: View Detections & 3D Poses

```bash
# Terminal 4: View annotated images
ros2 run rqt_image_view rqt_image_view /vision/detections

# Terminal 5: Monitor 3D object poses (Block 2)
ros2 topic echo /vision/object_pose
```

**Expected pose output:**
```yaml
header:
  frame_id: map
pose:
  position:
    x: 2.45
    y: -0.83
    z: 0.52
  orientation:
    w: 1.0
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detection_target` | `std_msgs/String` | Input: Set the target object to detect |
| `/oakd/rgb/preview/image_raw` | `sensor_msgs/Image` | Input: RGB camera feed from TurtleBot4 |
| `/oakd/rgb/preview/depth` | `sensor_msgs/Image` | Input: Depth camera feed (Block 2) |
| `/oakd/rgb/preview/camera_info` | `sensor_msgs/CameraInfo` | Input: Camera intrinsics (Block 2) |
| `/vision/detections` | `sensor_msgs/Image` | Output: Annotated image with bounding boxes |
| `/vision/object_pose` | `geometry_msgs/PoseStamped` | Output: 3D pose of target object in map frame (Block 2) |

## Troubleshooting

### No camera feed
```bash
# Check available camera topics
ros2 topic list | grep image

# If /oakd/rgb/preview/image_raw doesn't exist, try:
# /camera/color/image_raw
# /camera/image_raw
```

### YOLOWorld model download
First run will download the model (~140MB for yolov8l-worldv2.pt). Be patient.

### Low FPS
- Try smaller model: `-p model_size:=s`
- Reduce detection rate: `-p detection_rate_hz:=5.0`

### No detections
- Lower confidence: `-p conf_threshold:=0.3`
- Check if object is in default class list (edit node to add more classes)

### Block 2 Troubleshooting

See **[BLOCK2_TESTING.md](BLOCK2_TESTING.md)** for detailed Block 2 troubleshooting including:
- TF frame mismatches
- Invalid depth values
- Camera intrinsics issues
- Depth encoding problems

## Next Steps (Block 3)

- [ ] Create Nav2 action client for `/navigate_to_pose`
- [ ] Integrate with ollama command parser for natural language goals
- [ ] Handle multi-step navigation sequences
- [ ] Add visual servoing for fine positioning

## Architecture

### Block 1: 2D Detection
```
┌─────────────────┐
│  TurtleBot4 RGB │
│  /oakd/rgb/...  │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│  Vision Detector Node       │
│  - Subscribe to camera      │
│  - Listen for target        │
│  - Run YOLO inference       │
│  - Draw bounding boxes      │
│  - Publish annotated frame  │
└────────┬────────────────────┘
         │
         ▼
┌─────────────────┐
│  /vision/       │
│  detections     │
│  (Image)        │
└─────────────────┘
```

### Block 2: 3D Localization
```
TurtleBot4 OAK-D Camera:
  ├─ RGB Image ────────────┐
  ├─ Depth Image ──────────┤
  └─ Camera Info ──────────┤
                           ├→ vision_detector_node:
TF2 Transform Listener ────┤    - Sync RGB + Depth
                           │    - Run YOLO on RGB
                           │    - Get depth at bbox center
                           │    - Project to 3D camera frame
                           │    - Transform to map frame
                           ↓
                     3D Object Pose
                     (x, y, z in map)
                           ↓
                     /vision/object_pose
                     (PoseStamped)
```
         │
         ▼
┌─────────────────┐
│  /vision/       │
│  detections     │
│  (Image)        │
└─────────────────┘
```

## Code Structure

- **vision_detector_node.py**: Main detector node
- **test_vision_detector.py**: Test utility to set targets
- Vision detector runs at 10 Hz with non-blocking timer
- Thread-safe frame storage
- Skips frames if inference is too slow
