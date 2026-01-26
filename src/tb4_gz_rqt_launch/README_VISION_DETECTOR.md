# Vision Detector Node - Block 1

YOLOWorld-based real-time object detection for TurtleBot4.

## Features (Block 1)

✅ Subscribe to camera feed (`/oakd/rgb/preview/image_raw`)  
✅ Accept target object commands via `/detection_target` topic  
✅ Run YOLO inference at 10 Hz (non-blocking timer)  
✅ Draw bounding boxes (GREEN for target, ORANGE for others)  
✅ Publish annotated images to `/vision/detections`  
✅ Skip frames if inference is slow  

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

### Step 4: View Detections

```bash
ros2 run rqt_image_view rqt_image_view /vision/detections
```

## Testing the Success Criteria

1. **Launch everything** (Gazebo + Vision Detector + Image Viewer)
2. **Set target**: `ros2 topic pub /detection_target std_msgs/String '{data: "chair"}' --once`
3. **Drive the robot** manually in front of a chair using teleop:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. **Verify**:
   - Green bounding box appears around the chair
   - Label shows "TARGET: chair 0.85"
   - Top-left corner shows "Looking for: chair"
   - Other objects have orange boxes

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detection_target` | `std_msgs/String` | Input: Set the target object to detect |
| `/oakd/rgb/preview/image_raw` | `sensor_msgs/Image` | Input: Camera feed from TurtleBot4 |
| `/vision/detections` | `sensor_msgs/Image` | Output: Annotated image with bounding boxes |

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

## Next Steps (Block 2)

- [ ] Add depth camera synchronization
- [ ] Calculate 3D position of detected objects
- [ ] Integrate with ollama command parser
- [ ] Create navigation service based on detections

## Architecture

```
┌─────────────────┐
│  TurtleBot4 Cam │
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

## Code Structure

- **vision_detector_node.py**: Main detector node
- **test_vision_detector.py**: Test utility to set targets
- Vision detector runs at 10 Hz with non-blocking timer
- Thread-safe frame storage
- Skips frames if inference is too slow
