# Block 2 Testing: 3D Object Localization

## Overview

Block 2 adds depth camera integration and TF2 coordinate transforms to compute 3D positions of detected objects in the map frame.

## New Features

✅ Synchronized RGB + Depth camera subscription  
✅ Camera intrinsics handling (fx, fy, cx, cy)  
✅ 3D point computation using pinhole camera model  
✅ TF2 coordinate transformation (camera → map/odom)  
✅ Publishing object poses to `/vision/object_pose`  

## Prerequisites

Before testing Block 2, verify your TurtleBot4 simulation provides:

1. **Depth camera topic** (one of these):
   - `/oakd/rgb/preview/depth`
   - `/oakd/depth/image_raw`
   - `/camera/depth/image_raw`

2. **Camera info topic**:
   - `/oakd/rgb/preview/camera_info`
   - `/camera/camera_info`

3. **TF frames** (check with `ros2 run tf2_ros tf2_echo map <camera_frame>`):
   - Camera optical frame (e.g., `oakd_rgb_camera_optical_frame`)
   - Map or odom frame

## Testing Procedure

### Step 1: Launch TurtleBot4 with Depth Camera

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

### Step 2: Verify Camera Topics

```bash
# Check available camera topics
ros2 topic list | grep -E '(depth|camera_info|oakd|camera)'

# Check depth topic type and rate
ros2 topic info /oakd/rgb/preview/depth --verbose
ros2 topic hz /oakd/rgb/preview/depth

# Check camera info
ros2 topic echo /oakd/rgb/preview/camera_info --once
```

### Step 3: Verify TF Frames

```bash
# List all TF frames
ros2 run tf2_ros tf2_echo map base_link

# Check camera frame (adjust name as needed)
ros2 run tf2_ros tf2_echo map oakd_rgb_camera_optical_frame
# or try:
ros2 run tf2_ros tf2_echo odom oakd_rgb_camera_link
```

### Step 4: Run Vision Detector with Block 2

```bash
# Terminal 2: Start vision detector
cd /path/to/your/workspace
source install/setup.bash
ros2 run tb4_gz_rqt_launch vision_detector_node
```

**Expected output:**
```
[INFO] [vision_detector_node]: Loading YOLOWorld model (size: l)...
[INFO] [vision_detector_node]: Vision Detector initialized (Block 2: 3D Localization)
[INFO] [vision_detector_node]: Subscribing to RGB: /oakd/rgb/preview/image_raw
[INFO] [vision_detector_node]: Subscribing to Depth: /oakd/rgb/preview/depth
[INFO] [vision_detector_node]: Subscribing to CameraInfo: /oakd/rgb/preview/camera_info
[INFO] [vision_detector_node]: Camera intrinsics: fx=320.0, fy=320.0, cx=320.0, cy=240.0
```

### Step 5: Set Detection Target

```bash
# Terminal 3: Set target object
ros2 topic pub /detection_target std_msgs/String '{data: "chair"}' --once
```

### Step 6: Monitor 3D Object Pose

```bash
# Terminal 4: Watch object pose output
ros2 topic echo /vision/object_pose
```

**Expected output when chair is detected:**
```yaml
header:
  stamp:
    sec: 1769440123
    nanosec: 456789000
  frame_id: map
pose:
  position:
    x: 2.45
    y: -0.83
    z: 0.52
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### Step 7: Visualize Detections

```bash
# Terminal 5: View annotated images
ros2 run rqt_image_view rqt_image_view /vision/detections
```

## Troubleshooting

### Issue: "No camera info available for 3D projection"

**Cause**: Camera info topic name mismatch or not publishing.

**Solution**: Check topic name and update parameter:
```bash
ros2 run tb4_gz_rqt_launch vision_detector_node \
    --ros-args \
    -p camera_info_topic:=/camera/camera_info
```

### Issue: "TF2 transform failed: frame does not exist"

**Cause**: TF frame name mismatch.

**Solution 1**: List available frames:
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing all TF frames
```

**Solution 2**: Update camera_frame parameter:
```bash
ros2 run tb4_gz_rqt_launch vision_detector_node \
    --ros-args \
    -p camera_frame:=camera_link
```

**Solution 3**: Try odom frame instead of map:
```bash
ros2 run tb4_gz_rqt_launch vision_detector_node \
    --ros-args \
    -p target_frame:=odom
```

### Issue: "Invalid depth at (x, y): nan"

**Cause**: Depth camera not publishing, or object outside depth range.

**Solutions**:
1. Check depth topic is publishing: `ros2 topic hz /oakd/rgb/preview/depth`
2. Verify depth encoding: `ros2 topic echo /oakd/rgb/preview/depth --once`
3. Move robot closer to target object
4. Check for glass/reflective surfaces (cause NaN depths)

### Issue: "Unknown depth encoding"

**Cause**: Unsupported depth image format.

**Solution**: Check encoding and add support:
```bash
ros2 topic echo /oakd/rgb/preview/depth --once | grep encoding
# Common encodings: 16UC1 (millimeters), 32FC1 (meters)
```

### Issue: "Bbox center out of depth image bounds"

**Cause**: RGB and depth images have different resolutions.

**Solutions**:
1. Check resolutions:
   ```bash
   ros2 topic echo /oakd/rgb/preview/camera_info --once | grep -A2 "width\|height"
   ros2 topic echo /oakd/rgb/preview/depth --once | grep -A2 "width\|height"
   ```
2. If different, depth image needs to be resized to match RGB in `rgbd_callback`

## Configuration Parameters

All parameters can be set via command line:

```bash
ros2 run tb4_gz_rqt_launch vision_detector_node --ros-args \
    -p model_size:=m \
    -p conf_threshold:=0.6 \
    -p camera_topic:=/camera/image_raw \
    -p depth_topic:=/camera/depth/image_raw \
    -p camera_info_topic:=/camera/camera_info \
    -p detection_rate_hz:=5.0 \
    -p target_frame:=odom \
    -p camera_frame:=camera_optical_frame
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_size` | string | `l` | YOLOWorld model size (s/m/l) |
| `conf_threshold` | float | 0.5 | Detection confidence threshold |
| `camera_topic` | string | `/oakd/rgb/preview/image_raw` | RGB camera topic |
| `depth_topic` | string | `/oakd/rgb/preview/depth` | Depth camera topic |
| `camera_info_topic` | string | `/oakd/rgb/preview/camera_info` | Camera intrinsics topic |
| `detection_rate_hz` | float | 10.0 | Detection frequency (Hz) |
| `target_frame` | string | `map` | Target coordinate frame (map/odom) |
| `camera_frame` | string | `oakd_rgb_camera_optical_frame` | Camera optical frame name |

## Success Criteria

✅ **Block 2 Complete** when:
1. RGB and Depth images are synchronized
2. Camera intrinsics are successfully parsed
3. Target object's 3D position is computed
4. TF2 transform from camera to map succeeds
5. `PoseStamped` is published to `/vision/object_pose`
6. Log shows: `📍 Target "chair" at (x, y, z) in frame "map"`

## Next Steps (Block 3)

- [ ] Create Nav2 action client for `/navigate_to_pose`
- [ ] Integrate with LLM command parser
- [ ] Handle multi-object navigation sequences
- [ ] Add visual servoing for fine positioning

---

**Status**: Block 2 implemented ✅  
**Last Updated**: January 26, 2026
