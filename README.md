# Vision Language Navigator

An intelligent ROS 2 navigation system that combines natural language processing, computer vision, and autonomous navigation to enable TurtleBot4 robots to navigate to objects described in plain English.

## 🎯 Features

- **Natural Language Commands**: Speak naturally ("go to the chair", "find the table")
- **LLM-Powered Parsing**: Phi-3 model extracts semantic navigation goals via Ollama
- **Open-Vocabulary Object Detection**: YOLO-World detects any object by text description (zero-shot)
- **Autonomous Navigation**: Nav2 integration for path planning and obstacle avoidance
- **3D Localization**: Transforms detected objects from camera frame to map coordinates
- **Gazebo Simulation**: Test and develop in realistic warehouse environments

## 🧠 How It Works

```
User: "Go to the chair"
     ↓
[Phi-3 LLM] → Extracts: target="chair"
     ↓
[YOLO-World] → Detects chair in camera feed + depth data
     ↓
[TF Transform] → Converts camera coordinates → map coordinates
     ↓
[Nav2] → Plans path and navigates robot to goal
```

## 📋 Prerequisites

- **ROS 2 Jazzy** (or Humble/Iron with adjustments)
- **Ollama** with Phi-3 model
- **TurtleBot4 Gazebo packages**
- **Python 3.10+**
- **YOLO-World model** (downloaded automatically)

## Installation

### System Dependencies

```bash
# Install ROS 2 packages (replace <distro> with jazzy)
sudo apt update && sudo apt install \
  ros-jazzy-rqt-image-view \
  ros-jazzy-turtlebot4-gz-bringup


# Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# Pull Phi-3 model
ollama pull phi3

# Install Python dependencies
pip install requests --break-system-packages
```

### Build Package

```bash
# Navigate to your workspace root
cd ~/vision-language-navigator  # Replace with your actual workspace path
colcon build --packages-select tb4_interfaces tb4_gz_rqt_launch
source install/setup.bash
```

## Usage

### 1. Launch TurtleBot4 Simulator with Nav2
**Terminal 1: Launch Gazebo with Nav2 and Localization**
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true localization:=true rviz:=true
```

**Set the initial spawn point of the robot (critical for AMCL localization)**

After robot has spawned in the world, wait ~5 seconds for all nodes to initialize, then undock the robot with HMI and:


*Using Command Line in another terminal*
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
  }
}"
```

### 2. Run Interactive Command Parser

**Terminal 2: Start Command Parser - Send target**
```bash
# From your workspace root
colcon build --packages-select tb4_interfaces tb4_gz_rqt_launch
source install/setup.bash
ros2 run tb4_gz_rqt_launch parse_command_node
```
**Terminal 3: Run Object Detection Node**
```bash
ros2 run tb4_gz_rqt_launch vision_detector_node
```
**Terminal 4: Visualize Object Detection in Live Camera Feed**
```bash
 ros2 run rqt_image_view rqt_image_view /vision/detections
```
## 🔧 System Architecture

### Component Overview

**1. Command Parser Node (`parse_command_node`)**
- Accepts natural language input from user
- Sends text to Phi-3 LLM via Ollama API
- Extracts structured navigation goals: `{target, relation, reference}`
- Example: "go to the chair" → `{target: "chair", relation: null, reference: null}`

**2. Vision Detector Node (`vision_detector_node`)**
- Subscribes to camera feed (`/oakd/rgb/preview/image_raw`) and depth data
- Uses **YOLO-World** for open-vocabulary object detection
  - Zero-shot detection: recognizes any object from text prompts
  - No retraining needed for new object categories
  - Slower than YOLOv8 but far more flexible
- Calculates 3D position using depth and TF transforms
- Converts coordinates from `camera_link` → `map` frame
- Sends navigation goals to Nav2 when target object detected

**3. Nav2 Navigation Stack**
- AMCL localization for map-based positioning
- Global and local costmaps for obstacle avoidance
- Path planning with multiple algorithms (NavFn, Smac, etc.)
- Controller for trajectory execution

### Pipeline Flow

```
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐    ┌──────────┐
│ User Input  │───▶│  Phi-3 LLM   │───▶│  YOLO-World     │───▶│  Nav2    │
│ "go to X"   │    │  (Ollama)    │    │  + TF Transform │    │  Action  │
└─────────────┘    └──────────────┘    └─────────────────┘    └──────────┘
                          │                      │                    │
                   Parse target           Detect & Localize      Navigate
                   object name            object in 3D space     to goal
```


## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| **Build & Dependencies** |
| `No executable found: parse_command_node` | Rebuild: `colcon build --packages-select tb4_gz_rqt_launch && source install/setup.bash` |
| `ModuleNotFoundError: requests` | `pip install requests --break-system-packages` |
| `ModuleNotFoundError: tb4_interfaces` | Build interfaces first: `colcon build --packages-select tb4_interfaces` |
| `ModuleNotFoundError: ultralytics` | `pip install ultralytics --break-system-packages` |
| **Ollama & LLM** |
| `Connection refused` (Ollama) | Start Ollama: `ollama serve` (in separate terminal) |
| Slow inference (60+ seconds) | Normal for Phi-3 on first request; caches improve subsequent requests |
| Stop Phi-3 model in background | `sudo killall ollama` or `pkill ollama` |
| **Nav2 & Localization** |
| `TF2 transform failed: "map" does not exist` | Launch with localization: `nav2:=true localization:=true` AND set initial pose |
| `AMCL cannot publish pose` | Set initial pose using RViz "2D Pose Estimate" or `/initialpose` topic |
| `map` frame exists but no transform | Move robot slightly so AMCL can converge using laser scan data |
| AMCL not converging | Verify laser scan is publishing: `ros2 topic hz /scan` |
| **Motion Control** |
| `Ignoring velocities - autonomous behavior running` | Undock robot: `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"` |
| Robot won't accept commands | Check if docked or reflexes enabled |
| **RViz & Visualization** |
| `Fixed frame "map" does not exist` | Set initial pose first, or change Fixed Frame to `odom` in RViz Global Options |
| Can't see robot in RViz | Add RobotModel display (Add → RobotModel) |
| No map displayed | Add Map display (Add → Map, Topic: `/map`) |
| **Object Detection** |
| Vision detector not finding objects | Check camera feed: `ros2 topic hz /oakd/rgb/preview/image_raw` |
| No detections published | Verify target object is in YOLO-World vocabulary and visible in frame |
| TF transform errors in vision node | Ensure `map` frame exists and localization is active |

## ✅ Testing & Verification

### Test Individual Components

**1. Test Ollama & Phi-3:**
```bash
curl http://localhost:11434/api/generate -d '{
  "model": "phi3",
  "prompt": "Extract the target object from: go to the chair",
  "stream": false
}'
```

**2. Test Camera Feed:**
```bash
ros2 topic hz /oakd/rgb/preview/image_raw  # Should show ~30 Hz
ros2 run rqt_image_view rqt_image_view /oakd/rgb/preview/image_raw
```

**3. Test Localization:**
```bash
ros2 run tf2_ros tf2_echo map odom  # Should show transform data
ros2 topic echo /amcl_pose --once    # Should show robot pose
```

**4. Test Navigation:**
```bash
# Send a simple navigation goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### Verify Robot Spawn Location
```bash
ros2 topic echo /odom --once  # Check position (should be ~0, 0)
```

## 📚 Documentation

- **[README_COMMAND_PARSER.md](README_COMMAND_PARSER.md)** - Detailed API reference for command parser module
- **[README_VISION_DETECTOR.md](README_VISION_DETECTOR.md)** - Vision detector implementation details
- **[TEST_SERVICE.md](TEST_SERVICE.md)** - Service testing guide
- **[ollama_test.py](ollama_test.py)** - Standalone test script for Phi-3 JSON extraction

## Project Structure

```
tb4_gz_rqt_launch/
├── launch/
│   └── tb4_gz_rqt_launch.launch.py    # Main launch file
├── tb4_gz_rqt_launch/
│   ├── __init__.py
│   ├── command_parser_node.py          # Interactive command parser (ROS 2 node)
│   └── ollama_test.py                  # LLM parsing logic (Phi-3 via Ollama)
├── package.xml
├── setup.py
└── README.md                           # This file
```

## See Also

- [Main Project README](../../README.md) - Full project overview
- [Phase 1 Setup Guide](../../docs/PHASE1_SETUP.md) - Ollama + Phi-3 installation

