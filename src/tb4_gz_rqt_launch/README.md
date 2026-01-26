# tb4_gz_rqt_launch

ROS 2 package for TurtleBot4 Gazebo simulation with LLM-based natural language command parsing.

## Features

- **TurtleBot4 Gazebo Bringup**: Launch TurtleBot4 in Gazebo simulator
- **RQT Image View**: Automatically starts camera visualization
- **Interactive Command Parser**: Parse natural language navigation commands using Phi-3 LLM (via Ollama)

## Installation

### System Dependencies

```bash# Install ROS 2 packages (replace <distro> with jazzy)
sudo apt update && sudo apt install \
  ros-<distro>-rqt-image-view \
  ros-<distro>-turtlebot4-gz-bringup


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
cd /path/to/your/workspace  # Replace with your actual workspace path
colcon build --packages-select tb4_interfaces tb4_gz_rqt_launch
source install/setup.bash
```

## Usage

### 1. Launch TurtleBot4 Simulator
**Terminal 1: Launch Gazebo**
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

### 2. Run Interactive Command Parser

**Terminal 2: Start Ollama**

```bash
ollama run phi3
```

**Terminal 3: Start Command Parser**
```bash
# From your workspace root
colcon build --packages-select tb4_interfaces tb4_gz_rqt_launch
source install/setup.bash
ros2 run tb4_gz_rqt_launch parse_command_node
```


## Command Parser Details

### How It Works

1. **Input**: Natural language command (e.g., "go to the chair")
2. **Processing**: Phi-3 LLM extracts structured navigation goal via Ollama API
3. **Output**: JSON with `target`, `relation`, and `reference` fields


| Issue | Solution |
|-------|----------|
| `No executable found: parse_command_node` | Rebuild: `colcon build --packages-select tb4_gz_rqt_launch` |
| `Connection refused` (Ollama) | Start Ollama: `ollama serve` |
| `ModuleNotFoundError: requests` | `pip install requests --break-system-packages` |
| Slow inference (60+ seconds) | Normal for Phi-3 on first request; subsequent requests are faster |
| `ModuleNotFoundError: tb4_interfaces` | Build interfaces: `colcon build --packages-select tb4_interfaces` |
| Stop Phi-3 model running in background | `sudo killall ollama` |

## Documentation

- **[README_COMMAND_PARSER.md](README_COMMAND_PARSER.md)** - Detailed API reference for command parser module
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
