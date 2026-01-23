# tb4_gz_rqt_launch

ROS 2 package for TurtleBot4 Gazebo simulation with LLM-based natural language command parsing.

## Features

- **TurtleBot4 Gazebo Bringup**: Launch TurtleBot4 in Gazebo simulator
- **RQT Image View**: Automatically starts camera visualization
- **Interactive Command Parser**: Parse natural language navigation commands using Phi-3 LLM (via Ollama)

## Installation

### System Dependencies

```bash
# Install ROS 2 packages (replace <distro> with jazzy)
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
cd ~/ros2_jazzy/vision-language-navigator
colcon build --packages-select tb4_interfaces tb4_gz_rqt_launch
source install/setup.bash
```

## Usage

### 1. Launch TurtleBot4 Simulator

```bash
ros2 launch tb4_gz_rqt_launch tb4_gz_rqt_launch.launch.py
```

**Notes:**
- Requires working display (GUI) for `rqt_image_view`
- Verify camera topic (e.g., `/camera/image_raw`) after spawn

### 2. Run Interactive Command Parser

**Terminal 1: Start Ollama**
```bash
ollama serve
```

**Terminal 2: Start Command Parser**
```bash
cd ~/ros2_jazzy/vision-language-navigator
source install/setup.bash
parse_command_node
```

**Example Session:**
```
============================================================
🤖 INTERACTIVE COMMAND PARSER
============================================================
Enter navigation commands (or 'quit' to exit)

🗣️  Command: go to the chair
[INFO] [command_parser_node]: Processing command: "go to the chair"
[INFO] [command_parser_node]: ✅ Parsed: target="chair", relation="null", reference="null"

📊 Result:
{
  "target": "chair",
  "relation": null,
  "reference": null
}

🗣️  Command: find the table near the window
[INFO] [command_parser_node]: ✅ Parsed: target="table", relation="near", reference="window"

📊 Result:
{
  "target": "table",
  "relation": "near",
  "reference": "window"
}

🗣️  Command: quit
👋 Goodbye!
```

## Command Parser Details

### How It Works

1. **Input**: Natural language command (e.g., "go to the chair")
2. **Processing**: Phi-3 LLM extracts structured navigation goal via Ollama API
3. **Output**: JSON with `target`, `relation`, and `reference` fields

### Test Commands

Try these examples:
- `go to the chair`
- `find the table near the window`
- `navigate to the kitchen`
- `move to the sofa left of the lamp`
- `go to bed`

### Performance

- **First request**: ~60-80 seconds (model loading + inference)
- **Subsequent requests**: ~20-40 seconds (inference only)
- **Model**: Phi-3 (3.8B parameters, 2.2 GB)
- **Inference**: GPU-accelerated via Ollama

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `No executable found: parse_command_node` | Rebuild: `colcon build --packages-select tb4_gz_rqt_launch` |
| `Connection refused` (Ollama) | Start Ollama: `ollama serve` |
| `ModuleNotFoundError: requests` | `pip install requests --break-system-packages` |
| Slow inference (60+ seconds) | Normal for Phi-3 on first request; subsequent requests are faster |
| `ModuleNotFoundError: tb4_interfaces` | Build interfaces: `colcon build --packages-select tb4_interfaces` |

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
