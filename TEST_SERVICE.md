# Testing the ROS 2 Command Parser Service

This guide shows how to test the `/parse_command` service with user input.

## Quick Start

### Prerequisites
1. **Ollama running** (in separate terminal):
   ```bash
   ollama serve
   ```

2. **ROS 2 packages built**:
   ```bash
   cd ~/ros2_jazzy/vision-language-navigator
   colcon build
   ```

### Run Interactive Test

```bash
cd ~/ros2_jazzy/vision-language-navigator
python3 test_parse_service.py
```

The script will:
1. Start the `parse_command_node` service
2. Prompt you to enter navigation commands
3. Send each command to Phi-3 via Ollama
4. Display parsed results (target, relation, reference, success)

### Example Session

```
======================================================================
ROS 2 Command Parser Service - Interactive Test
======================================================================

[1/3] Starting parse_command_node...
[✓] parse_command_node started on /parse_command

[2/3] Ready for commands!
----------------------------------------------------------------------
Enter natural language navigation commands:
  Examples:
    • 'go to the chair'
    • 'move to the table near the window'
    • 'navigate to the kitchen'
  Commands: 'quit' or 'exit' to stop

🤖 Enter command: go to the sofa left of the lamp

  [Processing] 'go to the sofa left of the lamp'...

  Response:
    target_object='sofa'
    relation_label='left of'
    reference_object='lamp'
    success=True

🤖 Enter command: quit
```

## Manual Testing with ROS CLI

If you prefer to test manually:

### Terminal 1: Start Ollama
```bash
ollama serve
```

### Terminal 2: Start the service node
```bash
cd ~/ros2_jazzy/vision-language-navigator
source install/setup.bash
source /opt/ros/jazzy/setup.bash
parse_command_node
```

### Terminal 3: Call the service
```bash
cd ~/ros2_jazzy/vision-language-navigator
source install/setup.bash
source /opt/ros/jazzy/setup.bash

# Test with different commands
ros2 service call /parse_command tb4_interfaces/srv/ParseCommand "{command_text: 'go to the chair'}"
ros2 service call /parse_command tb4_interfaces/srv/ParseCommand "{command_text: 'find the table near the window'}"
ros2 service call /parse_command tb4_interfaces/srv/ParseCommand "{command_text: 'navigate to the kitchen'}"
```

## Expected Output Format

**Request:**
```yaml
command_text: "find the table near the window"
```

**Response:**
```yaml
target_object: "table"
relation_label: "near"
reference_object: "window"
success: true
```

## Field Meanings

| Field | Description | Example |
|-------|-------------|---------|
| `command_text` | Natural language navigation command | "go to the chair" |
| `target_object` | Destination object the robot should navigate to | "chair", "kitchen", "table" |
| `relation_label` | Spatial relationship to reference object | "near", "left of", "behind" |
| `reference_object` | Landmark object (if relation exists) | "window", "bookshelf", "lamp" |
| `success` | Whether parsing succeeded | `true` / `false` |

## Troubleshooting

### "Service call failed"
- Check that `parse_command_node` is running
- Verify Ollama is accessible: `curl http://localhost:11434/api/tags`
- Check firewall/network connectivity

### "ModuleNotFoundError"
- Ensure you've run `colcon build`
- Source the environment: `source install/setup.bash`

### Slow responses (~60-80 seconds)
- This is normal for Phi-3 inference on first pass
- GPU usage will decrease on subsequent calls
- Consider using a lighter model if latency is critical

## Test Coverage

The interactive test validates:
- ✅ Service registration (`/parse_command`)
- ✅ Command parsing accuracy (simple and complex commands)
- ✅ JSON extraction from LLM output
- ✅ ROS service response mapping
- ✅ Error handling

---

**Last Updated**: January 23, 2026  
**Status**: ✅ Fully Functional
