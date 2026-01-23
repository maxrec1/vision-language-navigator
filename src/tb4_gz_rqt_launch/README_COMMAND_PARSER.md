# Command Parser - Phi-3 LLM Integration

## Overview

This module extracts structured navigation goals from natural language commands using **Phi-3** (via Ollama).

**Input**: Natural language text (e.g., "go to the chair")  
**Output**: Structured JSON with semantic information:
```json
{
  "target": "chair",
  "relation": null,
  "reference": null
}
```

## How It Works

1. Sends user text to **Phi-3** (3.8B parameter model running locally via Ollama)
2. Uses a **system prompt** to force strict JSON output
3. Parses and validates the JSON response
4. Handles markdown formatting and edge cases automatically

## Quick Start

### Prerequisites
- **Ollama** installed and running: `ollama serve`
- **Phi-3 model** downloaded: `ollama pull phi3`
- **Python requests library**: `pip install requests --break-system-packages`

### Run Test
```bash
python3 ollama_test.py
```

### Expected Output
```
============================================================
OLLAMA + PHI-3 COMMAND PARSER TEST
============================================================

[1] Input: 'Go to the chair'
    Output: {
      "target": "chair",
      "relation": null,
      "reference": null
    }
...
```

## API Reference

### `parse_command(user_input: str) -> dict`

Extracts structured goal from natural language.

**Arguments:**
- `user_input` (str): User command (e.g., "find the table near the window")

**Returns:**
- `dict`: JSON with keys `target`, `relation`, `reference`, or `None` on error

**Example:**
```python
from ollama_test import parse_command

result = parse_command("Move to the sofa left of the lamp")
print(result)
# Output: {'target': 'sofa', 'relation': 'left of', 'reference': 'lamp'}
```

## Field Descriptions

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `target` | str | Destination object | "chair", "kitchen" |
| `relation` | str or null | Spatial preposition | "near", "behind", "left of" |
| `reference` | str or null | Landmark object (when relation exists) | "window", "bookshelf" |

## System Prompt

The parser uses few-shot prompting to ensure reliable JSON output:

```
You are a robot navigation parser. Your only job is to extract semantic targets from user text.
Output format: JSON only. No markdown, no conversational filler.
Keys: 'target', 'relation', 'reference'
If no spatial constraint: set 'relation' and 'reference' to null.
```

## Performance

- **Inference Time**: ~60-80 seconds per command (GPU-accelerated)
- **Accuracy**: High for navigation commands
- **Latency**: Best for offline planning, not real-time

## Troubleshooting

### "Failed to connect to Ollama"
```bash
# Make sure Ollama is running
ollama serve
```

### "Failed to parse JSON"
- Check Ollama is using Phi-3: `ollama list`
- Restart Ollama service: `sudo systemctl restart ollama`

## Next Steps

This module will be wrapped in a **ROS 2 Service Node** (`/parse_command`) for integration with TurtleBot4 navigation.
