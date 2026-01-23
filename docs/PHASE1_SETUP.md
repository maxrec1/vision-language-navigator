# Phase 1 Setup: LLM Command Parser with Phi-3 & Ollama

This guide walks through installing Ollama and Phi-3, then testing the command parser locally before ROS 2 integration.

## What's Been Completed

### Step 1: Install Ollama ✅
- **Installed**: Ollama via official installer script
- **Status**: Running as systemd service on `localhost:11434`
- **GPU Support**: Detected and enabled (NVIDIA GPU detected)

### Step 2: Pull Phi-3 Model ✅
- **Model**: Phi-3 (3.8B parameters)
- **Download Size**: 2.2 GB
- **Command Used**: `ollama pull phi3`
- **Status**: Ready for inference

### Step 3: Verify with curl & JSON Extraction ✅
- **API Endpoint**: `http://localhost:11434/api/generate`
- **JSON Extraction**: Robust parser handles markdown formatting and extra content
- **Test Results**: All 5 test cases passing ✅

## Test Results

```
[1] Input: 'Go to the chair'
    Output: {"target": "chair", "relation": null, "reference": null}

[2] Input: 'Find the table near the window'
    Output: {"target": "table", "relation": "near", "reference": "window"}

[3] Input: 'Navigate to the kitchen'
    Output: {"target": "kitchen", "relation": null, "reference": null}

[4] Input: 'Move to the sofa left of the lamp'
    Output: {"target": "sofa", "relation": "left of", "reference": "lamp"}

[5] Input: 'Go to bed'
    Output: {"target": "bed", "relation": null, "reference": null}
```

## Installation Instructions

### Prerequisites
- Linux (Ubuntu 22.04+ recommended)
- NVIDIA GPU (optional but recommended for speed)
- Internet connection

### Step 1: Install Ollama

**Download and install:**
```bash
curl -fsSL https://ollama.ai/install.sh | sh
```

**Verify installation:**
```bash
ollama --version
```

**Start Ollama service:**
```bash
# Ollama runs as a systemd service automatically
# Check status:
sudo systemctl status ollama

# If not running:
sudo systemctl start ollama
```

### Step 2: Pull Phi-3 Model

```bash
ollama pull phi3
```

This downloads ~2.2 GB. Verify it's installed:
```bash
ollama list
```

### Step 3: Install Python Dependencies

```bash
pip install requests --break-system-packages
```

### Step 4: Test the Parser

```bash
cd /path/to/vision-language-navigator
python3 src/tb4_gz_rqt_launch/ollama_test.py
```

**Expected output:**
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

## Key Implementation Details

### System Prompt Strategy
The system prompt uses **few-shot prompting** to force Phi-3 to output strict JSON:

```
You are a robot navigation parser. Your only job is to extract semantic targets from user text.

Output format: JSON only. No markdown, no conversational filler.

Keys: 
- 'target' (the destination object)
- 'relation' (spatial preposition)
- 'reference' (landmark object)

If no spatial constraint exists: set 'relation' and 'reference' to null.

Examples:
Input: 'Go to the chair' -> Output: {"target": "chair", "relation": null, "reference": null}
Input: 'Find the table near the window' -> Output: {"target": "table", "relation": "near", "reference": "window"}
```

### JSON Extraction Robustness
Even when Phi-3 outputs markdown-wrapped JSON, the parser correctly extracts the valid JSON by:
1. Finding the first `{` and matching closing `}` via brace counting
2. Stripping extra markdown syntax
3. Parsing the resulting valid JSON

### Performance Notes
- **Inference Time**: ~60-80 seconds per command on GPU (Phi-3 is slower than expected)
- **CPU Usage**: Can run on CPU but will be significantly slower
- **Recommendation**: Monitor GPU usage; consider lighter model (Llama 2 7B) if latency becomes critical

## Troubleshooting

### "Connection refused" or "Failed to connect to Ollama"

**Problem**: Ollama service is not running.

**Solution**:
```bash
sudo systemctl start ollama
# or
ollama serve
```

### "ModuleNotFoundError: No module named 'requests'"

**Solution**:
```bash
pip install requests --break-system-packages
```

### "Timeout after 120 seconds"

**Problem**: Phi-3 inference is taking too long (normal behavior).

**Solution**: Wait longer or check GPU availability:
```bash
nvidia-smi  # Check GPU status
```

### "Failed to parse JSON"

**Problem**: Phi-3 returned malformed JSON.

**Solution**: This occasionally happens with LLMs. Re-run the test or check Ollama logs:
```bash
journalctl -u ollama -f
```

## Quick Reference

### Check Ollama Status
```bash
curl http://localhost:11434/api/tags
```

### Test Command Extraction
```bash
python3 src/tb4_gz_rqt_launch/ollama_test.py
```

### Manual curl Test
```bash
curl -s http://localhost:11434/api/generate \
  -H "Content-Type: application/json" \
  -d '{
    "model": "phi3",
    "prompt": "Current Input: Go to the kitchen",
    "system": "Output JSON only with keys: target, relation, reference",
    "stream": false
  }'
```

### Stop Ollama Service
```bash
sudo systemctl stop ollama
```

## Hardware Notes
- **GPU Detected**: NVIDIA (service auto-configured for GPU acceleration)
- **Inference Speed**: ~60s per request (typical for Phi-3 on first pass)
- **Memory**: Monitor if running Gazebo + Phi-3 simultaneously

## Next Steps

After verifying Phase 1 works, proceed to **Phase 2: ROS 2 Service Integration**:
- Create ROS 2 service interface (`.srv` file)
- Wrap `parse_command()` in a ROS 2 service node
- Test with `ros2 service call`

---

✅ **Phase 1: Complete**  
→ Ready for Phase 2: Create ROS 2 Service Node
