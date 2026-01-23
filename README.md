# Vision Language Navigator

ROS 2 + TurtleBot4 + Phi-3 LLM-based navigation system for converting natural language commands to structured robot navigation goals.

## 🚀 Quick Start

### Prerequisites
- **ROS 2 Jazzy** installed
- **Ollama** (local LLM runner)
- **TurtleBot4 Gazebo simulator** packages
- Python 3.10+

### Installation

1. **Clone repository**
   ```bash
   git clone https://github.com/maxrec1/vision-language-navigator.git
   cd vision-language-navigator
   ```

2. **Install Ollama + Phi-3**
   ```bash
   # See docs/PHASE1_SETUP.md for detailed instructions
   curl -fsSL https://ollama.ai/install.sh | sh
   ollama pull phi3
   ```

3. **Install Python dependencies**
   ```bash
   pip install requests --break-system-packages
   ```

    **Test Command parser**
     ```bash
   python3 src/tb4_gz_rqt_launch/ollama_test.py   # might take some time to run
    ```

4. **Build ROS 2 workspace**
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **Run simulator + command parser**
   ```bash
   # Launch TurtleBot4 simulator + command parser
   ros2 launch tb4_gz_rqt_launch tb4_gz_rqt_launch.launch.py
   ```

## 📋 Project Structure

```
vision-language-navigator/
├── README.md                          ← You are here
├── .gitignore                         ← Git ignore rules
├── docs/
│   └── PHASE1_SETUP.md               ← Installation & setup guide
├── src/tb4_gz_rqt_launch/
│   ├── README_COMMAND_PARSER.md       ← Command parser API reference
│   ├── ollama_test.py                 ← Test script (standalone)
│   ├── command_parser_node.py         ← ROS 2 service node (Phase 2)
│   ├── package.xml
│   ├── setup.py
│   └── tb4_gz_rqt_launch/
│       ├── __init__.py
│       └── ... (other modules)
├── launch/
│   ├── tb4_gz_rqt_launch.launch.py    ← Main launch file
│   └── command_parser.launch.py       ← Parser node launcher (Phase 2)
└── README.md (this file)
```

## 🧠 How It Works

### Phase 1: LLM Command Parser ✅
Extracts structured navigation goals from natural language using **Phi-3** running locally via **Ollama**.

**Input**: `"go to the chair"`  
**Output**: `{"target": "chair", "relation": null, "reference": null}`

**Features:**
- Runs locally (privacy-preserving)
- GPU-accelerated inference
- Robust JSON extraction (handles markdown wrapping)
- Few-shot prompting for reliable output

### Phase 2: ROS 2 Service Integration (In Progress)
Wraps Phase 1 in a ROS 2 service node for TurtleBot4 integration.

**Service**: `/parse_command` (ParseCommand.srv)  
**Interface**: Converts natural language → structured navigation goals

### Phase 3: Navigation Stack (Planned)
Integrates with TurtleBot4 navigation stack and YOLO object detection.

## 📖 Documentation

- **[Phase 1 Setup Guide](docs/PHASE1_SETUP.md)** — How to install Ollama + Phi-3 + test the parser
- **[Command Parser Module](src/tb4_gz_rqt_launch/README_COMMAND_PARSER.md)** — API reference & usage examples

## 🧪 Testing

### Run Phase 1 Test (Standalone)
```bash
python3 src/tb4_gz_rqt_launch/ollama_test.py
```

**Output:**
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

[2] Input: 'Find the table near the window'
    Output: {
      "target": "table",
      "relation": "near",
      "reference": "window"
    }

...

[6] INTERACTIVE MODE - Enter your own command
🤖 Enter a navigation command (or press Enter to skip):
```

### Check Ollama Status
```bash
curl http://localhost:11434/api/tags
ollama list
```

## 🔧 Configuration

### Ollama Settings
- **Model**: Phi-3 (3.8B parameters, 2.2 GB)
- **API Endpoint**: `http://localhost:11434/api/generate`
- **Inference Timeout**: 120 seconds

### System Prompt
The parser uses a few-shot prompting strategy to force strict JSON output. See [docs/PHASE1_SETUP.md](docs/PHASE1_SETUP.md#system-prompt-strategy) for details.

## 📊 Performance

| Metric | Value |
|--------|-------|
| Model | Phi-3 (3.8B) |
| Inference Time | ~60-80s per command (GPU) |
| Accuracy | High for navigation commands |
| JSON Parsing Success | >95% |
| Latency Type | Offline planning (not real-time) |

## ⚠️ Troubleshooting

### Ollama connection failed
```bash
# Make sure Ollama service is running
sudo systemctl start ollama
# or
ollama serve
```

### Import error: `requests` not found
```bash
pip install requests --break-system-packages
```

### Slow inference
- Check GPU availability: `nvidia-smi`
- Consider lighter model (Llama 2 7B) if needed
- Monitor memory usage: `watch nvidia-smi`

### JSON parsing errors
- Re-run the test (LLMs are non-deterministic)
- Check Ollama logs: `journalctl -u ollama -f`
- See [docs/PHASE1_SETUP.md#troubleshooting](docs/PHASE1_SETUP.md#troubleshooting)

## 📝 Project Phases

### ✅ Phase 1: LLM Command Parser
- Install Ollama + Phi-3 ✅
- Implement JSON extraction ✅
- Test parser locally ✅
- Document API ✅

### 🔄 Phase 2: ROS 2 Service Integration
- Create `.srv` interface (ParseCommand)
- Implement service node
- Launch file integration
- ROS CLI testing

### 📅 Phase 3: Navigation Stack
- Integrate with TurtleBot4 move_base
- Add YOLO object detection
- Handle multi-step navigation
- Real-time visualization

## 🤝 Contributing

1. Fork the repo
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📄 License

This project is licensed under the **MIT License** — see LICENSE file for details.

## 👤 Author

**maxrec** — ROS 2 + Vision Language Navigation Research

## 🙏 Acknowledgments

- Phi-3 by Microsoft
- Ollama for local LLM inference
- TurtleBot4 by Clearpath Robotics
- ROS 2 Community

---

**Last Updated**: January 23, 2026  
**Status**: Phase 1 Complete, Phase 2 In Progress
