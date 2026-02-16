# generalist_bt_gen

This repository contains a software module which runs a behavior tree for an autonomous mobile robot. Whenever the behavior tree executor detects that the BT in its current form has got limitations and can not complete its mission with the available behaviors it will query an LLM in order for it to generate a new subtree which purpose it is to be expand the domain of the robot. The tree then gets relaunched and the robot can finish its task with the new and improved BT guiding its action.

The general idea is to have a given set of BT skills that call ROS 2 services/actions, while a **mission coordinator** node outside the tree talks to the LLM (via `llm_interface`) to decide **which** subtree/entry point should run next. It also triggers tree regeneration when the skill catalog is insufficient. The BT itself remains a pure execution layer.

The way to interact with the system is by chat, either via CLI or via a simple web interface; both talk to the mission coordinator action server.

All "Thinking" (LLM calls, plan validation, regeneration) now happens **outside** the BT. The coordinator gathers context (cameras, GPS, satellite map) via `context_gatherer`, queries LangChain, and then instructs `bt_executor` which subtree to execute. Blackboard updates produced by the LLM are injected before the BT run; no dedicated LLM plugins live inside the tree anymore.

## Installation

### Prerequisites

- **OS**: Ubuntu 24.04 LTS (or compatible)
- **ROS 2**: Jazzy Jalisco
- **Python**: 3.12+

### 1. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation guide: https://docs.ros.org/en/jazzy/Installation.html

```bash
# After installing ROS 2, source it
source /opt/ros/jazzy/setup.bash
```

### 2. Clone the Repository

```bash
git clone <repository-url>
cd generalist_bt_gen
```

### 3. Install Python Dependencies

The project requires several Python packages that are not part of the standard ROS 2 installation:

```bash
# Install Python dependencies for the LLM interface and web UI
pip3 install fastapi uvicorn jinja2 rich langchain langchain-core langchain-google-genai --break-system-packages
```

**Installed packages:**
- `fastapi` (0.129.0+) - Web framework for the browser UI
- `uvicorn` (0.40.0+) - ASGI server for FastAPI
- `jinja2` - Template engine for HTML rendering
- `rich` - Terminal formatting for CLI chat
- `langchain` (1.2.10+) - LLM orchestration framework
- `langchain-core` (1.2.13+) - Core LangChain components
- `langchain-google-genai` (4.2.0+) - Google Gemini integration

> **Note**: On Ubuntu 24.04 and later, pip requires `--break-system-packages` flag when installing outside a virtual environment. ROS 2 Jazzy on Ubuntu 24.04 uses the system Python.

### 4. Build the Workspace

```bash
# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update

# Install ROS 2 package dependencies
rosdep install --from-paths src --ignore-src -y

# Build all packages
source /opt/ros/jazzy/setup.bash
colcon build
```

### 5. Source the Workspace

```bash
source install/setup.bash
```

### 6. Verify Installation

Test that the Python nodes can be found:

```bash
# Check if ROS 2 nodes are available
ros2 pkg list | grep -E "(user_interface|llm_interface|mission_coordinator|bt_executor)"

# Test Python imports
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -c "import fastapi; import langchain; from langchain_google_genai import ChatGoogleGenerativeAI; print('✅ All dependencies OK')"
```

## Simulation Setup (Clearpath Husky A200)

For testing behavior trees without physical hardware, use the Clearpath Husky A200 simulation with Gazebo Harmonic.

### Install Husky Simulation

```bash
# Install from apt (recommended)
sudo apt-get update
sudo apt-get install ros-jazzy-clearpath-simulator

# Or source install (see docs/husky_simulation_setup.md)
```

### Launch with tmux (Multi-Panel)

We provide a tmux script to manage multiple ROS nodes simultaneously:

```bash
# Launch all nodes in separate tmux panes
./scripts/tmux_ros_launch.sh

# Then attach to the session
./scripts/tmux_ros_launch.sh attach

# When done: ./scripts/tmux_ros_launch.sh kill
```

This creates a session with 5 windows:
- **sim** - Husky simulation (Gazebo Harmonic)
- **bt_stack** - Behavior tree executor + coordinator
- **web_ui** - Web UI monitor
- **teleop** - Keyboard control
- **logs** - ROS topic/node inspection

Learn more: [Using tmux with ROS](./docs/tmux_ros_guide.md)

## Dependencies / Tech stack

- BehaviorTree.CPP as Behavior Tree framework
- ROS 2 Jazzy
- BehaviorTree.ROS2 as behavior tree wrapper
- **Gazebo Harmonic** - 3D simulator (via ros-jazzy-ros-gz)
- **Clearpath Husky A200** - Simulated robot platform
- LangChain to use PromptTemplates and Tool Calls / MCP servers
- OpenAI API / Gemini API / Claude API / Gemini Robotics 1.5 https://ai.google.dev/gemini-api/docs/robotics-overview
- MCP Servers:
  - ROS2 MCP server https://github.com/robotmcp/ros-mcp-server
  - OpenStreetMapMCP https://github.com/wiseman/osm-mcp

## Repo structure

- project_roadmap.md -> currently working on v0.5
- bt_executor -> extends BehaviorTree.ROS2 with needed functionality
- robot_actions -> robot specific behavior tree actions like Drive, TakePicture etc.
- mission_coordinator -> orchestrates LLM planning + bt_executor goals
- llm_interface/context_gatherer -> LangChain

### Possible scenarios where this approach can outperform current methods of behavior tree generation and hence show higher levels of deliberation for the robot:

- Give commands like
  - "Drive around the lake and take photos every 10m" - unclear without context which lake
  - "Explore this field and take pictures of all solitary standing trees." - unclear which field, how to explore, finding solitary trees
  - "Cover the whole field and search for a lost key" - coverage path planning, search mission
- Situation specific recoveries like a stuck robot - can't proceed to drive with current plan, trying to find an alternative path with visual context of sorrounding and satellite map
- Can give always suggestions or even create additions to the skill catalogue of composable bt actions
