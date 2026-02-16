#!/bin/bash
# tmux_ros_launch.sh - Launch ROS nodes in tmux panes for easy management
# Usage: ./tmux_ros_launch.sh [kill|attach]

SOCKET_DIR="${TMPDIR:-/tmp}/ros-tmux-sockets"
mkdir -p "$SOCKET_DIR"
SOCKET="$SOCKET_DIR/husky_bt.sock"
SESSION="husky_bt"
WORKSPACE="${HOME}/generalist_bt_gen"
CLEARPATH_WS="${HOME}/clearpath_ws"

if [ "$1" = "kill" ]; then
    echo "=== Killing tmux session: $SESSION ==="
    tmux -S "$SOCKET" kill-server 2>/dev/null && echo "Session killed." || echo "No session found."
    exit 0
fi

if [ "$1" = "attach" ]; then
    echo "=== Attaching to session: $SESSION ==="
    tmux -S "$SOCKET" attach -t "$SESSION"
    exit 0
fi

# Kill existing session
tmux -S "$SOCKET" kill-server 2>/dev/null

echo "=== Creating tmux session: $SESSION ==="
echo "Socket: $SOCKET"
echo ""

# Create base session
tmux -S "$SOCKET" new-session -d -s "$SESSION" -n "sim"

# Window 1: Gazebo Simulation (Husky)
tmux -S "$SOCKET" rename-window -t "$SESSION:0" "sim"
tmux -S "$SOCKET" send-keys -t "$SESSION:0.0" "source /opt/ros/jazzy/setup.bash" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:0.0" "source ${CLEARPATH_WS}/install/setup.bash 2>/dev/null || true" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:0.0" "echo '=== Launching Husky Simulation ==='" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:0.0" "ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml" Enter

# Window 2: generalist_bt_gen Stack
tmux -S "$SOCKET" new-window -t "$SESSION:1" -n "bt_stack"
tmux -S "$SOCKET" send-keys -t "$SESSION:1" "source /opt/ros/jazzy/setup.bash" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:1" "source ${WORKSPACE}/install/setup.bash" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:1" "echo '=== Launching BT Stack ==='" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:1" "ros2 launch generalist_bringup generalist_bringup.launch.py" Enter

# Window 3: Web UI Access
tmux -S "$SOCKET" new-window -t "$SESSION:2" -n "web_ui"
tmux -S "$SOCKET" send-keys -t "$SESSION:2" "echo '=== Web UI ==='" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:2" "echo 'Open browser to: http://localhost:8080'" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:2" "${WORKSPACE}/scripts/watch_webui.sh 2>/dev/null || sleep 9999" Enter

# Window 4: Teleop / Testing
tmux -S "$SOCKET" new-window -t "$SESSION:3" -n "teleop"
tmux -S "$SOCKET" send-keys -t "$SESSION:3" "source /opt/ros/jazzy/setup.bash" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:3" "echo '=== Teleop Commands ==='" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:3" "echo 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:3" "# Or test with: ros2 topic pub /cmd_vel geometry_msgs/TwistStamped ..." Enter

# Window 5: Logs / Debug
tmux -S "$SOCKET" new-window -t "$SESSION:4" -n "logs"
tmux -S "$SOCKET" send-keys -t "$SESSION:4" "echo '=== ROS Logs / Debug ==='" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:4" "ros2 topic list" Enter
tmux -S "$SOCKET" send-keys -t "$SESSION:4" "ros2 node list" Enter

echo "=== Session created ==="
echo ""
echo "To attach:     ./tmux_ros_launch.sh attach"
echo "To kill:       ./tmux_ros_launch.sh kill"
echo "Panels:"
echo "  0:sim    - Husky simulation (Gazebo)"
echo "  1:bt_stack - BT Executor stack"
echo "  2:web_ui   - Web UI monitor"
echo "  3:teleop   - Manual control / testing"
echo "  4:logs     - ROS topic/node inspection"
echo ""

# Optional: attach immediately
# tmux -S "$SOCKET" attach -t "$SESSION"
