# Using tmux with ROS

This guide explains how to use tmux to manage multiple ROS nodes effectively.

## Why tmux?

- **Persistence**: Sessions survive SSH disconnections
- **Multiple panes**: View logs from different nodes simultaneously
- **Easy control**: Switch between windows, scroll history, copy logs
- **Clean startup**: Launch your entire stack with one command

## tmux Cheat Sheet

| Key | Action |
|-----|--------|
| `Ctrl+b d` | Detach from session (leave running) |
| `Ctrl+b n` | Next window |
| `Ctrl+b p` | Previous window |
| `Ctrl+b <number>` | Go to window N |
| `Ctrl+b "` | Split horizontally |
| `Ctrl+b %` | Split vertically |
| `Ctrl+b [` | Enter scroll mode (use arrows, `q` to exit) |

## Our tmux Launch Script

We provide `scripts/tmux_ros_launch.sh` to start a pre-configured session:

### Start the Stack

```bash
./scripts/tmux_ros_launch.sh
```

Output:
```
=== Session created ===
To attach:     ./tmux_ros_launch.sh attach
To kill:       ./tmux_ros_launch.sh kill
Panels:
  0:sim    - Husky simulation (Gazebo)
  1:bt_stack - BT Executor stack
  2:web_ui   - Web UI monitor
  3:teleop   - Manual control / testing
  4:logs     - ROS topic/node inspection
```

### Attach to the Session

```bash
./scripts/tmux_ros_launch.sh attach
```

You're now in tmux! Use `Ctrl+b n` to switch windows.

### Detach (Keep Running)

Press `Ctrl+b d` - the session continues running in the background.

### Kill All Sessions

```bash
./scripts/tmux_ros_launch.sh kill
```

## Manual tmux Commands

If you want to create your own sessions:

```bash
# Create a new session
SOCKET="/tmp/ros.sock"
tmux -S "$SOCKET" new-session -d -s my_ros -n "gazebo"

# Send commands to panes
tmux -S "$SOCKET" send-keys -t my_ros:0.0 "ros2 launch clearpath_gz empty_launch.py" Enter

# Create a new window
tmux -S "$SOCKET" new-window -t my_ros:1 -n "bt"

# List sessions
tmux -S "$SOCKET" list-sessions

# Attach
tmux -S "$SOCKET" attach -t my_ros
```

## Tips

### Check If Nodes Are Running

Switch to window 4 (logs) and run:
```bash
watch -n 1 'ros2 node list'
```

### Monitor Topics

```bash
# In logs window
ros2 topic echo /scan
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped '{twist: {linear: {x: 0.5}}}'
```

### Scroll Through Logs

Press `Ctrl+b [` (scroll mode), use arrows or PgUp/PgDn, press `q` to exit.

### Resize Panes

Press `Ctrl+b` then arrow keys to resize.

## Troubleshooting

### "Can't create socket"

Make sure the socket directory exists:
```bash
mkdir -p /tmp/ros-tmux-sockets
```

### "Session already exists"

Either attach to it or kill it:
```bash
./scripts/tmux_ros_launch.sh kill
```

### Logs are too verbose

In logs window, set log level:
```bash
ros2 service call /rosout/set_logger_level rcl_interfaces/SetLoggerLevel '{logger_name: bt_executor, level: WARN}'
```

## See Also

- `scripts/tmux_ros_launch.sh` - Our launch script
- [Clearpath Simulation Docs](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)
- [tmux User Guide](https://github.com/tmux/tmux/wiki)
