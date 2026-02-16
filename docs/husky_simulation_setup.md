# Husky A200 Simulation Setup

This document describes how to set up the Clearpath Husky A200 simulation with Gazebo Harmonic for ROS 2 Jazzy.

## Prerequisites

- ROS 2 Jazzy installed
- Gazebo Harmonic (installed via `ros-jazzy-ros-gz`)

## Installation

### Option 1: Binary Install (Recommended)

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-clearpath-simulator
```

### Option 2: Source Install

```bash
# Create workspace
mkdir ~/clearpath_ws/src -p

# Import dependencies
source /opt/ros/jazzy/setup.bash
sudo apt install python3-vcstool

cd ~/clearpath_ws
wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/jazzy/dependencies.repos
vcs import src < dependencies.repos
rosdep install -r --from-paths src -i -y

# Build packages
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Launch Empty World with Husky

```bash
source /opt/ros/jazzy/setup.bash
source ~/clearpath_ws/install/setup.bash  # if using source install

# Launch with Husky A200
ros2 launch clearpath_gz empty_launch.py robot_config_yaml:=husky_a200_sample.yaml
```

### Available Robot Configurations

- `husky_a200_sample.yaml` - Husky A200 with default sensors
- Custom configs can be placed in `~/clearpath_config/`

## Integration with generalist_bt_gen

Once the simulation is running, the behavior tree executor can control the simulated Husky via:

- `/cmd_vel` - Velocity commands
- `/odom` - Odometry feedback
- `/scan` - LiDAR data
- `/camera` - Camera images (if configured)

Test with teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Troubleshooting

### No GPU acceleration
The simulator works without GPU but performance will be reduced.

### Controller not working
See [Xbox Controller Setup](#) in Useful Resources.
