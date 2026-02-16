# Gazebo Harmonic + Clearpath Husky A200 Setup

## Prerequisites

- ROS 2 Jazzy installed
- generalist_bt_gen workspace built

## Installation

### 1. Add Gazebo Package Repository

```bash
# Add Gazebo signing key
sudo curl https://packages.osrfoundation.org/gazebo.gpg | sudo tee /etc/apt/keyrings/00-gazebo.gpg

# Add Gazebo repository
sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/00-gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

sudo apt-get update
```

### 2. Install Gazebo Harmonic

```bash
# Install Gazebo Harmonic + ROS-Gazebo bridge
sudo apt-get install -y gz-harmonic ros-jazzy-ros-gz
```

### 3. Install Clearpath Husky Packages

```bash
# Install Husky simulation packages
sudo apt-get install -y \
    ros-jazzy-clearpath-common \
    ros-jazzy-clearpath-platform \
    ros-jazzy-clearpath-gz
```

### 4. Build Workspace

```bash
cd ~/generalist_bt_gen
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Launch Husky Simulation

### Basic Husky in Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source ~/generalist_bt_gen/install/setup.bash

# Launch Husky in Gazebo
ros2 launch clearpath_gz simulation.launch.py
```

### Custom Husky with generalist_bt_gen

Create a custom launch file:

```python
# src/generalist_bringup/launch/husky_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include Clearpath Gazebo launch
    clearpath_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clearpath_gz'),
                'launch',
                'simulation.launch.py'
            )
        )
    )
    
    # Add generalist_bt_gen nodes
    bt_executor_node = Node(
        package='bt_executor',
        executable='bt_executor_node',
        output='screen',
    )
    
    mission_coordinator_node = Node(
        package='mission_coordinator',
        executable='mission_coordinator_node',
        output='screen',
    )
    
    return LaunchDescription([
        clearpath_gz_launch,
        bt_executor_node,
        mission_coordinator_node,
    ])
```

## Verification

1. **Check Gazebo is running:**
   ```bash
   gz sim --version
   ```

2. **List Husky topics:**
   ```bash
   ros2 topic list | grep husky
   ```

3. **Visualize in RViz:**
   ```bash
   ros2 run rviz2 rviz2
   ```

## Troubleshooting

- **"gz: command not found"**: Ensure `/usr/bin` is in PATH and Gazebo is installed
- **Husky not spawning**: Check `clearpath_gz` package is installed: `ros2 pkg list | grep clearpath`
- **Robot not moving**: Verify `/cmd_vel` topic exists and is being published

## Integration with generalist_bt_gen

The Husky A200 provides:
- Differential drive base (navigate via `/cmd_vel`)
- GPS sensor (for outdoor localization)
- LiDAR (for obstacle avoidance)
- Camera (for visual context gathering)

These integrate with:
- `context_gatherer`: Reads GPS, camera, LiDAR data
- `robot_actions`: Drive, TakePicture actions
- `bt_executor`: Executes behavior trees on simulated robot