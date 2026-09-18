# Source this file before using ROS 2 with the Blueboat.
source /opt/ros/jazzy/setup.bash
source /home/luke/generalist_bt_gen/install/setup.bash
source /home/luke/miniconda3/etc/profile.d/conda.sh
conda activate ros2jazzy

export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="enx00e04c200950" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null
