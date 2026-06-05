# robot_actions

This pkg contains the source files and exports a BT.CPP library of specific robot actions from which behavior trees can be built for specific robots. The nodes use BT.CPP version 4.8.2. 
ROS - related nodes will be making use of the boilerplate classes in behaviortree_ros2 to make action and service calls and to subscribe to topics

## List of available actions

| Action Class Name | Type | Ports | Description | 
| --- | --- | --- | --- |
| MoveTo | `RosActionNode<nav2_msgs::action::NavigateToPose>` | `pose` (`"x,y,theta"`) | Sends NavigateToPose goals to Nav2 |
| TakePicture / TakePhoto | `SyncActionNode` | `image_topic`, `output_directory`, `filename_prefix`, `timeout_ms`, `filepath` (output) | Saves the latest RGB image from a configurable topic to disk |
| FindObjectLocation / FindAnything | `RosServiceNode<language_feature_msgs::srv::FindObjectLocations>` | `object`, `max_results`, `default_yaw`, `x`/`y`/`yaw`/`pose`/`frame_id` (outputs) | Calls FindAnything and exposes the first returned object point as a 2D pose |
| LogTemperature | `RosServiceNode<std_srvs::srv::Trigger>` | `logfile_path` (input) | Requests a temperature capture/logging service |
| RestartNode | `RosServiceNode<std_srvs::srv::SetBool>` | `node_name` (input) | Asks supervisor service to restart the specified node |
| DistanceTraveled | `StatefulActionNode` | `interval_m`, `odom_topic`, `odom_timeout_ms`, `distance_accumulated_m` (output) | Returns `SUCCESS` whenever the configured odometry distance interval has been traveled, otherwise `RUNNING` |
