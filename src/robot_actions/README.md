# robot_actions

This pkg contains the source files and exports a BT.CPP library of specific robot actions from which behavior trees can be built for specific robots. The nodes use BT.CPP version 4.8.2. 
ROS - related nodes will be making use of the boilerplate classes in behaviortree_ros2 to make action and service calls and to subscribe to topics

## List of available actions

| Action Class Name | Type | Ports | Description | 
| --- | --- | --- | --- |
| MoveTo | `RosActionNode<nav2_msgs::action::NavigateToPose>` | `pose` (`"x,y,theta"`) | Sends NavigateToPose goals to Nav2 |
| TakePicture | `RosServiceNode<std_srvs::srv::Trigger>` | `filepath` (output) | Invokes camera capture service and returns saved path |
| LogTemperature | `RosServiceNode<std_srvs::srv::Trigger>` | `logfile_path` (input) | Requests a temperature capture/logging service |
| RestartNode | `RosServiceNode<std_srvs::srv::SetBool>` | `node_name` (input) | Asks supervisor service to restart the specified node |
