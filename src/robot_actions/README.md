# robot_actions

This pkg contains the source files and exports a BT.CPP library of specific robot actions from which behavior trees can be built for specific robots. The nodes use BT.CPP version 4.8.2. 
ROS - related nodes will be making use of the boilerplate classes in behaviortree_ros2 to make action and service calls and to subscribe to topics

## List of available actions

| Action Class Name | Type | Ports | Description | 
| --- | --- | --- | --- |
| MoveTo | AsyncAction | InputPort: pose (string convert to X,Y, Yaw) | Calls the Nav2 action server |
| TakePicture | AsyncAction | OutputPort: filepath (string) | Calls a service to save a picture | 
| LogTemperature | AsyncAction | InputPort: logfile_path (string) | Calls a service to measure the temp and log it to a file |
| RestartNode | AsyncAction | InputPort: node_name (string) | Triggers a restart of a specific node to regain functionality | 