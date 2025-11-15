# v0.5

The CLI/webendpoint to send commands to the behavior tree works
The mission coordinator can send queries to an API endpoint and receives the subtree to execute back
The subtree is then executed or the tree exits with a failure message
No self-update implemented
Only sending commands in natural language to the robot

Needed packages:
- `bt_executor` - BehaviorTree.ROS2 server that drives ticks and hands failures to the LLM pipeline.
- `chat_interface` - CLI client that sends natural-language commands and shows action feedback.
- `mission_coordinator` - ROS node that handles LLM planning, context capture, and calls into `bt_executor`.
- `robot_actions` - Robot action nodes to execute 

# v0.8

the tree can expand itself with the failure context but not environment or ros-context


# v1.0
Self expanding trees are able active 
When the robot encounters a unknown situation it can self-repair via the bt_updater after getting context for the behavior tree generation


