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

# v0.6

`context_gatherer` node is implemented with the following features:
- gets called in order to populate the context for payload generation by llm_interface for a chosen subtree
- the context_gatherer action call should have a list of needed context that should be retrieved

# v1.0

Full integration milestone - reliable mission execution with real robot actions and context:

- **Real sensor integration**: `context_gatherer` captures actual sensor data (camera images, GPS/pose, point clouds, semantic maps)
- **Intelligent payload generation**: `llm_interface` uses gathered context to generate appropriate blackboard payloads via LangChain
- **Robot actions validated**: All `robot_actions` plugins tested in simulation (Gazebo/Isaac Sim) with Nav2 integration
- **Operator-in-the-loop**: Operator approval flow (`OperatorDecision` service) works end-to-end with pending plan visualization
- **Mission logging**: Complete transcript logging for debugging and auditing (JSONL format)
- **Multi-tree execution**: Mission coordinator can chain multiple subtrees based on LLM planning
- **Graceful failure handling**: Failed subtrees properly report errors back to coordinator with context

# v2.0
Self expanding trees are able active 
When the robot encounters a unknown situation it can self-repair via the bt_updater after getting context for the behavior tree generation


