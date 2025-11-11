# v0.5

The CLI/webendpoint to send commands to the behavior tree works
The thinking node can send queries to an API endpoint and recieves the subtree to execute back
The subtree is then executed or the tree exits with a failure message
No self-update implemented
Only sending commands in natural language to the robot

Needed packages:
- `bt_executor` – BehaviorTree.ROS2 server that drives ticks and hands failures to the LLM pipeline.
- `chat_interface` – CLI client that sends natural-language commands and shows action feedback.
- `llm_actions` – Thinking/decision nodes inside the BT that call the LangChain service and inject returned subtrees.

# v0.8

the tree can expand itself with the failure context but not environment or ros-context

# v0.9
Self expanding trees are able active 
When the robot encounters a unknown situation it can self-repair via the bt_updater after getting context for the behavior tree generation



