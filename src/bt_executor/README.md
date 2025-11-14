# BT_executor 

This package extends behaviortree.ros2 `TreeExecutionServer` in its own class called `GeneralistBehaviorTreeServer`. We assume for now that the User Command is send via the action goal call. How this information then gets used and processed inside the tree to be written on the blackboard remains as open TODO in this package.

Overriding the following methods with the following purpose:

| Method | Purpose |
| --- | --- |
| `void onTreeCreated(BT::Tree& tree)` | set the goal payload of the action call goal to "user_command" on the blackboard | 
| `std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status)` | t.b.d | 
| `void onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)` | If the tree failed we trigger the extension of the tree here by returning a custom error message to the action client | 
| `std::optional<std::string> onLoopFeedback()` | The feedback is used to display information in the user interface on the webpage, active node for example, blackboard status stuff like this | 

--- 

### ROS node

bt_executor_node -> Creates an instance of the Action Server Class and spins it

## Action Plugin import and BT import

We import the actions from robot_actions and llm_actions (configured via action_server parameters) the behavior tree and its subtree live inside this package in trees/