# `mission_coordinator` package

The mission_coordinator is responsible for sending the user request coming from the web/chat UI via a action call to a LLM, provide context of available sensor sources and information about available robot action behavior trees. 
After receiving a behavior tree decision which tree to execute the mission coordinator calls the `bt_executor` action server with the respective tree to call. The payload is needed context for the execution of the tree, this could for example be waypoints or a tree species which is the target to photograph. The payload will be parsed from JSON to populate the blackboard of the `bt_executor` for execution. 

The mission coordinator should have the possibility to be somewhat configurable during run-time with dynamic reconfigure by the user interface. E.g. the LLM provider should be changeable by the UI, it should exist an option to toggle if the mission_coordinator waits for an acknowledge by the user before sending an action goal to execute a BT. 

